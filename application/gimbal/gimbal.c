/*------------------------------------------------------------------------------*/
#include "stdio.h"//引入基础头文件
#include <math.h>
/*------------------------------------------------------------------------------*/
#include "gimbal.h"//拿到本模块对外接口 GimbalInit(),GimbalTask()      
#include "robot_board.h"//根据 CHASSIS_BOARD / ONE_BOARD 决定编译哪部分代码
#include "robot_params.h"//读云台几何参数、限位、控制宏定义等
#include "robot_types.h"//
/*------------------------------------------------------------------------------*/
#include "dji_motor.h"//分别给四个轮毂电机和两个关节电机提供驱动接口
#include "DMmotor.h"//DM电机的接口
/*------------------------------------------------------------------------------*/
#include "ins_task.h"//使用 IMU/INS 解算结果
/*------------------------------------------------------------------------------*/
#include "message_center.h"//云台和上层命令模块通过发布/订阅通信
/*------------------------------------------------------------------------------*/
#include "general_def.h"//一些module的通用数值型定义
/*------------------------------------------------------------------------------*/
#include "bmi088.h"//IMU传感器接口
#include "referee_UI.h"//裁判系统数据解析，提供给UI显示用
#include "controller.h"//PID控制器实现
/*------------------------------------------------------------------------------*/

static INS_Instance *gimbal_IMU_data;               //保存云台 IMU/INS 实例指针
DJIMotorInstance *yaw_motor;                        //云台 yaw 轴的 DJI 电机对象
DMMotorInstance *pitch_motor;                       //云台 pitch 轴的 DM 电机对象
DJIMotorInstance *pitch_version;                    //云台 pitch 轴的 DJI 电机对象(未使用)
static Publisher_t *gimbal_pub;                     //用于发布云台的数据
static Subscriber_t *gimbal_sub;                    //用于订阅云台的控制命令
static Gimbal_Upload_Data_s gimbal_feedback_data;   //云台要回传给别的模块的数据
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;           //云台接收到的控制命令
static float pitch_speed_feedforward = 0;           //俯仰轴速度前馈
static float yaw_speedFeed;
//static float yaw_angle_imu,yaw_gyro_imu;
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;  //表示从另一块板/串口收到的数据
static float yaw_speed_feedforward = 0;             //yaw轴速度前馈
static float filtered_yaw_vel = 0;                  //保存滤波后的视觉 yaw 速度
static const float yaw_vel_filter_alpha = 0.12f;    //一阶低通滤波系数，数值越小滤波效果越明显，但响应越慢

static const float yaw_vel_deadzone = 0.1f;         //yaw 速度死区
extern float vision_yaw_vel;                        //视觉给出的 yaw 速度
static float yaw_feedforward_vel_gain = -0.8f;      //定义 yaw 速度前馈增益
const float pitch_offset = 7.7f;                    //作为 pitch 偏置量
float yaw_gyro_twoboard=0,yaw_current_feedforward;  //yaw轴电流前馈
float pitch_current_feedforward,K_pitch_current_feedforward,B_pitch_current_feedforward;//俯仰轴电流前馈的 PID 参数
float pitch_tor_feedforward = 0;                    //保存 pitch 力矩前馈
float pitch_gyro_measure = 0;                       //保存 pitch 轴陀螺仪测量值
// static PID_Setting_s pitch_settings,yaw_settings;//保存 pitch 和 yaw 的 PID 设置参数
// extern float imu_angle[3];                       //IMU 角度数组
// extern float imu_gyro[3];                        //IMU 角速度数组
/*--------------------------------------------------------------------------------------------------------------------------*/
//currentFromAngle():用于把角度映射成电流值
float currentFromAngle(float x){
    float y=-0.0011*pow(x,5)+0.0623*pow(x,4)-1.2218*pow(x,3)+7.4649*pow(x,2)-42*x+6776;

    return -y;
}
/*--------------------------------------------------------------------------------------------------------------------------*/

uint16_t powerLim;//根据电池电量计算出的功率限制值，单位为百分比
// static float getYawSpeedFeed(uint16_t power){
//     switch (power)
//     {
//     case 70:
//         return 1.25;
//         break;
//         case 75:
//         return 1.25;
//         break;
//         case 80:
//         return 1.4;
//         break;
//         case 85:
//         return 1.4;
//         break;
//         case 90:
//         return 1.4;
//         break;
//         case 95:
//         return 1.4;
//         break;
//         case 100:
//         return 1.5;
//         break;
//         case 105:
//         return 1.5;
//         break;
//         case 110:
//         return 1.5;
//         break;
//         case 120:
//         return 1.6;
//         break;
//         case 200:
//         return 1.7;
//         break;
//     default:return 0;
//         break;
//     }
// }
/*------------------------------------------------------------------------云台初始化函数---------------------------------------------------------------------------------------*/
void GimbalInit()
{
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // YAW
    
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 3,//1.7,//1.8,//0.6, // 0.24, // 0.31, // 0.45
                .Ki            = 1,
                .Kd            = 0.1f,//0.13,//0.07,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 100,
            },
            .speed_PID = {
                .Kp            = 2500, // 18000, // 10500,//1000,//10000,// 11000
                .Ki            = 0,    // 0
                .Kd            = 10,    // 10, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,// | PID_OutputFilter,
                .IntegralLimit = 500,
                .MaxOut        = 10000,//16384,//25000, // 20000
                // .Output_LPF_RC=1,//0.4,
                // .CoefA=0.2,
                // .CoefB=2,//0.3,
            },
            .other_angle_feedback_ptr = &chassis_rs485_recv.yaw_angle,//&gimbal_IMU_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET], // yaw????????
            // ins_task.md bodyframe
            .other_speed_feedback_ptr = &chassis_rs485_recv.yaw_gyro,//&yaw_gyro_twoboard,//&chassis_rs485_recv.yaw_gyro-&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],//,&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &yaw_speed_feedforward,
            .current_feedforward_ptr = &yaw_current_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
            // .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            .feedforward_flag      = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,  //
        },
        .motor_type = GM6020};
        yaw_motor   = DJIMotorInit(&yaw_config);
        #endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    BMI088_Init_Config_s config = {
        .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
        .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
        .heat_pid_config = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .Improve       = PID_IMPROVE_NONE,
            .IntegralLimit = 0.90f,
            .MaxOut        = 0.95f,
        },
        .heat_pwm_config = {
            .htim      = &htim10,
            .channel   = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period    = 5000 - 1,
        },
        .spi_acc_config = {
            .GPIOx      = GPIOA,
            .cs_pin     = GPIO_PIN_4,
            .spi_handle = &hspi1,
        },
        .spi_gyro_config = {
            .GPIOx      = GPIOB,
            .cs_pin     = GPIO_PIN_0,
            .spi_handle = &hspi1,
        },
        //.cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,

    };
    gimbal_IMU_data = INS_Init(BMI088Register(&config)); //初始化云台 IMU/INS 实例
// PITCH
     Motor_Init_Config_s pitch_motor_config = {//DM4310
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x02,
            .rx_id = 0x12,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 4.8,
                .Ki = 0,
                .Kd = 0.015,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.3,
                .MaxOut = 10,
            },
            .speed_PID = {
                .Kp = 1.2,
                .Ki = 0.0,
                .Kd = 0.001,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 0.6,
                .MaxOut = 1.2,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch?????
             .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET],  //缁涘绶熸穱顔芥暭   
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
            .speed_feedforward_ptr = &pitch_speed_feedforward,
            // .current_feedforward_ptr = &pitch_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag      = SPEED_FEEDFORWARD,
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
    };
    pitch_motor = DMMotorInit(&pitch_motor_config);
    DMMotorStop(pitch_motor);

    #endif
    // ?????total_angle???,???????,??????,???????????????
    
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
/*
 * A nonlinear function that maps a value
 * from the range [X_MIN, -X_MAX] to the range [Y_MIN, Y_MAX].
 */
double nonlinear(const double x) {
#define X_MIN (-0.5)
#define X_MAX 0.5
#define Y_MIN (-2700)
#define Y_MAX 3000

    const double scaled = (x - X_MIN) / (X_MAX - X_MIN) * 4.0 - 2.0;
    const double out = (tanh(scaled) + 1.0) / 2.0;
    return out * (Y_MAX - Y_MIN) + Y_MIN;
}

float PitchNonlinear(float x) {
    //return (215.65 * x -3050.0);
    return (186.0 * x - 1292.0);
}

// ???????????
#define NUM_SEGMENTS 11

const double coeffs[NUM_SEGMENTS][4] = {
    {0.8782, 47.2692, -548.1474, 7500.0000},
    {0.8782, 49.9038, -450.9744, 7000.0000},
    {-2.4321, 53.3287, -316.7722, 6500.0000},
    {-2.3856, 8.8205, 62.3377, 6000.0000},
    {34.5316, -56.3055, -369.7766, 5500.0000},
    {-6.0712, 78.3677, -341.0958, 5000.0000},
    {-0.4334, 1.8704, -4.0958, 4500.0000},
    {1.7548, -13.4722, -140.9974, 4000.0000},
    {-3.9207, 20.5351, -95.3710, 3000.0000},
    {5.0001, -41.0982, -203.1216, 2500.0000},
    {5.0001, -12.5977, -305.1439, 2000.0000}
};

const double x_start[NUM_SEGMENTS] = {
    -20.0000, -19.0000, -17.7000, -11.6000, -2.5000, -1.2000, 3.0000, 14.8000, 21.2600, 26.5000, 28.4000,
};

// ??????? x ??? y ?
double evaluate_spline(const double x) {
    if (x < x_start[0]) {
        return coeffs[0][3];
    } else if (x > x_start[NUM_SEGMENTS - 1]) {
        return coeffs[NUM_SEGMENTS - 1][3];
    }

    int i;

    // ??? x ??????
    for (i = 0; i < NUM_SEGMENTS; i++) {
        if (x >= x_start[i] && (i == NUM_SEGMENTS - 1 || x < x_start[i + 1])) {
            break;
        }
    }

    // ???? y ?
    const double dx = x - x_start[i];
    const double y = coeffs[i][0] * dx * dx * dx + coeffs[i][1] * dx * dx + coeffs[i][2] * dx + coeffs[i][3];

    return y;
}

static float YawFeedForwardCalculate(float direction)
{
    if (direction > 1e-6)
    {
        yaw_current_feedforward = 4000;
    }
    else if (direction < -1e-6)
    {
        yaw_current_feedforward = -4000;
    }
    else
    {
        yaw_current_feedforward = 0;
    }
}


void pitch_limit(float angle)
{
    if(angle>PITCH_UP_LIMIT)
        angle=PITCH_UP_LIMIT;
    else if(angle<PITCH_DOWN_LIMIT)
        angle=PITCH_DOWN_LIMIT;
    pitch_motor->motor_controller.pid_ref = angle;
}
// static PIDInstance YAW_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  //
//     .IntegralLimit = 3000,
//     // .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//     .MaxOut        = 30,

// };
// static PIDInstance PITCH_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  //
//     .IntegralLimit = 30,
//     .Improve       = PID_OutputFilter|PID_Derivative_On_Measurement|PID_Integral_Limit|PID_Trapezoid_Intergral/*PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement*/,
//     .MaxOut        = 30,
//     .Output_LPF_RC=0.7,
// };
float nuc_version_control[2];
// float filiter(float input){
//     static float alpha=1,input_last;
//     float output=alpha*input+(1-alpha)*input_last;
//     input_last=output;
//     return output;
// }
float yaw_feedforward_angleErrmax=10,yaw_feedforward_max=3200; //
/* ????????????????????,?????????????IMU????,??????????????? */
// float fpv_pitch_test=0 , telescope_test=0;
uint16_t pitch_test=0;
uint8_t fpv_state=0;//0:rise 1:down 2:ready
uint16_t fpv_count=0;
float fpv_pitch_up=0,fpv_pitch_down=0,telescope_up=0,telescope_down=0;
extern uint8_t telescope_pos ;//0:normal 1:zoom
extern uint8_t fpv_pos ;//0:normal 1:lob
extern float pitch_vel;
void GimbalTask()
{
    // ??????????????
    // ????????閿熸枻鎷???????????
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // yaw_current_feedforward=yaw_feedforward_max*(yaw_motor->motor_controller.angle_PID.Err/yaw_feedforward_angleErrmax);//angleErr??????4000
    // if(yaw_current_feedforward>yaw_feedforward_max)yaw_current_feedforward=yaw_feedforward_max;
    // else if(yaw_current_feedforward<-yaw_feedforward_max)yaw_current_feedforward=-yaw_feedforward_max;

    //------------------------------------------------------------------------
    // gimbal_cmd_recv.pitch_version=filiter(gimbal_cmd_recv.pitch_version);
    // gimbal_cmd_recv.yaw_version=filiter(gimbal_cmd_recv.yaw_version);
    // if(gimbal_cmd_recv.nuc_mode){
    //     nuc_version_control[0]=PIDCalculate(&PITCH_version_PID,gimbal_cmd_recv.pitch_version,0);
    //     nuc_version_control[1]=PIDCalculate(&YAW_version_PID,gimbal_cmd_recv.yaw_version,0);
    // }
    // else memset(nuc_version_control,0,sizeof(nuc_version_control));
    // @todo:???????????????????,??????????????IMU??????????????????????,yaw?????offset??????????????
    // 
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)        
    // yaw_gyro_twoboard=chassis_rs485_recv.yaw_gyro-gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    //yaw_current_feedforward=nonlinear(yaw_motor->motor_controller.angle_PID.Err);
    
    // if(!gimbal_cmd_recv.yaw_speedFeed)
    // yaw_speedFeed=1.25;
    // else yaw_speedFeed=0;
    // if(gimbal_cmd_recv.yaw_kp)yaw_motor->motor_controller.angle_PID.Kp=gimbal_cmd_recv.yaw_kp;
    // else yaw_motor->motor_controller.angle_PID.Kp=1.7;
    // if(gimbal_cmd_recv.yaw_kd)yaw_motor->motor_controller.angle_PID.Kd=gimbal_cmd_recv.yaw_kd;
    // else yaw_motor->motor_controller.angle_PID.Kd=0.13;
    // if(gimbal_cmd_recv.yaw_speedKp)yaw_motor->motor_controller.speed_PID.Kp=gimbal_cmd_recv.yaw_speedKp;
    // else yaw_motor->motor_controller.speed_PID.Kp=3000;
    switch (gimbal_cmd_recv.gimbal_mode) {
        // ??
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);          
            // DJIMotorStop(pitch_motor);
            break;
        // ?????????????,???????yaw?????offset??????????????????
        case GIMBAL_GYRO_MODE: // ?????????????        
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor,SPEED_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                float raw_yaw_vel = 0;
                // 閼奉亞鐎Ο鈥崇础閿涙矮濞囬悽銊潒鐟欏褰佹笟娑氭畱yaw闁喎瀹虫担婊€璐熼崜宥夘洯
                #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
                    // 閸楁洘婢橀幋鏉噄mbal閺夋寧膩瀵骏绱版禒宥禨B閹恒儲鏁归惃鍕潒鐟欏鏆熼幑顔垮箯閸欐潾aw闁喎瀹?
                    raw_yaw_vel = vision_yaw_vel;
                #elif defined(CHASSIS_BOARD)
                    // 閸欏本婢樻惔鏇犳磸濡€崇础閿涙矮绮燫S485閹恒儲鏁归惃鍒mbal閺夋寧鏆熼幑顔垮箯閸欐潾aw闁喎瀹?
                    raw_yaw_vel = chassis_rs485_recv.yaw_vel;
                #endif

                // 娴ｅ酣鈧碍鎶ゅ▔顫礉閹舵垵鍩楃憴鍡氼潕/闁俺顔嗘稉顓犳畱妤傛﹢顣堕崳顏勶紣
                filtered_yaw_vel += yaw_vel_filter_alpha * (raw_yaw_vel - filtered_yaw_vel);

                // 鐏忓繘鈧喐顒撮崠鐚寸礉闁灝鍘ゅ顔肩毈濞夈垹濮╂禍褏鏁撻崜宥夘洯閹垫澘濮?
                if (fabsf(filtered_yaw_vel) < yaw_vel_deadzone) {
                    filtered_yaw_vel = 0.0f;
                }

                yaw_speed_feedforward = filtered_yaw_vel * yaw_feedforward_vel_gain;

                float yaw_error = gimbal_cmd_recv.yaw_version - *yaw_motor->motor_controller.other_angle_feedback_ptr;
                // 婢跺嫮鎮婄憴鎺戝鐠恒劏绉洪敍鍩?80鎺抽敍?
                if (yaw_error > 180.0f)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr - 360.0 + yaw_error;
                }
                else if (yaw_error < -180.0)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr + (360 + yaw_error);
                }
                
                DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_version);
            }
            else
            {
                // 闂堢偠鍤滈惉鍕佸蹇ョ窗缁備胶鏁ら崜宥夘洯
                yaw_speed_feedforward = 0;
                //yaw_current_feedforward = 0;
                DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            }
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,MOTOR_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw??pitch????robot_cmd?閿熸枻鎷???????????
        break;
        default:
            break;
    }
    


    gimbal_feedback_data.yaw_angle_pidout             =yaw_motor->motor_controller.angle_PID.Output;
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; // ???????
    gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    gimbal_feedback_data.pitch_ecd                    = 0;//pitch_motor->measure.ecd;
    gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    gimbal_feedback_data.pitch_total_angle            = 0;//pitch_motor->measure.total_angle;
    gimbal_feedback_data.yaw_motor_real_current       = (float)yaw_motor->measure.real_current;
    

    //yaw_current_feedforward = YawFeedForwardCalculate(yaw_motor->motor_controller.angle_PID.Err);
    #endif
    #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    #ifndef BIG_HEAD
    // pitch_tor_feedforward = 0.42146 * cos(1.43 + gimbal_IMU_data->output.INS_angle[0]);
    // pitch_tor_feedforward = 
    // pitch_current_feedforward = PitchNonlinear(*pitch_motor->motor_controller.other_angle_feedback_ptr);
   
    pitch_gyro_measure =  gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET];
    switch (gimbal_cmd_recv.gimbal_mode) {
        // ??
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(pitch_motor);
            pitch_motor->motor_controller.speed_PID.Iout = 0;
            pitch_motor->motor_controller.angle_PID.Iout = 0;
             pitch_speed_feedforward = 0;
            break;
        // ?????????????,???????yaw?????offset??????????????????
        case GIMBAL_GYRO_MODE: // ?????????????
            DMMotorEnable1(pitch_motor);

            // DJIMotorChangeFeed(pitch_motor,SPEED_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            // DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
    
            
            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_version);
                pitch_limit(gimbal_cmd_recv.pitch_version);
                pitch_speed_feedforward = pitch_vel;
            }
            else
            {
                // DJIMotorSetRef(pitch_motor,pitch_offset + gimbal_cmd_recv.pitch);
                pitch_limit(gimbal_cmd_recv.pitch);
                pitch_speed_feedforward = 0;
            }
            // DJIMotorSetRef(fpv_pitch_motor,fpv_pitch_test);
            // DJIMotorSetRef(telescope_motor,telescope_test);
           
        
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(pitch_motor);
            // DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            // DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP,MOTOR_FEED);
            // DJIMotorChangeFeed(pitch_motor,SPEED_LOOP,MOTOR_FEED);
       
            DJIMotorSetRef(pitch_motor, pitch_offset + gimbal_cmd_recv.pitch); 
            // DJIMotorSetRef(pitch_motor, pitch_test); 
           
            
        break;
        default:
            break;
    } 
    // pitch_motor->motor_controller.other_speed_feedback_ptr = &pitch_speed_feedforward;
    #endif // !BIG_HEAD
    #ifdef BIG_HEAD
    pitch_current_feedforward=-exp((352-gimbal_IMU_data->output.INS_angle_deg[INS_PITCH_ADDRESS_OFFSET])/40);
    // pitch_motor->motor_controller.speed_PID.MaxOut=16384-fabs(pitch_current_feedforward);
    if(pitch_current_feedforward>-3500)pitch_current_feedforward=-3500;
    else if(pitch_current_feedforward<-6500)pitch_current_feedforward=-6500;
        switch (gimbal_cmd_recv.gimbal_mode) {
        
        // ??
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(pitch_motor);
            break;
        // ?????????????,???????yaw?????offset??????????????????
        case GIMBAL_GYRO_MODE: // ?????????????
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor,ANGLE_LOOP, OTHER_FEED);
            DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
            DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
        default:
            break;
    }
    #endif
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; // ???????
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    
    #endif
    // ????????????pitch???????????????
    // ????IMU???/pitch??????????????????????????????
    // ...
    
    // ???閿熸枻鎷???????,?????imu??yaw??ecd
    // gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; // ???????
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    // ???????
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

