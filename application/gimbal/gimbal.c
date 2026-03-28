#include "stdio.h"

#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "DMmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"
#include "referee_UI.h"
#include "controller.h"

static INS_Instance *gimbal_IMU_data; // ???IMU????
DJIMotorInstance *yaw_motor;
DMMotorInstance *pitch_motor;
DJIMotorInstance *pitch_version;
static Publisher_t *gimbal_pub;                   // ???????????????(?????????cmd)
static Subscriber_t *gimbal_sub;                  // cmd?????????????
static Gimbal_Upload_Data_s gimbal_feedback_data; // ?????cmd??????????
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // ????cmd????????
float pitch_current_feedforward,K_pitch_current_feedforward,B_pitch_current_feedforward;
static float pitch_speed_feedforward = 0;
//static float yaw_angle_imu,yaw_gyro_imu;
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
float yaw_gyro_twoboard=0,yaw_current_feedforward;
const float pitch_offset = 7.7f;
float pitch_tor_feedforward = 0;
float pitch_gyro_measure = 0;
// static PID_Setting_s pitch_settings,yaw_settings;
// extern float imu_angle[3];
// extern float imu_gyro[3];
float currentFromAngle(float x){
    float y=-0.0011*pow(x,5)+0.0623*pow(x,4)-1.2218*pow(x,3)+7.4649*pow(x,2)-42*x+6776;

    return -y;
}
static float yaw_speedFeed;
uint16_t powerLim;
static float getYawSpeedFeed(uint16_t power){
    switch (power)
    {
    case 70:
        return 1.25;
        break;
        case 75:
        return 1.25;
        break;
        case 80:
        return 1.4;
        break;
        case 85:
        return 1.4;
        break;
        case 90:
        return 1.4;
        break;
        case 95:
        return 1.4;
        break;
        case 100:
        return 1.5;
        break;
        case 105:
        return 1.5;
        break;
        case 110:
        return 1.5;
        break;
        case 120:
        return 1.6;
        break;
        case 200:
        return 1.7;
        break;
    default:return 0;
        break;
    }
}
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
            // ?????????????????????,?????,ins_task.md????c???bodyframe????????
            .other_speed_feedback_ptr = &chassis_rs485_recv.yaw_gyro,//&yaw_gyro_twoboard,//&chassis_rs485_recv.yaw_gyro-&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],//,&gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
            // .current_feedforward_ptr=&yaw_current_feedforward,
            //.speed_feedforward_ptr=&yaw_speedFeed,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
            // .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            // .feedforward_flag  =CURRENT_FEEDFORWARD,
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
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
        // .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,

    };
    gimbal_IMU_data = INS_Init(BMI088Register(&config)); // IMU??????,???????????????yaw????????????????
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
                .Kp = 12.5,
                .Ki = 0,
                .Kd = 0.01,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 0.91,
                .Ki = 9.81,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 1.5,
                .MaxOut = 2,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch?????
             .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[0],  //等待修改   
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[0],
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
//     .DeadBand      = 0,  // 0.75,  //?????????????????????????
//     .IntegralLimit = 3000,
//     // .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//     .MaxOut        = 30,

// };
// static PIDInstance PITCH_version_PID = {
//     .Kp            = 1,   // 25,//25, // 50,//70, // 4.5
//     .Ki            = 0,    // 0
//     .Kd            = 0, // 0.0,  // 0.07,  // 0
//     .DeadBand      = 0,  // 0.75,  //?????????????????????????
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
float yaw_feedforward_angleErrmax=10,yaw_feedforward_max=3200; // ???err???????????
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
    // ????????��???????????
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
    // ????????????��???????��??????,???????robot_cmd??????????��?,gimbal???yaw_ref??pitch_ref
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
            // DJIMotorStop(yaw_motor);
            // DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP, OTHER_FEED);
            // DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            if (gimbal_cmd_recv.nuc_mode == version_control)
            {
                float error = gimbal_cmd_recv.yaw_version - *yaw_motor->motor_controller.other_angle_feedback_ptr;
                if (error > 180.0f)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr - 360.0 + error;
                }
                else if (error < -180.0)
                {
                    gimbal_cmd_recv.yaw_version = *yaw_motor->motor_controller.other_angle_feedback_ptr + (360 + error);
                }
                DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw_version);
            }
            else
            {
                DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            }
            break;
        case GIMBAL_MOTOR_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor,ANGLE_LOOP,MOTOR_FEED);
            DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw??pitch????robot_cmd?��???????????
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
    

    //yaw_current_feedforward = YawFeedForwardCalculate(yaw_motor->motor_controller.angle_PID.Err);
    #endif
    #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    #ifndef pitch_motor->motor_controller.angle_PID.IntegralLimit = pitch_PID.gyro_mode.angle_PID.IntegralLimit;
    // pitch_tor_feedforward = 0.42146 * cos(1.43 + gimbal_IMU_data->output.INS_angle[0]);
    // pitch_tor_feedforward = 
    // pitch_current_feedforward = PitchNonlinear(*pitch_motor->motor_controller.other_angle_feedback_ptr);
   
    pitch_gyro_measure =  gimbal_IMU_data->INS_data.INS_gyro[1];
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
    pitch_current_feedforward=-exp((352-gimbal_IMU_data->output.INS_angle_deg[0])/40);
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
    
    // ???��???????,?????imu??yaw??ecd
    // gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round; // ???????
    // gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
    // gimbal_feedback_data.pitch_ecd                    = pitch_motor->measure.ecd;
    // gimbal_feedback_data.yaw_total_angle              = yaw_motor->measure.total_angle;
    // gimbal_feedback_data.pitch_total_angle            = pitch_motor->measure.total_angle;
    // ???????
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
