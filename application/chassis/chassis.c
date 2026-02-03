/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "DMmotor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"
#include "remote_control.h"
#include "ins_task.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF               0
#define RF               1
#define RB               2
#define LB               3

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
// #include "can_comm.h"

// static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
//attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef CHASSIS_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Ctrl_Cmd_s_half_float chassis_cmd_recv_half_float;
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

SuperCapInstance *cap;                                              // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
DMMotorInstance *joint_l, *joint_r;
extern referee_info_t *referee_data_for_ui;
 uint16_t power_supdata_watch  ;
volatile uint8_t superCap_watchdog;
// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0

static INS_Instance *Chassis_IMU_data; // 底盘IMU数据
// extern INS_Instance *gimbal_IMU_data; // ???IMU????
extern RC_ctrl_t rc_ctrl[2]; 
// 跟随模式底盘的pid
// 目前没有设置单位，有些不规范，之后有需要再改
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 105,//105,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 1.5, // 0.0,  // 0.07,  // 0
    .DeadBand      = 2.0,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 16384,

};

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;         // 底盘速度解算后的临时输出,待进行限幅
static float kl=1;
extern INS_Instance *INS;
int8_t sign_lf, sign_rf, sign_lb, sign_rb;
static leg_mode_e leg_mode = LEG_ACTIVE_SUSPENSION;
GPIO_InitTypeDef GPIO_InitStruct = {0};
volatile static float joint_tor_feedforward = 0;

static float friction_lf_state = 0.0f;
static float friction_rf_state = 0.0f;
static float friction_lb_state = 0.0f;
static float friction_rb_state = 0.0f;

void ChassisInit()
{
#if defined(CHASSIS_BOARD)
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
        //.cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,
};
    Chassis_IMU_data = INS_Init(BMI088Register(&config)); // IMU??????,???????????????yaw????????????????
#endif
    #if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config_1 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.0
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            // .current_feedforward_ptr = &friction_lf_state,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            // .feedforward_flag       = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s chassis_motor_config_2 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5 , 3.75
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            // .current_feedforward_ptr = &friction_rf_state,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            // .feedforward_flag       = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_3 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            // .current_feedforward_ptr = &friction_rb_state,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            // .feedforward_flag       = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
     Motor_Init_Config_s chassis_motor_config_4 = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3 ,// 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 16000,
            },
            // .current_feedforward_ptr = &friction_lb_state,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
            // .feedforward_flag       = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config_1.can_init_config.tx_id                             = 1;
    chassis_motor_config_1.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config_1);
    
    chassis_motor_config_2.can_init_config.tx_id                             = 2;
    chassis_motor_config_2.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config_2);
    
    chassis_motor_config_3.can_init_config.tx_id                             = 3;
    chassis_motor_config_3.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config_3);

    chassis_motor_config_4.can_init_config.tx_id                             = 4;
    chassis_motor_config_4.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config_4);
    
    Motor_Init_Config_s joint_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 10,
                .Ki = 0.0,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0.5,
                .MaxOut = 5,
            },
            .speed_PID = {
                .Kp = 20,
                .Ki = 0,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
                .IntegralLimit = 0.5,
                .MaxOut = 10,
            },
            //  .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch?????
             .other_angle_feedback_ptr = &Chassis_IMU_data->output.INS_angle[1],  //等待修改   
            // ??????????????,????,ins_task.md??c??bodyframe?????
            .other_speed_feedback_ptr = &Chassis_IMU_data->INS_data.INS_gyro[0],
            .current_feedforward_ptr = &joint_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
            .control_range = {
                .P_max = 12.5,
                .V_max = 10,
                .T_max = 28,
            },
        },
    };
    
    joint_motor_config.can_init_config.tx_id = 0x01;
    joint_motor_config.can_init_config.rx_id = 0x11;
    joint_motor_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    // joint_l = DMMotorInit(&joint_motor_config);
    
    joint_motor_config.can_init_config.tx_id = 0x02;
    joint_motor_config.can_init_config.rx_id = 0x12;
    joint_motor_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    // joint_r = DMMotorInit(&joint_motor_config);
    
    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            // .tx_id      = 0X427, // 超级电容默认接收id
            // .rx_id      = 0x300, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
    #endif


    // motor_lb->motor_controller.speed_PID.Iout=0;motor_rb->motor_controller.speed_PID.Iout=0;
    // motor_lf->motor_controller.speed_PID.Iout=0;motor_rf->motor_controller.speed_PID.Iout=0;

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD) // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
 #endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
float vxy_k=1,super_vxy_k=2;//小陀螺wz和vx，vy的比例
float friction_compensation_lf = 600.0f;  
float friction_compensation_rf = -750.0f;  
float friction_compensation_lb = 110.0f;  
float friction_compensation_rb = -100.0f;  

// float friction_compensation_lf = 0.0f;  
// float friction_compensation_rf = 0.0f;  
// float friction_compensation_lb = 0.0f;  
// float friction_compensation_rb = 0.0f;  

// 死区与滞回参数
#define FRICTION_KICK_THRESHOLD 100.0f    // 低速启动阈值（度/秒）
#define FRICTION_HYSTERESIS 30.0f          // 滞回带宽度（度/秒）
#define FRICTION_SMOOTHING 0.2f            // 补偿平滑系数（0~1）

/**
 * @brief 标准摩擦补偿：定值随指令方向变号，带智能死区
 * @param velocity_cmd 速度指令（度/秒）
 * @param compensation 补偿定值
 * @param last_sign 上次方向状态指针（用于滞回）
 * @return 补偿力（与指令方向相反）
 */
static float calculateFrictionCompensation(float velocity_cmd, float compensation, int8_t *last_sign)
{
    //无指令时：补偿归零，确保绝对静止
    if (fabsf(velocity_cmd) < 1.0f) {
        *last_sign = 0;
        return 0.0f;
    }
    
    int8_t current_sign;
    
    // 滞回判断：防止方向符号抖动
    if (fabsf(velocity_cmd) > FRICTION_KICK_THRESHOLD + FRICTION_HYSTERESIS) {
        current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
        *last_sign = current_sign;
    } else if (fabsf(velocity_cmd) < FRICTION_KICK_THRESHOLD - FRICTION_HYSTERESIS) {
        // 低速启动区：给kick启动力
        if (*last_sign == 0) {
            current_sign = (velocity_cmd > 0.0f) ? 1 : -1;
            *last_sign = current_sign;
        } else {
            current_sign = *last_sign;  // 保持上次方向
        }
    } else {
        // 滞回带内：保持上次状态，防止翻符号
        current_sign = (*last_sign != 0) ? *last_sign : ((velocity_cmd > 0.0f) ? 1 : -1);
        *last_sign = current_sign;
    }
    return -compensation * (float)current_sign;
}

/**
 * @brief 更新四个轮子的摩擦补偿
 * @param velocity_cmd_* 各轮速度指令（度/秒）
 */
static void updateAllWheelsFriction(float velocity_cmd_lf, float velocity_cmd_rf, 
                                     float velocity_cmd_lb, float velocity_cmd_rb)
{
    
    float target_lf = calculateFrictionCompensation(velocity_cmd_lf, friction_compensation_lf, &sign_lf);
    float target_rf = calculateFrictionCompensation(velocity_cmd_rf, friction_compensation_rf, &sign_rf);
    float target_lb = calculateFrictionCompensation(velocity_cmd_lb, friction_compensation_lb, &sign_lb);
    float target_rb = calculateFrictionCompensation(velocity_cmd_rb, friction_compensation_rb, &sign_rb);
    // 平滑过渡到目标值
    friction_lf_state += FRICTION_SMOOTHING * (target_lf - friction_lf_state);
    friction_rf_state += FRICTION_SMOOTHING * (target_rf - friction_rf_state);
    friction_lb_state += FRICTION_SMOOTHING * (target_lb - friction_lb_state);
    friction_rb_state += FRICTION_SMOOTHING * (target_rb - friction_rb_state);
    // 调试输出
    
}
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = -chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;

    updateAllWheelsFriction(vt_lf, vt_rf, vt_lb, vt_rb);
}

static ramp_t super_ramp;
static float Power_Output;
float superCap_buff=50;
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 * @param
 * @param
 *
 */
 static void LimitChassisOutput()
 {
     static float Plimit;

    // 缓冲能量闭环
    if (chassis_cmd_recv.power_buffer < 50 && chassis_cmd_recv.power_buffer >= 40)
        Plimit = 0.9 + (chassis_cmd_recv.power_buffer - 40) * 0.01;
    else if (chassis_cmd_recv.power_buffer < 40 && chassis_cmd_recv.power_buffer >= 35)
        Plimit = 0.75 + (chassis_cmd_recv.power_buffer - 35) * (0.15f / 5);
    else if (chassis_cmd_recv.power_buffer < 35 && chassis_cmd_recv.power_buffer >= 30)
        Plimit = 0.6 + (chassis_cmd_recv.power_buffer - 30) * (0.15 / 5);
    else if (chassis_cmd_recv.power_buffer < 30 && chassis_cmd_recv.power_buffer >= 20)
        Plimit = 0.35 + (chassis_cmd_recv.power_buffer - 20) * (0.25f / 10);
    else if (chassis_cmd_recv.power_buffer < 20 && chassis_cmd_recv.power_buffer >= 10)
        Plimit = 0.15 + (chassis_cmd_recv.power_buffer - 10) * 0.01;
    else if (chassis_cmd_recv.power_buffer < 10 && chassis_cmd_recv.power_buffer > 0)
        Plimit = 0.05 + chassis_cmd_recv.power_buffer * 0.01;
    else if (chassis_cmd_recv.power_buffer == 60)
        Plimit = 1;
    // Plimit = 0;
    // if(chassis_cmd_recv.SuperCap_flag_from_user==SUPERCAP_PMOS_OPEN){
    //     Power_Output=chassis_cmd_recv.power_limit - 10 + 20 * Plimit+superCap_buff;
    // }
    // else {
    Power_Output = chassis_cmd_recv.power_limit - 10 + 20 * Plimit;
    // }
    //超电
    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

    ramp_init(&super_ramp, 300);
 }
float cap_power_output;
float super_power_max=0;
// 提高功率上限，飞坡或跑路
static void SuperLimitOutput()
{
    static float power_output;
    if(pm01_od.v_out>21)cap_power_output=chassis_cmd_recv.power_limit+120;
    else cap_power_output=chassis_cmd_recv.power_limit +10 + 100 * (0.01f*(float)pm01_od.v_out- 16.0f) / 5.0f;
    Power_Output = (power_output + (cap_power_output - power_output) * ramp_calc(&super_ramp));
    // Power_Output = (power_output + (250 - 20 + 40 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f - power_output) * ramp_calc(&super_ramp));
    PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

    power_output = Power_Output;
}

/**
 * @brief 超电控制算法
 *
 *
 */
// uint8_t Super_Voltage_Allow_Flag;
// static SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
 void Super_Cap_control()
 {
//     // 状态机逻辑,滞回
//     switch (SuperCap_state) {
//         case SUPER_STATE_LOW:
//             if (cap->cap_msg_s.CapVot > SUPER_VOLTAGE_THRESHOLD_HIGH) {
//                 SuperCap_state = SUPER_STATE_HIGH;
//             }
//             break;
//         case SUPER_STATE_HIGH:
//             if (cap->cap_msg_s.CapVot < SUPER_VOLTAGE_THRESHOLD_LOW) {
//                 SuperCap_state = SUPER_STATE_LOW;
//             }
//             break;
//         default:
//             SuperCap_state = SUPER_STATE_LOW;
//             break;
//     }

//     // 小于12V关闭
//     if (SuperCap_state == SUPER_STATE_LOW) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;
//     } else if (SuperCap_state == SUPER_STATE_HIGH) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;
//     } else {
//         // none
//     }

//     // User允许开启电容 且 电压充足
//     if (chassis_cmd_recv.SuperCap_flag_from_user == SUPER_USER_OPEN) {
//         cap->cap_msg_g.enabled = SUPER_CMD_OPEN;
if(chassis_cmd_recv.SuperCap_flag_from_user==SUPERCAP_PMOS_OPEN){
        SuperLimitOutput();
}
//     } else {
//         cap->cap_msg_g.enabled = SUPER_CMD_CLOSE;
else {
         LimitChassisOutput();
}
//     }

     // 设定速度参考值
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
 }

// 获取功率裆位
 static void Power_get()
 {
     //cap->cap_msg_g.power_limit = chassis_cmd_recv.power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
 }
extern Power_Data_s power_data; // 电机功率数据;
extern float power_value;
uint64_t can_send_count=0;
float offset_angle_watch;
float text_vx=0;
static float freequence = 0,freequence_last=0; 
extern float half_to_float(uint16_t half);
extern Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
 float follow_kp,follow_kd;
float super_speed=10000;
float textkd=6,text_speed=10000;
static float getRotato_K(uint8_t level){
    switch(level){
        case 1:
            return textkd;
            break;
        case 2:
            return 75;
            break;
            case 3:
            return 80;
            break;
            case 4:
            return 85;
            break;
            case 5:
            return 90;
            break;
            case 6:
            return 95;
            break;
            case 7:
            return 100;
            break;
            case 8:
            return 105;
            break;
            case 9:
            return 110;
            break;
            case 10:
            return 120;
            break;
        default:
            return 100;
}
}
extern uint16_t powerLim;

static float getWzspeed(uint16_t powerLimit){
    switch (powerLimit){
        case 70://------------------
            return 3500;
        break;
        case 75:
        return 3750;
        break;
        case 80://------------------
                return 4000;
        break;
        case 85:
        return 4175;
        break;
        case 90:
        return 4350;
        break;
        case 95:
        return 4525;
        break;
        case 100://------------------
        return 4700;
        break;
        case 105:
        return 4900;
        break;
        case 110:
        return 5100;
        break;
        case 120://------------------
        return 5300;
        break;
        default:
        return 3000;
        break;
    }
}

void joint_limit(float l_target, float r_target)
{
    if(l_target > JOINT_LEFT_UP_LIMIT)
        l_target = JOINT_LEFT_UP_LIMIT;
    if(l_target < JOINT_LEFT_DOWN_LIMIT)
        l_target = JOINT_LEFT_DOWN_LIMIT;
    joint_l->ctrl.pos_set = l_target;

    if(r_target > JOINT_RIGHT_UP_LIMIT)
        r_target = JOINT_RIGHT_UP_LIMIT;
    if(r_target < JOINT_RIGHT_DOWN_LIMIT)
        r_target = JOINT_RIGHT_DOWN_LIMIT;
    joint_r->ctrl.pos_set = r_target;
}



float offset_angle_watch;
float angle_l,angle_r;
volatile static float angle_avg;//, tor_avg;
// float angle_measure_all[50];//,tor_measure_all[10];
// float angle_measure_sum;//, tor_measure_sum;
float angle_target, angle_l_target, angle_r_target;
int16_t avg_count = 0, avg_i;
// float l_offset = -0.311236, r_offset = 0.0434394;
float l_offset = 1.7437732, r_offset = -0.0351452;
float length_l_measure,length_r_measure,length_measure;
float length_target;
float angle_test = 0.07f;//0.17;
// float length_test = 0.2;
float leg_p = 0.3;

float safe_sqrt(float x)
{
    if(x < 0.0f)
    {
        return 0.0f;
    }
    return __builtin_sqrtf(x);
}


/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息

    if(superCap_watchdog==0) {
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING);
        HAL_CAN_Start(&hcan1);
}
    else superCap_watchdog--;
#ifdef CHASSIS_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    // chassis_cmd_recv_half_float = *(Chassis_Ctrl_Cmd_s_half_float *)CANCommGet(chasiss_can_comm);
    // chassis_cmd_recv.power_limit=1000;
    // chassis_cmd_recv.chassis_power=1000;
    // chassis_cmd_recv.power_buffer=1000;
    // chassis_cmd_recv.vx=chassis_rs485_recv.vx;//half_to_float(chassis_cmd_recv_half_float.vx);
    // chassis_cmd_recv.vy=chassis_rs485_recv.vy;//half_to_float(chassis_cmd_recv_half_float.vy);
    // chassis_cmd_recv.wz=chassis_rs485_recv.wz;//half_to_float(chassis_cmd_recv_half_float.wz);
    // chassis_cmd_recv.gimbal_error_angle=chassis_rs485_recv.gimbal_error_angle;
    // chassis_cmd_recv.offset_angle=chassis_rs485_recv.offset_angle;
    //chassis_cmd_recv.offset_angle=half_to_float(chassis_cmd_recv_half_float.offset_angle);
    //chassis_cmd_recv.SuperCap_flag_from_user=chassis_cmd_recv_half_float.SuperCap_flag_from_user;
    // chassis_cmd_recv.chassis_mode=chassis_rs485_recv.chassis_mode;//chassis_cmd_recv_half_float.chassis_mode;

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE){ // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DMMotorStop(joint_l);
        DMMotorStop(joint_r);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DMMotorEnable1(joint_l);
        DMMotorEnable1(joint_r);
    }

    static float offset_angle;
    static float sin_theta, cos_theta;
    static float current_speed_vw, vw_set;
    static ramp_t rotate_ramp;
    static dipAngle = 0;

    // offset_angle       = chassis_cmd_recv.offset_angle + chassis_cmd_recv.gimbal_error_angle;
    // offset_angle_watch = offset_angle;

    if(chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB || chassis_cmd_recv.chassis_mode == CHASSIS_CLIMB_RETRACT)
        // offset_angle =(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
        offset_angle = 0;
    else
        // offset_angle =chassis_cmd_recv.offset_angle;
        offset_angle = 0;
    chassis_cmd_recv.offset_angle = 0;

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:     //一般不进入
            // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_recv.wz = 0;
            powerLim=0;
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode  = LEG_ACTIVE_SUSPENSION;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台
            powerLim=0;            
            // chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            chassis_cmd_recv.wz = 0;
            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode  = LEG_ACTIVE_SUSPENSION;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        
        // vw_set = chassis_cmd_recv.vw_set;
        // vxy_k=chassis_cmd_recv.wz_K;
            // if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_PMOS_OPEN) {
            if(chassis_cmd_recv.SuperCap_flag_from_user==SUPERCAP_PMOS_OPEN){
                powerLim=200;
                if(chassis_cmd_recv.wz_K)vxy_k=chassis_cmd_recv.wz_K;
                else vxy_k=5;
                if(chassis_cmd_recv.wz)vw_set=chassis_cmd_recv.wz;
                else vw_set=25000;
                // vxy_k=chassis_cmd_recv.wz_K;
            } else {
                powerLim=chassis_cmd_recv.power_limit;
                // chassis_cmd_recv.power_limit=110;
                vw_set=getWzspeed(chassis_cmd_recv.power_limit);
                vxy_k =1;
                // vxy_k=textkd;
                // 26.6f*chassis_cmd_recv.power_limit+1537;
            }
            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;

            chassis_cmd_recv.wz = chassis_vw;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            leg_mode            = LEG_ACTIVE_SUSPENSION;
            break;
            
        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz = -5000;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD); // 矫正小陀螺偏心
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle /*+ 22*/) * DEGREE_2_RAD);
            leg_mode            = LEG_ACTIVE_SUSPENSION;
            break;
        case CHASSIS_CLIMB:
        case CHASSIS_CLIMB_WITH_PULL:
        case CHASSIS_CLIMB_WITH_PUSH:
            // chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            chassis_cmd_recv.wz = 0;
            cos_theta           = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta           = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode            = LEG_CLIMB;
            ramp_init(&rotate_ramp, 250);
            switch(chassis_cmd_recv.chassis_mode)
            {
                case CHASSIS_CLIMB:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
                case CHASSIS_CLIMB_WITH_PULL:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
                    break;
                case CHASSIS_CLIMB_WITH_PUSH:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
                default:
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
                    break;
            }
            break;
        case CHASSIS_CLIMB_RETRACT:
            // chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, offset_angle, 0);
            chassis_cmd_recv.wz = 0;
            cos_theta           = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta           = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            leg_mode            = LEG_CLIMB_RETRACT;
            ramp_init(&rotate_ramp, 250);
            break;
        default:
            break;
    }

    switch(leg_mode)
    {
        case LEG_ACTIVE_SUSPENSION:
            dipAngle = 0;
            // joint_l->ctrl.kp_set = 150;
            // joint_l->ctrl.kd_set = 2;
            // joint_l->ctrl.tor_set = 7;
            // joint_r->ctrl.kp_set = 150;
            // joint_r->ctrl.kd_set = 2;
            // joint_r->ctrl.tor_set = -7;
            Chassis_Follow_PID.Kp = 105;
            break;
        case LEG_CLIMB:
            dipAngle = 0.1;
            // joint_l->ctrl.kp_set = 150;
            // joint_l->ctrl.kd_set = 2;
            // joint_l->ctrl.tor_set = 7;
            // joint_r->ctrl.kp_set = 150;
            // joint_r->ctrl.kd_set = 2;
            // joint_r->ctrl.tor_set = -7;
            Chassis_Follow_PID.Kp = 105;
            break;
        case LEG_CLIMB_RETRACT:
            dipAngle = 0;
            // joint_l->ctrl.kp_set = 50;
            // joint_l->ctrl.kd_set = 4;
            // joint_l->ctrl.tor_set = -6;
            // joint_r->ctrl.kp_set = 50;
            // joint_r->ctrl.kd_set = 4;
            // joint_r->ctrl.tor_set = 6;
            Chassis_Follow_PID.Kp = 50;
            break;
        default:
            break;
    }


    angle_l = joint_l->measure.pos - l_offset;
    angle_r = -joint_r->measure.pos + r_offset;
    angle_avg = (angle_l + angle_r) / 2;
    // tor_avg = (joint_l->measure.tor - joint_r->measure.tor) / 2;
    // angle_measure_all[avg_count] = (angle_l + angle_r) / 2;
    // tor_measure_all[avg_count] = (joint_l->measure.tor - joint_r->measure.tor) / 2;
    // avg_count++;
    // if(avg_count >= 10)
    // {
    //     angle_measure_sum = 0;
    //     // tor_measure_sum = 0;
    //     for(avg_i = 0; avg_i < avg_count; avg_i++)
    //     {
    //         angle_measure_sum += angle_measure_all[avg_i];
    //         // tor_measure_sum += tor_measure_all[avg_i];
    //     }
    //     angle_avg = angle_measure_sum / avg_count;
    //     // tor_avg = tor_measure_sum / avg_count;
    //     avg_count = 0;
    // }
    joint_tor_feedforward = - 25.9625 * angle_avg * angle_avg * angle_avg
                            + 62.3065 * angle_avg * angle_avg
                            - 51.3808 * angle_avg
                            + 18.8648;
    length_l_measure = -0.05814 * angle_l * angle_l + 0.2072 *angle_l + 0.107;
    length_r_measure = -0.05814 * angle_r * angle_r + 0.2072 *angle_r + 0.107;
    length_measure = (length_l_measure + length_r_measure)/2;
    length_target = length_measure - leg_p * (Chassis_IMU_data->output.INS_angle[1] - dipAngle);
    angle_target = (-0.2072 + safe_sqrt(0.2072 * 0.2072 + 4 * 0.05814 * (0.107 - length_target)))/(-2 * 0.05814);
    angle_l_target = angle_target + l_offset;
    angle_r_target = r_offset - angle_target;
    // if(leg_mode == LEG_CLIMB_RETRACT)
    //     joint_limit(l_offset + angle_test, r_offset - angle_test);
    // else
    //     joint_limit(angle_l_target, angle_r_target);
    joint_l->motor_controller.pid_ref = dipAngle;
    joint_r->motor_controller.pid_ref = dipAngle;
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    chassis_vx*=vxy_k;
    chassis_vy*=vxy_k;
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    Super_Cap_control();
#endif                                                         // CHASSIS_BOARD
    // 获得给电容传输的电容吸取功率等级
    // Power_get();

    // 给电容传输数据
    // SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 推送反馈消息
    memcpy(&chassis_feedback_data.CapFlag_open_from_real, &cap->cap_msg_s.SuperCap_open_flag_from_real, sizeof(uint8_t));
    memcpy(&chassis_feedback_data.cap_voltage, &cap->cap_msg_s.CapVot, sizeof(float));
    memcpy(&chassis_feedback_data.chassis_power_output, &Power_Output, sizeof(float));
    memcpy(&chassis_feedback_data.chassis_voltage, &cap->cap_msg_s.chassis_voltage_from_cap, sizeof(float));
   
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    // memcpy(&chassis_feedback_data.power_cal,&power_data.total_power,sizeof(float));
    // memcpy(&chassis_feedback_data.power_real,&power_value,sizeof(float));
    // memcpy(&chassis_feedback_data.angle,Chassis_IMU_data->output.INS_angle,3*sizeof(float));
    //CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD

}
