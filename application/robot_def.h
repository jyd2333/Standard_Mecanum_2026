/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#define NUC_RECV_SIZE 32
#define Chassis_Ctrl_Cmd_s_uart_size sizeof(Chassis_Ctrl_Cmd_s_uart)
#define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)
/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
//#define ONE_BOARD // 单板控制整车
#define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板
#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据
// #define BIG_HEAD
/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数



#define PITCH_POS_MAX_ECD           4650
#define PITCH_POS_MIN_ECD           5700
#define YAW_CHASSIS_ALIGN_ECD       817 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096   0    // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD           5225 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_POS_UP_LIMIT_ECD      6191 // 云台竖直方向高处限位编码器值,若对云台有机械改动需要修改
#define PITCH_POS_DOWN_LIMIT_ECD    4830 // 云台竖直方向低处限位编码器值,若对云台有机械改动需要修改
#define JOINT_LEFT_UP_LIMIT         0.73f
#define JOINT_LEFT_DOWN_LIMIT       -0.036f
#define JOINT_RIGHT_UP_LIMIT        -2.37f
#define JOINT_RIGHT_DOWN_LIMIT      -3.14f
#define PITCH_DOWN_LIMIT            -0.45f
#define PITCH_UP_LIMIT              0.45f


#define PITCH_FEED_TYPE     1 // 云台PITCH轴反馈值来源:编码器为0,陀螺仪为1
#define PITCH_INS_FEED_TYPE 1//1 // 云台PITCH轴陀螺仪反馈:角度值为0,弧度制为1
#define PITCH_ECD_UP_ADD    1 // 云台抬升时编码器变化趋势,增为1,减为0 (陀螺仪变化方向应相同)

// 发射参数
#define ONE_BULLET_DELTA_ANGLE 1330   // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 19.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         6     // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE             414.5   // 320.5   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            361   // 320.5   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           77   // 轮子半径
#define REDUCTION_RATIO_WHEEL  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define CHASSIS_SPEED          40000 // 键盘控制不限功率时底盘最大移动速度
#define YAW_K                  0.00025f
#define PITCH_K                0.000004f

// 模拟小电脑负重 652.2
// 其他参数(尽量所有参数集中到此文件)
#define BUZZER_SILENCE 0 // 蜂鸣器静音,1为静音,0为正常

#define IMU_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改传感器参数
#ifndef IMU_DEF_PARAM_WARNING
#define IMU_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !IMU_DEF_PARAM_WARNING

// 陀螺仪校准数据，开启陀螺仪校准后可从INS中获取
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.00142266625f//-0.000909539289f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.000708730659f//0.00354450056f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET 0.000225723968f
// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f
// 设置陀螺仪数据相较于云台的yaw,pitch,roll的方向
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, -1.0f, 0.0f},                 \
    {0.0f, 0.0f, -1.0f},              \
    {1.0f, 0.0f, 0.0f}

#define INS_YAW_ADDRESS_OFFSET   2 // 陀螺仪数据相较于云台的yaw的方向
#define INS_PITCH_ADDRESS_OFFSET 1 // 陀螺仪数据相较于云台的pitch的方向
#define INS_ROLL_ADDRESS_OFFSET  0 // 陀螺仪数据相较于云台的roll的方向
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_REVERSE_ROTATE,    // 反方向小陀螺
    CHASSIS_CLIMB,             // 上台阶
    CHASSIS_CLIMB_RETRACT,     // 收腿
} chassis_mode_e;

typedef enum {
    LEG_ACTIVE_SUSPENSION = 0,  // 主动悬挂
    LEG_CLIMB,                  // 上台阶
    LEG_CLIMB_RETRACT,          // 收腿
} leg_mode_e;
// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_MOTOR_MODE,     // 编码器反馈模式，英雄部署模式可用
} gimbal_mode_e;
typedef enum{
    none_version_control,
    version_control,
}nuc_mode_e;
// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef struct{
	uint32_t send_ID;
	uint32_t receive_ID;
	uint8_t error_state;
	uint16_t p_int;
	uint16_t v_int;
	uint16_t t_int;
	float PMAX;
	float VMAX;
	float TMAX;
	float pos;
	float vel;
	float tor;
	uint8_t MOS_temperature;
	uint8_t temperature;
}DM4310_t;
extern DM4310_t DM4310;
/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot/UI订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    uint16_t power_buffer;           // 60焦耳缓冲能量
    float chassis_power;             // 底盘瞬时功率
    uint8_t level;                   // 机器人等级
    uint16_t power_limit;            // 底盘功率限制
    uint8_t SuperCap_flag_from_user; // 超电的标志位
    float vx;                        // 前进方向速度
    float vy;                        // 横移方向速度
    float wz;                        // 旋转速度
    float offset_angle;              // 底盘和归中位置的夹角
    float gimbal_error_angle;        // 云台当前位置与目标（归中）位置的夹角
    chassis_mode_e chassis_mode;
    float vw_set;
    float wz_K;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;
typedef struct
{
    // 控制部分
    uint16_t power_buffer;           // 60焦耳缓冲能量
    float chassis_power;             // 底盘瞬时功率
    uint8_t level;                   // 机器人等级
    uint16_t power_limit;            // 底盘功率限制
    uint8_t SuperCap_flag_from_user; // 超电的标志位
    uint16_t vx;                        // 前进方向速度
    uint16_t vy;                        // 横移方向速度
    uint16_t wz;                        // 旋转速度
    uint16_t offset_angle;              // 底盘和归中位置的夹角
    uint16_t gimbal_error_angle;        // 云台当前位置与目标（归中）位置的夹角
    chassis_mode_e chassis_mode;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s_half_float;
// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float yaw_version;
    float pitch_version;
    float yaw_speedFeed;
    float yaw_kp;
    float yaw_kd;
    float yaw_speedKp;
    gimbal_mode_e gimbal_mode;
    nuc_mode_e nuc_mode;
    uint8_t ifSpeedFeed;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    loader_mode_e load_mode_last;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    uint16_t shooter_heat_cooling_rate; // 枪口热量冷却
    uint16_t shooter_referee_heat;      // 17mm枪口热量
    uint16_t shooter_cooling_limit;     // 枪口热量上限
    float shoot_rate;                   // 连续发射的射频,unit per s,发/秒
    float bullet_speed;                 // 子弹速度
    float shoot_aim_angle;
    uint8_t Shoot_Once_Flag;
    int32_t shoot_count;
} Shoot_Ctrl_Cmd_s;

// cmd发布的UI数据,由UI订阅
typedef struct
{
    uint8_t ui_send_flag; // UI发送标志位
    chassis_mode_e chassis_mode;
    uint16_t chassis_attitude_angle; // 底盘姿态角
    friction_mode_e friction_mode;
    uint8_t rune_mode;
    uint8_t SuperCap_mode;           // 开关指示 未开启为1
    float SuperCap_voltage;          // 超电电压
    float Chassis_Ctrl_power;        // 底盘控制功率
    uint16_t Cap_absorb_power_limit; // 超电吸收功率
    float Chassis_voltage;           // 底盘电压
    uint16_t Chassis_power_limit;    // 底盘功率
    uint16_t Shooter_heat;              // 枪口热量
    uint8_t robot_level;
    uint16_t Heat_Limit;             // 热量上限
    uint16_t nuc_flag;
} UI_Cmd_s;

/* ----------------gimbal/shoot/chassis/UI发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    float real_vx;
    float real_vy;
    float real_wz;
    uint8_t CapFlag_open_from_real;
    float cap_voltage;
    uint16_t capget_power_limit;
    float chassis_power_output;
    float chassis_voltage;
    float chassis_imu_data[3];
    float chassis_pitch;
} Chassis_Upload_Data_s;
typedef struct 
{
  uint16_t  yaw
}Chassis_Upload_Data_s_half_float;

typedef struct
{
    INS_Instance *gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_ecd;
    uint16_t pitch_ecd;
    float yaw_total_angle;
    float pitch_total_angle;
    float yaw_angle_pidout;
    float yaw_angleP;
    float yaw_angleD;
    float yaw_speedP;
    float speedFeed;
} Gimbal_Upload_Data_s;

typedef struct
{
    int shooter_heat_control; // 热量控制
    float shooter_local_heat; // 本地热量
    float loader_angle;
} Shoot_Upload_Data_s;
typedef enum 
{
    red,
    blue,
    /* data */
}version_color;

typedef struct
{
    float vx;
    float vy;
    float wz;
    float yaw_control;
    float yaw_angle;
    float yaw_gyro;
    float offset_angle;
    float gimbal_error_angle;
    // float wz_set;
    int32_t shoot_count;
    float nuc_yaw;
    // float wz_K;
    // float yaw_speedFeed;
    // float yaw_kp;
    // float yaw_kd;
    // float yaw_speedKp;
    gimbal_mode_e gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    nuc_mode_e nuc_mode;
    uint8_t reset_flag;
    // version_color color;
    uint8_t UI_SendFlag;
    uint8_t superCap_flag;
    
}Chassis_Ctrl_Cmd_s_uart;
typedef struct
{
    
    float yaw_motor_single_round_angle;
    float yaw_total_angle;
    float yaw_ecd;
    float chassis_pitch_angle;
    float chassis_yaw_gyro;
    float initial_speed;
    float yaw_angle_pidout;
    version_color color;
}Chassis_Upload_Data_s_uart;
typedef struct
{
    // code to go here
    uint8_t eee;
} UI_Upload_Data_s;

/* CAN通信句柄结构体 */
typedef struct {
    // CAN硬件参数
    CAN_HandleTypeDef *can_handle;  // CAN句柄
    uint32_t tx_id;                 // 发送ID  
    uint32_t rx_id;                 // 接收ID
    uint32_t tx_mailbox;            // 发送邮箱号
    
    // 数据缓冲区 
    uint8_t rx_data[8];            // 接收数据缓冲区
    uint8_t tx_data[8];            // 发送数据缓冲区
} CAN_Handler;
extern uint8_t nuc_rx[8];
#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H