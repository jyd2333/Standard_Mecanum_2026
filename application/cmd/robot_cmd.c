// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "omni_UI.h"
// module
#include "wit_c_sdk.h"
#include "IIC_CONTROL.h"
#include "hwt606.h"
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"
#include "fifo.h"
#include "tool.h"
#include "super_cap.h"
#include "AHRS_MiddleWare.h"
#include "rm_referee.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "vofa.h"
#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else           // PITCH电机反馈数据源为编码器
                                                
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
int signl=0;
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef CHASSIS_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD
static float nuc_pitch,nuc_yaw;
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Ctrl_Cmd_s_half_float chassis_cmd_send_half_float;
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Chassis_Ctrl_Cmd_s_uart chassis_cmd_send_uart;
static Chassis_Upload_Data_s_uart chassis_fetch_data_uart;
extern referee_info_t *referee_data_for_ui;
static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
gimbal_mode_e gimbal_mode_last;
HostInstance *host_instance; // 上位机接口
HostInstance *IMU_instance; // 接口
HostInstance *rs485_master_instance; // 接口
HostInstance *rs485_slaver_instance; // 接口
// 这里的四元数以wxyz的顺序
static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位
uint8_t vision_send_data[32]; // 给视觉上位机发送的数据
uint8_t chasssis_ctrl_data[sizeof(Chassis_Ctrl_Cmd_s_uart)+3];
uint8_t chasssis_update_data[sizeof(Chassis_Upload_Data_s_uart)+3];
static Publisher_t *gimbal_cmd_pub  ;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
int Shoot_Aim_Angle = 0;
uint16_t Switch_Left_Last;
static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
int Trig_Time = 0;	//发射触发时间
static referee_info_t *referee_data; // 用于获取裁判系统的数据
static hwt606_info_t *HWT606_data;
uint8_t UI_SendFlag = 1; // UI发送标志位
extern uint16_t g_power_set ; // 功率设置
uint8_t auto_rune; // 自瞄打符标志位
uint8_t lob_mode=0;//lob_shoot
uint8_t telescope_pos = 0;//0:normal 1:zoom
uint8_t fpv_pos = 0;//0:normal 1:lob
float rec_yaw, rec_pitch;
float fire_advice;
// #define Chassis_Ctrl_Cmd_s_uart_size sizeof(Chassis_Ctrl_Cmd_s_uart)
// #define Chassis_Upload_Data_s_uart_size sizeof(Chassis_Upload_Data_s_uart)
uint8_t SuperCap_flag_from_user = 0; // 超电标志位
uint8_t rc_update_flag = 0;//遥控器数据更新标志位（防止同一个周期多次触发）
// float imu_angle[3];
// float imu_gyro[3];
static PIDInstance PITCH_version_PID = {
    .Kp            = 0.005,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.008, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0.3,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 30,
    .Improve       = PID_OutputFilter|PID_Derivative_On_Measurement|PID_Integral_Limit|PID_Trapezoid_Intergral/*PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement*/,
    .MaxOut        = 0.04,
    .Output_LPF_RC=0.95,
};
static PIDInstance YAW_version_PID = {
    .Kp            = 0.0037,//0.02,//0.033,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.00055,//0.001,//0.003, // 0.0,  // 0.07,  // 0
    .DeadBand      = 0,  // 0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement|PID_OutputFilter,
    .MaxOut        = 0.1,//0.05,//0.045,
    .Output_LPF_RC=0.7,
};
float alphaA=0.1,alphaB=0.7;
float nuc_yaw_filiter(float input){
    static float input_last;
    float output=0;
    if(input<0)output=alphaB*input+(1-alphaB)*input_last;
    else output=alphaB*input+(1-alphaB)*input_last;
    input_last=output;
    return output;
}
float nuc_pitch_filiter(float input){
    static float input_last;
    float output=alphaA*input+(1-alphaA)*input_last;
    input_last=output;
    return output;
}
void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    vision_recv_data[8] = 1;
}
static void FIFO_WRITE(uint16_t recv_len){
    DaemonReload(host_instance->daemon);         // 先喂狗
    WritePacketToFIFO(&NUC_fifo,host_instance->comm_instance,NUC_RECV_SIZE);
}
uint16_t float_to_half(float f) {
    uint32_t bit_pattern;
    memcpy(&bit_pattern, &f, sizeof(f));  // 获取float的二进制表示

    uint32_t sign = (bit_pattern >> 31) & 0x1;
    uint32_t exponent = (bit_pattern >> 23) & 0xFF;
    uint32_t mantissa = bit_pattern & 0x7FFFFF;

    // 处理非规范化数
    if (exponent == 0) {
        return (sign << 15); // 非规范化数
    }

    // 处理无穷大和NaN
    if (exponent == 0xFF) {
        return (sign << 15) | (0x1F << 10) | (mantissa >> 13);
    }

    // 规范化数的转换
    int half_exponent = exponent - 127 + 15;
    if (half_exponent > 0x1F) half_exponent = 0x1F;  // 最大指数
    if (half_exponent < 0) half_exponent = 0;  // 最小指数

    uint16_t half = (sign << 15) | (half_exponent << 10) | (mantissa >> 13);
    return half;
}
// 半精度浮点数转单精度浮点数
float half_to_float(uint16_t half) {
    uint32_t sign = (half >> 15) & 0x1;           // 符号位
    uint32_t exponent = (half >> 10) & 0x1F;      // 指数位
    uint32_t mantissa = half & 0x3FF;             // 尾数位

    // 处理非规范化数
    if (exponent == 0) {
        // 非规范化数处理（需要补零）
        return sign ? -0.0f : 0.0f;
    }

    // 处理无穷大和NaN
    if (exponent == 0x1F) {
        if (mantissa == 0) {
            return sign ? -INFINITY : INFINITY;
        } else {
            return NAN;
        }
    }

    // 规范化数的转换
    int32_t float_exp = (int32_t)(exponent) - 15 + 127; // 调整指数偏移量
    uint32_t float_mantissa = mantissa << 13;           // 扩展尾数至23位

    // 生成32位浮点数（符号 + 指数 + 尾数）
    uint32_t float_bits = (sign << 31) | (float_exp << 23) | float_mantissa;

    float result;
    memcpy(&result, &float_bits, sizeof(result));  // 将位模式转换为浮点数
    return result;
}
// 将 float 拆分为 4 个 uint8_t 字节（手动处理高低位）
void float_to_uint8_manual(float value, uint8_t *bytes) {
    uint32_t as_int;
    memcpy(&as_int, &value, sizeof(float)); // 将 float 转换为 uint32_t

    *bytes = (uint8_t)(as_int & 0xFF);         // 第 1 个字节（低位）
    *(bytes+1) = (uint8_t)((as_int >> 8) & 0xFF);  // 第 2 个字节
    *(bytes+2) = (uint8_t)((as_int >> 16) & 0xFF); // 第 3 个字节
    *(bytes+3) = (uint8_t)((as_int >> 24) & 0xFF); // 第 4 个字节（高位）
}

// 将 4 个 uint8_t 字节解析回 float（手动处理高低位）
float uint8_to_float_manual(uint8_t *bytes) {
    uint32_t as_int = 0;

    as_int |= ((uint32_t)bytes[0]);
    as_int |= ((uint32_t)bytes[1] << 8);
    as_int |= ((uint32_t)bytes[2] << 16);
    as_int |= ((uint32_t)bytes[3] << 24);

    float value;
    memcpy(&value, &as_int, sizeof(float)); // 将 uint32_t 转换回 float
    return value;
}

// 将 int32_t 转换为 uint8_t 数组（长度 4）
void int32_to_uint8_array(int32_t value, uint8_t *array) {
    array[0] = (uint8_t)((value >> 24) & 0xFF); // 高位字节
    array[1] = (uint8_t)((value >> 16) & 0xFF);
    array[2] = (uint8_t)((value >> 8) & 0xFF);
    array[3] = (uint8_t)(value & 0xFF);        // 低位字节
}

// 从 uint8_t 数组还原为 int32_t
int32_t uint8_array_to_int32(const uint8_t *array) {
    return (int32_t)(
        ((int32_t)array[0] << 24) | // 高位字节
        ((int32_t)array[1] << 16) |
        ((int32_t)array[2] << 8) |
        ((int32_t)array[3])
    );
}
typedef union {
    uint8_t bytes[4];
    float value;
} FloatUnion;
float bytesToFloat(uint8_t* data) {
    FloatUnion u;
    for(int k=0;k<4;k++)u.bytes[k] = data[k];
    return u.value;
}
volatile uint32_t time_cnt;
float delta_t,delta_t_last;
void chassis_ctrl_485(Chassis_Ctrl_Cmd_s_uart data)
{
    static uint8_t master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size+3];
    master_tx_buf[0]=0XFF;
    master_tx_buf[1]=0X0D;
    float_to_uint8_manual(data.vx,master_tx_buf+2);
    float_to_uint8_manual(data.vy,master_tx_buf+6);
    float_to_uint8_manual(data.wz,master_tx_buf+10);
    float_to_uint8_manual(data.yaw_control,master_tx_buf+14);
    float_to_uint8_manual(data.yaw_angle,master_tx_buf+18);
    float_to_uint8_manual(data.yaw_gyro,master_tx_buf+22);
    float_to_uint8_manual(data.offset_angle,master_tx_buf+26);
    float_to_uint8_manual(data.gimbal_error_angle,master_tx_buf+30);
    int32_to_uint8_array(data.shoot_count,master_tx_buf+34);
    float_to_uint8_manual(data.nuc_yaw,master_tx_buf+38);
    // float_to_uint8_manual(data.wz_set,master_tx_buf+42);
    // float_to_uint8_manual(data.wz_K,master_tx_buf+46);
    // float_to_uint8_manual(data.yaw_speedFeed,master_tx_buf+50);
    // float_to_uint8_manual(data.yaw_kp,master_tx_buf+54);
    // float_to_uint8_manual(data.yaw_kd,master_tx_buf+58);
    // float_to_uint8_manual(data.yaw_speedKp,master_tx_buf+62);
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-7]=data.nuc_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-6]=data.UI_SendFlag;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-5]=data.superCap_flag;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-4]=data.reset_flag;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-3]=data.friction_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-2]=data.shoot_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size-1]=data.load_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size]=data.chassis_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size+1]=data.gimbal_mode;
    master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size+2]=0;
    for(int k=0;k<Chassis_Ctrl_Cmd_s_uart_size+2;k++)master_tx_buf[Chassis_Ctrl_Cmd_s_uart_size+2]+=master_tx_buf[k];
    HAL_UART_Transmit_DMA(&huart1,master_tx_buf,Chassis_Ctrl_Cmd_s_uart_size+3);
}
void chassis_update_485(Chassis_Upload_Data_s_uart data)
{
    static float textangle;
    static uint8_t slaver_tx_buf[Chassis_Upload_Data_s_uart_size+3];
    slaver_tx_buf[0]=0XFF;
    slaver_tx_buf[1]=0XFD;
    float_to_uint8_manual(data.yaw_motor_single_round_angle,slaver_tx_buf+2);
    float_to_uint8_manual(data.yaw_total_angle,slaver_tx_buf+6);
    float_to_uint8_manual(data.yaw_ecd,slaver_tx_buf+10);
    float_to_uint8_manual(data.chassis_pitch_angle,slaver_tx_buf+14);
    float_to_uint8_manual(data.chassis_yaw_gyro,slaver_tx_buf+18);
    float_to_uint8_manual(data.initial_speed,slaver_tx_buf+22);
    float_to_uint8_manual(data.yaw_angle_pidout,slaver_tx_buf+26);
    slaver_tx_buf[Chassis_Upload_Data_s_uart_size+1]=data.color;
    slaver_tx_buf[Chassis_Upload_Data_s_uart_size+2]=0;
    for(int k=0;k<Chassis_Upload_Data_s_uart_size+2;k++)slaver_tx_buf[Chassis_Upload_Data_s_uart_size+2]+=slaver_tx_buf[k];
    HAL_UART_Transmit_DMA(&huart1,slaver_tx_buf,Chassis_Upload_Data_s_uart_size+3);
}
void  chassis_update(){
chassis_update_485(chassis_fetch_data_uart);
}
// void getImu_485(){
//     static uint8_t imu_tx_buf[4];
//     imu_tx_buf[0]=0X50;
//     imu_tx_buf[1]=0X5A;
//     imu_tx_buf[2]=0XCC;
//     imu_tx_buf[3]=0XFF;
//     HAL_UART_Transmit_DMA(&huart1,imu_tx_buf,4);
// }
// uint8_t imu_rx_data[33];
// short angle[3],gyro[3],accel[3];
// void HWT606Decode(uint8_t *data){
    
//     if(data[0]==0x55&&data[1]==0x51){
//         if(hwt606_check_sum(data))for(int k=0;k<3;k++)accel[k]=(short)((short)data[2*k+3]<<8|data[2*k+2])*16*9.8/32768;
//     }   
//     if(data[11]==0x55&&data[12]==0x52){
//         if(hwt606_check_sum(data+11))for(int k=0;k<3;k++)gyro[k]=(short)((short)data[2*k+14]<<8|data[2*k+13])*2000/32768;
//     }
//     if(data[22]==0x55&&data[23]==0x53){
//         if(hwt606_check_sum(data+22))for(int k=0;k<3;k++)angle[k]=(short)((short)data[2*k+25]<<8|data[2*k+3+24])*180/32768;
//     }
// }
uint8_t ImuData[82];
float cnt;
// void getIMU(){
//     DaemonReload(IMU_instance->daemon);         // 先喂狗
//     cnt=DWT_GetDeltaT(&time_cnt); // 自动初始化
//     memcpy(ImuData,IMU_instance->comm_instance,82);
//     if(ImuData[0]==0x5A&&ImuData[1]==0xA5){
//         for(int k=0;k<3;k++){
//             imu_angle[k]=bytesToFloat(ImuData+4*k+54);
//             imu_gyro[k]=ANGLE_TO_RAD*bytesToFloat(ImuData+4*k+30);
//         }
//     }
//     imu_angle[1]*=-1;
// }
void rs485_master_writefifo()
{
    DaemonReload(rs485_master_instance->daemon);         // 先喂狗
    WritePacketToFIFO(&rs485_master_fifo,rs485_master_instance->comm_instance,Chassis_Upload_Data_s_uart_size+3);
}
void rs485_slaver_writefifo()
{
    DaemonReload(rs485_slaver_instance->daemon);         // 先喂狗
    chassis_update_485(chassis_fetch_data_uart);
    WritePacketToFIFO(&rs485_slaver_fifo,rs485_slaver_instance->comm_instance,Chassis_Ctrl_Cmd_s_uart_size+3);
}

uint8_t text_rx[29];
//-------------------------------------------------------------imu-----------------------------------------------------------------
/* 私有类型定义 --------------------------------------------------------------*/
float fAcc[3], fGyro[3], fAngle[3];
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			osDelay(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				//ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}
uint8_t *uart1_rx;
uint8_t usb_rxBufffer[11];
uint8_t* usbRxPointer;
void usb_rxCallback(){
    memcpy(usb_rxBufffer,usbRxPointer,11);
}
//-------------------------------------------------------------imu-----------------------------------------------------------------
void RobotCMDInit()
{
    DWT_GetDeltaT(&time_cnt); // 自动初始化
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));
    chassis_cmd_send.power_limit=100;//预设功率
    // g_power_set=&chassis_cmd_send.power_limit;
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
    
    HostInstanceConf host_conf = {
        .usart_handle=&huart1,
        .callback  = rs485_slaver_writefifo,
        .comm_mode = HOST_USART,
        .RECV_SIZE = Chassis_Ctrl_Cmd_s_uart_size+3,
    };
    
    rs485_slaver_instance = HostInit(&host_conf,2); // 双板通信串口
    uart1_rx=rs485_slaver_instance->comm_instance;
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    
#endif

    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    HostInstanceConf host_conf0 = {

        .callback  = FIFO_WRITE,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = NUC_RECV_SIZE,
    };
    host_instance = HostInit(&host_conf0,90); // 视觉通信串口

    HostInstanceConf host_conf = {
        .usart_handle=&huart1,
        .callback  = rs485_master_writefifo,
        .comm_mode = HOST_USART,
        .RECV_SIZE = Chassis_Upload_Data_s_uart_size+3,
    };
    rs485_master_instance = HostInit(&host_conf,2); // 双板通信串口

    // HostInstanceConf host_conf2 = {
    //     .usart_handle=&huart6,
    //     .callback  = getIMU,
    //     .comm_mode = HOST_USART,
    //     .RECV_SIZE = 82,
    // };
    // IMU_instance = HostInit(&host_conf2,2); // 陀螺仪通信串口
    // ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    // ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

    // CANComm_Init_Config_s comm_conf = {
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id      = 0x312,
    //         .rx_id      = 0x311,
    //     },
    //     .recv_data_len = 8,
    //     .send_data_len = 8,
    // };
    // cmd_can_comm = CANCommInit(&comm_conf);
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
   gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif
#if defined(ONE_BOARD)
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_data->referee_id.Robot_Color       = referee_data->GameRobotState.robot_id == 1 ? Robot_Red : Robot_Blue;
    referee_data->referee_id.Cilent_ID         = 0x0100 + referee_data->GameRobotState.robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = referee_data->GameRobotState.robot_id;          // 计算机器人ID
    referee_data->referee_id.Receiver_Robot_ID = 0;
}

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo  将单圈角度修改为-180~180
 *
 */
static void  CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float gimbal_yaw_current_angle;                                                // 云台yaw轴当前角度
    static float gimbal_yaw_set_angle;                                                    // 云台yaw轴目标角度
    angle                               = chassis_fetch_data_uart.yaw_motor_single_round_angle;//gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    gimbal_yaw_current_angle            = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
    gimbal_yaw_set_angle                = yaw_control;
    chassis_cmd_send.gimbal_error_angle = gimbal_yaw_set_angle - gimbal_yaw_current_angle; // 云台误差角

#if YAW_ECD_GREATER_THAN_4096 // 如果大于180度
    if (angle < 180.0f + YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE);
    else
        chassis_cmd_send.offset_angle =- (angle - YAW_ALIGN_ANGLE + 360.0f);
#else // 小于180度
    if (angle >= YAW_ALIGN_ANGLE - 180.0f && angle <= YAW_ALIGN_ANGLE + 180.0f) {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE);
    } else {
        chassis_cmd_send.offset_angle = -(angle - YAW_ALIGN_ANGLE - 360.0f);
    }
#endif
}
/**
 * @brief 对Pitch轴角度变化进行动态限位
 *
 */
float pitch_control_max,pitch_control_min;
static void PitchAngle_ActiveLimit()
{
    
    // if(gimbal_cmd_send.gimbal_mode==GIMBAL_MOTOR_MODE){
    //     pitch_control_max=205;
    //     pitch_control_min=155;
    // }
    // else {
    // pitch_control_max=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[1]+(PITCH_POS_UP_LIMIT_ECD-gimbal_fetch_data.pitch_ecd)*ECD_ANGLE_COEF_DJI;
    // pitch_control_min=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[1]-(gimbal_fetch_data.pitch_ecd-PITCH_POS_DOWN_LIMIT_ECD)*ECD_ANGLE_COEF_DJI;
    // // }
    // if(pitch_control>pitch_control_max)      pitch_control=pitch_control_max;
    // if(pitch_control<pitch_control_min)pitch_control=pitch_control_min;

}
/**
 * @brief 对Pitch轴角度变化进行限位
 *
 */
static void PitchAngleLimit()
{
    float limit_min, limit_max;
#if PITCH_INS_FEED_TYPE
    limit_min = -20.0f;//PITCH_LIMIT_ANGLE_DOWN * DEGREE_2_RAD;
    limit_max = 37.0f;//PITCH_LIMIT_ANGLE_UP * DEGREE_2_RAD;
#else
    limit_min = -24;//PITCH_LIMIT_ANGLE_DOWN;
    limit_max = 34;//PITCH_LIMIT_ANGLE_UP;
#endif

#if PITCH_ECD_UP_ADD // 云台抬升,反馈值增
    if (pitch_control > limit_max)
        pitch_control = limit_max;
    if (pitch_control < limit_min)
        pitch_control = limit_min;
#else
    if (pitch_control < limit_max)
        pitch_control = limit_max;
    if (pitch_control > limit_min)
        pitch_control = limit_min;
#endif

//     gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlProcess()
{
    // if (yaw_control - ECD_ANGLE_COEF_DJI*gimbal_fetch_data.yaw_ecd > 180) {
    //     yaw_control -= 360;
    // } else if (yaw_control - ECD_ANGLE_COEF_DJI*gimbal_fetch_data.yaw_ecd < -180) {
    //     yaw_control += 360;
    // }
    // if(chassis_cmd_send.chassis_mode==CHASSIS_NO_FOLLOW){
    //     if (yaw_control - chassis_fetch_data_uart.yaw_total_angle > 180) {
    //     yaw_control -= 360;
    // } else if (yaw_control - chassis_fetch_data_uart.yaw_total_angle < -180) {
    //     yaw_control += 360;
    // }}
    // else {
    if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] > 180) {
        yaw_control -= 360;
    } else if (yaw_control - gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] < -180) {
        yaw_control += 360;
    }
}

static float heat_coef;

static void HeatControl()
{
    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    static float rate_coef;
    if (heat_coef == 1)
        rate_coef = 1;
    else if (heat_coef >= 0.8 && heat_coef < 1)
        rate_coef = 0.8;
    else if (heat_coef >= 0.6 && heat_coef < 0.8)
        rate_coef = 0.6;
    else if (heat_coef < 0.6)
        rate_coef = 0.4;
    heat_coef = ((referee_data->GameRobotState.shooter_id1_42mm_cooling_limit - referee_data->PowerHeatData.shooter_17mm_heat0 + rate_coef * referee_data->GameRobotState.shooter_id1_42mm_cooling_rate) * 1.0f) / (1.0f * referee_data->GameRobotState.shooter_id1_42mm_cooling_limit);
    // 新热量管理
    if (referee_data->GameRobotState.shooter_id1_42mm_cooling_limit - 40 + 30 * heat_coef - shoot_fetch_data.shooter_local_heat <= shoot_fetch_data.shooter_heat_control) {
    //    shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

// 底盘模式
static uint8_t rc_mode[6];
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4
#define VISION_MODE      5

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    gimbal_mode_last=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
  //  SuperCap_flag_from_user       = SUPER_USER_CLOSE;
    memset(rc_mode, 1, sizeof(uint8_t));
    memset(rc_mode + 1, 0, sizeof(uint8_t) * 4);
    LOGERROR("[CMD] emergency stop!");
}
float pitch_nuc,yaw_nuc;
extern int load_mode_private;
static short nuc_pitch_last,nuc_yaw_last;
float kp,kd;
float pitch_shoot_angle=16.4;
float alpha_nuc_pitch=0.1,alpha_nuc_yaw=0.02,nuc_yaw_d=0.63,nuc_pitch_d=0.1,yaw_filiter=0.23,pitch_filiter,yaw_nuc_err,yaw_control_last;
float last=0,alpha=0.7;
float low_pass_filiter(float input){
    
    float output=alpha*input+(1-alpha)*last;
    last=output;
    return output;
}
//float pitch_control_max,pitch_control_min;
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 30; // 射频默认30Hz


    //滚轮下拨一下切换自瞄或关闭自瞄

    if (rc_update_flag == 1)
    {
        if (rc_data[TEMP].rc.dial > 250 && rc_data[LAST].rc.dial < 250)
        {
            if (gimbal_cmd_send.nuc_mode != version_control)
                gimbal_cmd_send.nuc_mode = version_control;
            else
                gimbal_cmd_send.nuc_mode = none_version_control;
        }

        switch (rc_data[TEMP].rc.switch_left)
        {
            case RC_SW_UP:

                if (rc_data[LAST].rc.switch_left == RC_SW_MID)//左中到上开关摩擦轮
                {
                    if (shoot_cmd_send.friction_mode == FRICTION_ON)
                        shoot_cmd_send.friction_mode = FRICTION_OFF;
                    else
                        shoot_cmd_send.friction_mode = FRICTION_ON;
                }
                break;

            case RC_SW_DOWN:

                if (gimbal_cmd_send.nuc_mode == none_version_control)
                {
                    if (rc_data[LAST].rc.switch_left == RC_SW_MID && shoot_cmd_send.friction_mode == FRICTION_ON)//左中到下且开摩擦轮时打弹
                    {
                        shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    }
                }
                else
                {
                    if (fire_advice == 1)
                    {
                        shoot_cmd_send.load_mode = LOAD_1_BULLET;
                    }
                }
                
                break;
            case RC_SW_MID:
                if (rc_data[LAST].rc.switch_left == RC_SW_DOWN)
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
                break;
        }

        switch (rc_data[TEMP].rc.switch_right)
        {
            case RC_SW_UP:
                if (rc_data[LAST].rc.switch_right == RC_SW_MID)
                {
                    if (chassis_cmd_send.chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW)
                        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                    else
                        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                }
                break;
            case RC_SW_DOWN:
                if (rc_data[LAST].rc.switch_right == RC_SW_MID)
                {
                    if (chassis_cmd_send.chassis_mode != CHASSIS_ROTATE)
                        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
                    else
                        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                }
                break;
        }
        
        rc_update_flag = 0;
    }
    

//    HeatControl();
    pitch_control += /*0.1**/RAD_TO_ANGLE*PITCH_K* (float)rc_data[TEMP].rc.rocker_l1 ;
    yaw_control -= /*0.05**/YAW_K * (float)rc_data[TEMP].rc.rocker_l_ ;
    // 底盘参数
    chassis_cmd_send.vx = 70.0f * (float)rc_data[TEMP].rc.rocker_r1; // 水平方向
    chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r_; // 竖直方向
    // if(chassis_cmd_send.chassis_mode==CHASSIS_NO_FOLLOW){
    //     gimbal_cmd_send.gimbal_mode=GIMBAL_MOTOR_MODE;
    //     if(gimbal_mode_last==GIMBAL_MOTOR_MODE){
    //     pitch_control += RAD_TO_ANGLE*PITCH_K* (float)rc_data[TEMP].rc.rocker_l1;
    //     yaw_control -= 2*YAW_K * (float)rc_data[TEMP].rc.rocker_l_;}
    //     else {
    //         yaw_control=chassis_fetch_data_uart.yaw_total_angle;
    //         }
    //     gimbal_mode_last=GIMBAL_MOTOR_MODE;
    // }
    // else {
    //     if(gimbal_mode_last==GIMBAL_GYRO_MODE){
    //     pitch_control += RAD_TO_ANGLE*PITCH_K* (float)rc_data[TEMP].rc.rocker_l1;
    //     yaw_control -= 2*YAW_K * (float)rc_data[TEMP].rc.rocker_l_;
    //     }
    //     else{
    //     yaw_control=gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg;
    //     }
    //     gimbal_mode_last=GIMBAL_GYRO_MODE;
    // }
    
    
        // pitch_control+=alpha_nuc_pitch*nuc_pitch+(1-alpha_nuc_pitch)*nuc_pitch_last;
        // nuc_yaw=yaw_filiter*nuc_yaw_last+(1-yaw_filiter)*nuc_yaw;
        
        // if(fabs(nuc_yaw)<7)yaw_control+=alpha_nuc_yaw*(nuc_yaw-nuc_yaw_d*(nuc_yaw-nuc_yaw_last));
        // yaw_nuc_err=nuc_yaw-nuc_yaw_last;
        // yaw_control_last=yaw_control;
        // nuc_pitch_last=nuc_pitch;
        // nuc_yaw_last=nuc_yaw;

        // if(gimbal_cmd_send.yaw_version<0){
        //     YAW_version_PID.Kp=0.01;
        //     YAW_version_PID.Kd=0.0015;
        // }
        // else {
        //     YAW_version_PID.Kp=0.012;
        //     YAW_version_PID.Kd=0.0014;
        // }
    
//     // // 云台参数
//     // 云台软件限位

    PitchAngle_ActiveLimit();// PITCH限位
    YawControlProcess();
    
     gimbal_cmd_send.yaw   = yaw_control;
     gimbal_cmd_send.pitch = pitch_control;    
}

ramp_t fb_ramp;
ramp_t lr_ramp;
ramp_t slow_ramp;

/**
 * @brief 键盘设定速度
 *
 */
static void ChassisSet()
{
    if(lob_mode) chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    else chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    // chassis_cmd_send.power_limit=100;//临时更改

    // 底盘移动
    static float current_speed_x = 0;
    static float current_speed_y = 0;
    // 前后移动
    // 防止逃跑时关小陀螺按Ctrl进入慢速模式
    if (rc_data[TEMP].key[KEY_PRESS].w) {
        chassis_cmd_send.vx = (current_speed_x + (CHASSIS_SPEED - current_speed_x) * ramp_calc(&fb_ramp)); // vx方向待测
        ramp_init(&slow_ramp, RAMP_TIME);                                                                  // 2000
    } else if (rc_data[TEMP].key[KEY_PRESS].s) {
        chassis_cmd_send.vx = (current_speed_x + (-CHASSIS_SPEED - current_speed_x) * ramp_calc(&fb_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w) { // 防止逃跑关小陀螺进入慢速移动
        chassis_cmd_send.vx = (current_speed_x + (4000 - current_speed_x) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s) {
        chassis_cmd_send.vx = (current_speed_x + (-4000 - current_speed_x) * ramp_calc(&slow_ramp));
        ramp_init(&fb_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vx = 0;
        ramp_init(&fb_ramp, RAMP_TIME);
    }

    // 左右移动
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        chassis_cmd_send.vy = (current_speed_y + (CHASSIS_SPEED - current_speed_y) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS].d) {
        chassis_cmd_send.vy = (current_speed_y + (-CHASSIS_SPEED - current_speed_y) * ramp_calc(&lr_ramp));
        ramp_init(&slow_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a) {
        chassis_cmd_send.vy = (current_speed_y + (+4000 - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d) {
        chassis_cmd_send.vy = (current_speed_y + (-4000 - current_speed_y) * ramp_calc(&fb_ramp));
        ramp_init(&lr_ramp, RAMP_TIME);
    } else {
        chassis_cmd_send.vy = 0;
        ramp_init(&lr_ramp, RAMP_TIME);
    }

    current_speed_x = chassis_cmd_send.vx;
    current_speed_y = chassis_cmd_send.vy;
}
float yaw_kx=500,pitch_ky=1000;
/**
 * @brief 鼠标移动云台
 *
 */
float freequence,pitch_vision_delta;
float lob_rate=0.1;
static void GimbalSet()
{
    

    // 相对角度控制
    // memcpy(&rec_yaw, vision_recv_data, sizeof(float));
    // memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));
    // pitch_control=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[0]+gimbal_cmd_send.pitch_version;
    // 按住鼠标右键且视觉识别到目标
    // if(lob_mode)
    // {
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_MOTOR_MODE;
    //     yaw_control -= rc_data[TEMP].mouse.x / 500.0f * lob_rate;
    //     pitch_control += rc_data[TEMP].mouse.y / 1000.0f * lob_rate;
    // }
    // else
    // {
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        yaw_control -= rc_data[TEMP].mouse.x / 500.0f;
        pitch_control += rc_data[TEMP].mouse.y / 1000.0f;
    // }
    pitch_vision_delta=gimbal_cmd_send.pitch_version/freequence;
    static float pitch_rotato_vision,yaw_rotato_vision;
    if(rc_data[TEMP].key[KEY_PRESS].e){
        if(fabs(gimbal_cmd_send.pitch_version)<16){
            pitch_rotato_vision=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            if(fabs(pitch_rotato_vision)<0.035)pitch_control-=pitch_rotato_vision;}
        // if(fabs(gimbal_cmd_send.pitch_version)<16)pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
        if(fabs(gimbal_cmd_send.yaw_version)<20){
            yaw_rotato_vision=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
            // yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
            if(fabs(yaw_rotato_vision<0.08))yaw_control-=yaw_rotato_vision;
        }
    }
    else if (rc_data[TEMP].mouse.press_r){
        // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
        // yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
        if(fabs(gimbal_cmd_send.pitch_version)<16){
            
            pitch_rotato_vision=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            // pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
            if(fabs(pitch_rotato_vision)<0.035)pitch_control-=pitch_rotato_vision;
            
        }
    }
    
    // if(fabs(gimbal_cmd_send.pitch_version)<16)pitch_control-=PIDCalculate(&PITCH_version_PID,gimbal_cmd_send.pitch_version,0);
    // if(fabs(gimbal_cmd_send.yaw_version)<20)yaw_control-=PIDCalculate(&YAW_version_PID,gimbal_cmd_send.yaw_version,0);
    YawControlProcess();
    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;
}

/**
 * @brief 键鼠设定机器人发射模式
 *
 */
static uint8_t mouse_r_last=0;
static void ShootSet()
{
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.shoot_rate = 30; // 射频默认30Hz

    // 仅在摩擦轮开启时有效
    if (shoot_cmd_send.friction_mode == FRICTION_ON) {
        // 打弹，单击左键单发，长按连发
        if (rc_data[TEMP].mouse.press_l) 
        {
            if (gimbal_cmd_send.nuc_mode == none_version_control)
            {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }
            else
            {
                if (fire_advice == 1)
                {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                }
                else
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
            }
            // 打符，单发
            // if (auto_rune == 1) {
            //     shoot_cmd_send.load_mode = LOAD_1_BULLET;
            // } else {
            //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // }
        } else {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    if (rc_data[TEMP].key[KEY_PRESS].g){
        shoot_cmd_send.load_mode = LOAD_REVERSE;
    }
    if(rc_data[TEMP].mouse.press_r)
    {
        gimbal_cmd_send.nuc_mode = version_control;
    }
    else
    {
        gimbal_cmd_send.nuc_mode = none_version_control;
    }
    mouse_r_last = rc_data[TEMP].mouse.press_r;
    HeatControl();
}

/**
 * @brief 键盘处理模式标志位
 *
 */

static void KeyGetMode()
{
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 1:
            if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
        case 0:
            if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) {
        case 1:
            if (shoot_cmd_send.friction_mode != FRICTION_ON)
                shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
    }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) {
    //     chassis_mode_e temp;
    //     temp=chassis_cmd_send.chassis_mode;
    //     case 1:
    //         if (chassis_cmd_send.chassis_mode == CHASSIS_REVERSE_ROTATE)
    //             chassis_cmd_send.chassis_mode = temp;
    //         break;
    //     case 0:
    //         if (chassis_cmd_send.chassis_mode == temp)
    //             chassis_cmd_send.chassis_mode = CHASSIS_REVERSE_ROTATE;
    //         break;
    // }
    switch (rc_data[TEMP].key[KEY_PRESS].r) {
        case 1:
            if (UI_SendFlag == 1) {
                UI_SendFlag = 0;
            }
            break;
        case 0:
            UI_SendFlag = 1;
            break;
    }
    
    switch (rc_data[TEMP].key[KEY_PRESS].ctrl) {
        case 1:
            auto_rune = 1;
            break;
        case 0:
            auto_rune = 0;
            break;
        default:
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) {
        case 1:
            SuperCap_flag_from_user = SUPER_USER_OPEN;
            break;
        case 0:
            SuperCap_flag_from_user = SUPER_USER_CLOSE;
            break;
    }
    switch(rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2){
        case 1:
            if(lob_mode==0)
            {
                lob_mode=1;
                 chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                telescope_pos = 1;
                fpv_pos = 1;
            }
        break;
        case 0:
            if(lob_mode)
            {
                lob_mode=0;
                 chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
                telescope_pos = 0;
                fpv_pos = 0;
            }
        break;
    }
}

/**
 * @brief 机器人复位函数，按下Ctrl+Shift+r
 *
 *
 */
static void RobotReset()
{
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].r) {
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
    if (rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].e) {
    chassis_cmd_send_uart.reset_flag=7;
    }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    ChassisSet();
    GimbalSet();
    ShootSet();
    KeyGetMode();
    
    PitchAngle_ActiveLimit();
    RobotReset(); // 机器人复位处理
}
static float  getkey_time= 0;
extern int32_t shoot_count; 
// extern referee_info_t *referee_data_for_ui;
int a=0;
uint8_t fifo_pack[NUC_RECV_SIZE];
uint8_t nuc_rx[8];
// short nuc_pitch,nuc_yaw;
void Version_devode()
{
    float pitch_version,yaw_version;
    pitch_version=2*3.14/8192*(float)(short)(nuc_rx[1]|nuc_rx[2]<<8);
    yaw_version=360.0/8192*(float)(short)(nuc_rx[3]|nuc_rx[4]<<8);
    if(fabs(yaw_version)>0.0001||yaw_version==0)gimbal_cmd_send.yaw_version=yaw_version;
    if(fabs(pitch_version)<0.0001||pitch_version==0)gimbal_cmd_send.pitch_version=pitch_version;

} 
#define checkNUC_num 20
float vision_filiter=1;
float checkNUCarray[2][checkNUC_num];
uint8_t ifTrueVision(float angle,uint8_t mode){
    checkNUCarray[mode][checkNUC_num]=angle;
    for(size_t i=0;i<checkNUC_num;i++){
        checkNUCarray[mode][i]=checkNUCarray[mode][i+1];
        }
    if(fabs(checkNUCarray[mode][0]-angle)< vision_filiter )return 1;
    else return 0;
}

 short pitch_version_cyc,yaw_version_cyc;
 float pitchChange,yawChange;

#define version_decode_to_angle 0.0439453125f
void USB_Version_devode(){
    //,yaw_version;
    // static float checkNUC[2];
    // static float checkNUCCCCCC[2];
    float fp_pitch,fp_yaw;
    // pitch_version_cyc=(short)(fifo_pack[1]|fifo_pack[2]<<8);
    // yaw_version_cyc=(short)(fifo_pack[3]|fifo_pack[4]<<8);
    // fp_pitch=(float)pitch_version_cyc*version_decode_to_angle;
    // fp_yaw=(float)yaw_version_cyc*version_decode_to_angle;
    // checkNUC[0]=fp_pitch;
    // checkNUC[1]=checkNUC[0];
    // checkNUCCCCCC[0]=fp_yaw;
    // checkNUCCCCCC[1]=checkNUCCCCCC[0];
    // pitchChange=fabs(checkNUC[0]-checkNUC[1]);
    // yawChange=fabs(checkNUCCCCCC[0]-checkNUCCCCCC[1]);
    // if(pitchChange<vision_filiter){
    //     gimbal_cmd_send.pitch_version=fp_pitch;
    // }
    // if(yawChange<vision_filiter){
    //     gimbal_cmd_send.yaw_version=fp_yaw;
    // }

    fire_advice = fifo_pack[1];
    fp_pitch = uint8_to_float_manual(fifo_pack + 4);
    fp_yaw = uint8_to_float_manual(fifo_pack + 8);
    if (fp_yaw != 0.0f)
    {
        gimbal_cmd_send.pitch_version = fp_pitch;
        gimbal_cmd_send.yaw_version = fp_yaw;
    }
    
    // if(ifTrueVision(fp_pitch,0)){
    //     if(fabs(fp_pitch)<10){
    //     gimbal_cmd_send.pitch_version=fp_pitch;
    //     }
    // }
    // if(ifTrueVision(fp_yaw,1)){
    //     if(fabs(fp_pitch)<10){
    //     gimbal_cmd_send.yaw_version=fp_yaw;}
    // }
}

uint8_t p_data[52];
float extract_value(const char *data, const char *key) {
    char *start = strstr(data, key);  // 查找键名的位置
    if (!start) return 0.0;           // 如果未找到，返回0

    start += strlen(key) + 1;         // 跳过键名和冒号
    return atof(start);               // 转换为浮点数
}
Chassis_Ctrl_Cmd_s_uart chassis_rs485_recv;
float voltage,current,power_value,energy;
extern Power_Data_s power_data; // 电机功率数据;
CAN_TxHeaderTypeDef can2_txheader;
uint8_t nuc_can_rx[8];
extern bool FIFO_READ_POWER(FIFOQueue *fifo, uint8_t *data, uint32_t len);
extern uint8_t calculate_checksum(uint8_t *data, uint8_t length);
CANInstance *nuc_Cantx;
CAN_TxHeaderTypeDef can2_txheader;
uint32_t can2_txmailbox;
float chassis_wzSet,chassis_wzK;
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
float refree_power_choice(uint8_t level){
    switch(level){
        case 1:
            return 70;
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
            return 70;
    }
}
float yawSpeedFeed;
float yaw_pid[3];
void RobotCMDTask()
{
#ifdef CHASSIS_BOARD 
//Judge_Uart();
DeterminRobotID();

//修改内容
// 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        MouseKeySet();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        EmergencyHandler(); // 调试/疯车时急停
    } 
    else {
        RemoteControlSet();
    }


    signl-=30;
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    chassis_rs485_recv.reset_flag=0;
    if(FIFO_Read_chassis_ctrl(&rs485_slaver_fifo,chasssis_ctrl_data,Chassis_Ctrl_Cmd_s_uart_size+3,0xff,0x0d)){
        signl=100;
        chassis_rs485_recv.vx=uint8_to_float_manual(chasssis_ctrl_data+2);
        chassis_rs485_recv.vy=uint8_to_float_manual(chasssis_ctrl_data+6);
        chassis_rs485_recv.wz=uint8_to_float_manual(chasssis_ctrl_data+10);
        chassis_rs485_recv.yaw_control=uint8_to_float_manual(chasssis_ctrl_data+14);
        chassis_rs485_recv.yaw_angle=uint8_to_float_manual(chasssis_ctrl_data+18);
        chassis_rs485_recv.yaw_gyro=uint8_to_float_manual(chasssis_ctrl_data+22);
        chassis_rs485_recv.offset_angle=uint8_to_float_manual(chasssis_ctrl_data+26);
        chassis_rs485_recv.gimbal_error_angle=uint8_to_float_manual(chasssis_ctrl_data+30);
        chassis_rs485_recv.shoot_count=uint8_array_to_int32(chasssis_ctrl_data+34);
        chassis_rs485_recv.nuc_yaw=uint8_to_float_manual(chasssis_ctrl_data+38);
        // chassis_rs485_recv.wz_set=uint8_to_float_manual(chasssis_ctrl_data+42);
        // chassis_rs485_recv.wz_K=uint8_to_float_manual(chasssis_ctrl_data+46);
        // chassis_rs485_recv.yaw_speedFeed=uint8_to_float_manual(chasssis_ctrl_data+50);
        // chassis_rs485_recv.yaw_kp=uint8_to_float_manual(chasssis_ctrl_data+54);
        // chassis_rs485_recv.yaw_kd=uint8_to_float_manual(chasssis_ctrl_data+58);
        // chassis_rs485_recv.yaw_speedKp=uint8_to_float_manual(chasssis_ctrl_data+62);
        chassis_rs485_recv.nuc_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-7];
        chassis_rs485_recv.UI_SendFlag=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-6];
        chassis_rs485_recv.superCap_flag=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-5];
        chassis_rs485_recv.reset_flag=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-4];
        chassis_rs485_recv.friction_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-3];
        chassis_rs485_recv.shoot_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-2];
        chassis_rs485_recv.load_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size-1];
        chassis_rs485_recv.chassis_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size];
        chassis_rs485_recv.gimbal_mode=chasssis_ctrl_data[Chassis_Ctrl_Cmd_s_uart_size+1];
    }
    if(chassis_rs485_recv.reset_flag==7){
        osDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset(); // 软件复位
    }
    chassis_fetch_data_uart.yaw_motor_single_round_angle=gimbal_fetch_data.yaw_motor_single_round_angle;
    chassis_fetch_data_uart.yaw_total_angle=gimbal_fetch_data.yaw_total_angle;
    chassis_fetch_data_uart.yaw_ecd=gimbal_fetch_data.yaw_ecd;
    chassis_fetch_data_uart.chassis_yaw_gyro=gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    chassis_fetch_data_uart.chassis_pitch_angle=gimbal_fetch_data.gimbal_imu_data->output.INS_angle[1];
    chassis_fetch_data_uart.initial_speed=referee_data->ShootData.bullet_speed;
    chassis_fetch_data_uart.color= referee_data->referee_id.Robot_Color;
    chassis_fetch_data_uart.yaw_angle_pidout=gimbal_fetch_data.yaw_angle_pidout;
    // gimbal_cmd_send.yaw_speedFeed=chassis_rs485_recv.yaw_speedFeed;
    gimbal_cmd_send.yaw=chassis_rs485_recv.yaw_control;
    gimbal_cmd_send.gimbal_mode=chassis_rs485_recv.gimbal_mode;
    gimbal_cmd_send.nuc_mode=chassis_rs485_recv.nuc_mode;
    gimbal_cmd_send.yaw_version=chassis_rs485_recv.nuc_yaw;
    // gimbal_cmd_send.yaw_speedFeed=chassis_rs485_recv.yaw_speedFeed;
    // gimbal_cmd_send.yaw_kp=chassis_rs485_recv.yaw_kp;
    // gimbal_cmd_send.yaw_kd=chassis_rs485_recv.yaw_kd;
    // gimbal_cmd_send.yaw_speedKp=chassis_rs485_recv.yaw_speedKp;
    shoot_cmd_send.load_mode=chassis_rs485_recv.load_mode;//修改内容
    shoot_cmd_send.shoot_mode=chassis_rs485_recv.shoot_mode;//修改内容
    shoot_cmd_send.shoot_count=chassis_rs485_recv.shoot_count;
    shoot_cmd_send.friction_mode=chassis_rs485_recv.friction_mode;//修改内容
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_42mm_cooling_rate, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_42mm_heat, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_42mm_cooling_limit, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));
    chassis_cmd_send.vx=chassis_rs485_recv.vx;
    chassis_cmd_send.vy=chassis_rs485_recv.vy;
    chassis_cmd_send.wz=chassis_rs485_recv.wz;
    // chassis_cmd_send.vw_set=chassis_rs485_recv.wz_set;
    chassis_cmd_send.chassis_mode=chassis_rs485_recv.chassis_mode;
    chassis_cmd_send.offset_angle=chassis_rs485_recv.offset_angle;
    chassis_cmd_send.gimbal_error_angle=chassis_rs485_recv.gimbal_error_angle;
    // chassis_cmd_send.wz_K=chassis_rs485_recv.wz_K;
    if(referee_data->GameRobotState.chassis_power_limit){
        chassis_cmd_send.power_limit=refree_power_choice(referee_data->GameRobotState.robot_level);
        //chassis_cmd_send.power_limit=100;//referee_data->GameRobotState.chassis_power_limit-10;
    }
    else chassis_cmd_send.power_limit=500;//100;//refree_power_choice(referee_data->GameRobotState.robot_level);
    g_power_set=chassis_cmd_send.power_limit;
    // chassis_cmd_send.power_limit=20;
    chassis_cmd_send.level=referee_data->GameRobotState.robot_level;
    chassis_cmd_send.power_buffer=referee_data->PowerHeatData.chassis_power_buffer;
    chassis_cmd_send.SuperCap_flag_from_user=chassis_rs485_recv.superCap_flag;
    if(pm01_od.v_out<1600)chassis_cmd_send.SuperCap_flag_from_user=0;
    // chassis_update_485(chassis_fetch_data_uart);
    if(signl<0){
        gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
        chassis_rs485_recv.chassis_mode=CHASSIS_ZERO_FORCE;
        shoot_cmd_send.load_mode=LOAD_STOP;//修改内容
        shoot_cmd_send.shoot_mode=SHOOT_OFF;//修改内容
    }
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);

    //memcpy(&power_value,&p_data[32],4);
    //power_value=p_data[32]<<24|p_data[33]<<16|p_data[34]<<8|p_data[35];
    //SubGetMessage(ui_feed_sub, &ui_fetch_data); 
    // UI
    referee_data_for_ui = referee_data;

    // memcpy(referee_data_for_ui, referee_data, sizeof(referee_info_t));
    memcpy(&ui_cmd_send.ui_send_flag, &chassis_rs485_recv.UI_SendFlag, sizeof(uint8_t));
    memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    memcpy(&ui_cmd_send.rune_mode, &auto_rune, sizeof(uint8_t));
    memcpy(&ui_cmd_send.SuperCap_mode, &chassis_rs485_recv.superCap_flag, sizeof(uint8_t));
    float power_value=0.01f*(float)pm01_od.v_out;
    memcpy(&ui_cmd_send.SuperCap_voltage, &power_value, sizeof(float));
    memcpy(&ui_cmd_send.Chassis_Ctrl_power, &chassis_fetch_data.chassis_power_output, sizeof(float));
    memcpy(&ui_cmd_send.Cap_absorb_power_limit, &chassis_fetch_data.capget_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&ui_cmd_send.Shooter_heat, &referee_data->PowerHeatData.shooter_42mm_heat, sizeof(uint16_t));
    memcpy(&ui_cmd_send.nuc_flag, &chassis_rs485_recv.nuc_mode, sizeof(uint8_t));
    // memcpy(&ui_cmd_send.Shooter_heat, &shoot_fetch_data.shooter_local_heat, sizeof(float));
    memcpy(&ui_cmd_send.robot_level,&referee_data->GameRobotState.robot_level,sizeof(uint8_t));
    // memcpy(&ui_cmd_send.Heat_Limit, &referee_data->GameRobotState.shooter_id1_42mm_cooling_limit, sizeof(uint16_t));
    PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    //CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif

#ifdef GIMBAL_BOARD 

//  CmdProcess();
// DeterminRobotID();
//Judge_Uart();
       //视觉
   if(FIFO_Read(&NUC_fifo,fifo_pack,NUC_RECV_SIZE,0XFF,0XFE))USB_Version_devode();
    // *nuc_can_rx=*(uint8_t*)CANCommGet(cmd_can_comm);
    //chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    if(FIFO_Read_chassis_ctrl(&rs485_master_fifo,chasssis_update_data,Chassis_Upload_Data_s_uart_size+3,0xff,0xfd)){
        chassis_fetch_data_uart.yaw_motor_single_round_angle=uint8_to_float_manual(chasssis_update_data+2);
        chassis_fetch_data_uart.yaw_total_angle=uint8_to_float_manual(chasssis_update_data+6);
        chassis_fetch_data_uart.yaw_ecd=uint8_to_float_manual(chasssis_update_data+10);
        chassis_fetch_data_uart.chassis_pitch_angle=uint8_to_float_manual(chasssis_update_data+14);
        chassis_fetch_data_uart.chassis_yaw_gyro=uint8_to_float_manual(chasssis_update_data+18);
        chassis_fetch_data_uart.initial_speed=uint8_to_float_manual(chasssis_update_data+22);
        chassis_fetch_data_uart.yaw_angle_pidout=uint8_to_float_manual(chasssis_update_data+26);
        // chassis_fetch_data_uart.color=chasssis_update_data[27];
        if(chassis_fetch_data_uart.color)vision_send_data[8]=1;
        else vision_send_data[8]=0;
    }
   SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
   SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
   chassis_cmd_send_uart.reset_flag=0;
   // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        MouseKeySet();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        EmergencyHandler(); // 调试/疯车时急停
    } 
    else {
        RemoteControlSet();
        PitchAngleLimit();
    }
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    // gimbal_fetch_data.yaw_motor_single_round_angle=chassis_fetch_data_uart.yaw_motor_single_round_angle;
    // gimbal_fetch_data.yaw_total_angle=chassis_fetch_data_uart.yaw_total_angle;
    CalcOffsetAngle();
    // shoot
   memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_42mm_cooling_rate, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_42mm_cooling_limit, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));
    chassis_cmd_send_uart.vx=chassis_cmd_send.vx;
    chassis_cmd_send_uart.vy=chassis_cmd_send.vy;
    chassis_cmd_send_uart.wz=chassis_cmd_send.wz;
    chassis_cmd_send_uart.offset_angle=chassis_cmd_send.offset_angle;
    chassis_cmd_send_uart.gimbal_error_angle=chassis_cmd_send.gimbal_error_angle;
    chassis_cmd_send_uart.chassis_mode=chassis_cmd_send.chassis_mode;
    // chassis_cmd_send_uart.chassis_mode=CHASSIS_ZERO_FORCE;
    chassis_cmd_send_uart.gimbal_mode=gimbal_cmd_send.gimbal_mode;
    chassis_cmd_send_uart.yaw_control=gimbal_cmd_send.yaw;
    chassis_cmd_send_uart.yaw_angle=gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
    chassis_cmd_send_uart.yaw_gyro=gimbal_fetch_data.gimbal_imu_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET];
    chassis_cmd_send_uart.nuc_yaw=gimbal_cmd_send.yaw_version;
    chassis_cmd_send_uart.shoot_mode=shoot_cmd_send.shoot_mode;
    chassis_cmd_send_uart.load_mode=shoot_cmd_send.load_mode;
    chassis_cmd_send_uart.shoot_count=shoot_count;
    chassis_cmd_send_uart.friction_mode=shoot_cmd_send.friction_mode;
    chassis_cmd_send_uart.nuc_mode=gimbal_cmd_send.nuc_mode;
    // chassis_cmd_send_uart.yaw_speedFeed=yawSpeedFeed;
    // chassis_cmd_send_uart.wz_K=chassis_wzK;
    // chassis_cmd_send_uart.wz_set=chassis_wzSet;
    chassis_cmd_send_uart.UI_SendFlag=UI_SendFlag;
    chassis_cmd_send_uart.superCap_flag=SuperCap_flag_from_user;
    // chassis_cmd_send_uart.yaw_kp=yaw_pid[0];
    // chassis_cmd_send_uart.yaw_kd=yaw_pid[1];
    // chassis_cmd_send_uart.yaw_speedKp=yaw_pid[2];
    chassis_ctrl_485(chassis_cmd_send_uart);//给底盘发送消息


    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send); 

    //向上位机发送消息
    vision_send_data[0]=0xff; 
    vision_send_data[1] = chassis_fetch_data_uart.color;
    float_to_uint8_manual(gimbal_fetch_data.gimbal_imu_data->output.INS_angle[1], vision_send_data + 4);//低位先行
    float_to_uint8_manual(gimbal_fetch_data.gimbal_imu_data->output.INS_angle[2], vision_send_data + 8);
    static uint8_t nuc_cnt = 25;
    nuc_cnt = (nuc_cnt + 1) % 25 + 25;
    vision_send_data[30]=nuc_cnt;
    vision_send_data[31]=0xfe;
    HostSend(host_instance, vision_send_data, 32);

#endif
    // 从其他应用获取回传数据
#ifdef ONE_BOARD  
//Judge_Uart();
    if(FIFO_Read(&NUC_fifo,fifo_pack,NUC_RECV_SIZE,0XFF,0XFE))Version_devode();
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
   SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
   SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
   // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) && (switch_is_down(rc_data[TEMP].rc.switch_right))) // 遥控器拨杆右[上]左[下],键鼠控制
        MouseKeySet();
    else if (RC_LOST || (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))) {
        EmergencyHandler(); // 调试/疯车时急停
    } 
    else {
        RemoteControlSet();
    }
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // shoot
   memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotState.shooter_id1_17mm_cooling_rate, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_heat0, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));
   memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    
 // chassis
    memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotState.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.SuperCap_flag_from_user, &SuperCap_flag_from_user, sizeof(uint8_t)); 
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
//待修改
    if(shoot_cmd_send.load_mode==LOAD_STOP){
    shoot_cmd_send.shoot_aim_angle = shoot_fetch_data.loader_angle - ONE_BULLET_DELTA_ANGLE;
    shoot_cmd_send.Shoot_Once_Flag = 1;}
    else if (shoot_cmd_send.load_mode==LOAD_1_BULLET){
        if(fabs(shoot_fetch_data.loader_angle-shoot_cmd_send.shoot_aim_angle)<0.1){
            shoot_cmd_send.Shoot_Once_Flag = 0;
        }
    }



    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    
    // 设置视觉发送数据,还需增加加速度和角速度数据

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置

    // master
    // static uint8_t frame_head[] = {0xAF, 0x32, 0x00, 0x12};
    // memcpy(vision_send_data, frame_head, 4);
    // memcpy(vision_send_data + 4, gimbal_fetch_data.gimbal_imu_data->INS_data.INS_quat, sizeof(float) * 4);
    // memcpy(vision_send_data + 20, &referee_data->GameRobotState.robot_id, sizeof(uint8_t));
    // memcpy(vision_send_data + 21, &auto_rune, sizeof(uint8_t));
    // vision_send_data[22] = 0;
    // for (size_t i = 0; i < 22; i++)
    //     vision_send_data[22] += vision_send_data[i];

}

