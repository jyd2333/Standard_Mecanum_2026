/***
 * @Author       : HDC h2019dc@outlook.com
 * @Date         : 2023-09-08
 * @LastEditors  : HDC h2019dc@outlook.com
 * @LastEditTime : 2023-11-09
 * @FilePath     : \2024_Control_New_Framework_Base-dev-all\modules\imu\ins_task.c
 * @Description  : 陀螺仪解算部分，参考官方陀螺仪
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 */

#include "ins_task.h"
#include "QuaternionEKF.h"
#include "general_def.h"
#include "robot_params.h"
// 旋转与零漂
static float gyro_scale_factor[3][3]  = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static float gyro_offset[3]           = {0.0f, 0.0f, 0.0f};
static float accel_offset[3]          = {0.0f, 0.0f, 0.0f};
//  加速度计低通滤波
static float accel_fliter_1[3]   = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3]   = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3]   = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
static uint8_t qekf_initialized   = 0;

/* Quaternion EKF tunings */
#define QEKF_PROCESS_NOISE1 (10.0f)
#define QEKF_PROCESS_NOISE2 (0.001f)
#define QEKF_MEASURE_NOISE  (1000000.0f)
#define QEKF_LAMBDA         (0.9996f)
#define QEKF_ACCEL_LPF      (0.0f) // 外部已做acc低通,EKF内部LPF关闭

static INS_Instance *INS       = NULL;
#include <math.h>
#include <stdbool.h>

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
inline static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], BMI088_Data_t *data)
{
    for (uint8_t i = 0; i < 3; i++) {
        gyro[i]  = data->gyro[0] * gyro_scale_factor[i][0] + data->gyro[1] * gyro_scale_factor[i][1] + data->gyro[2] * gyro_scale_factor[i][2] - gyro_offset[i];
        accel[i] = data->acc[0] * accel_scale_factor[i][0] + data->acc[1] * accel_scale_factor[i][1] + data->acc[2] * accel_scale_factor[i][2] ;//- accel_offset[i];
        mag[i]   = 0; // 不使用磁力计
    }
}
INS_Instance *INS_Init(BMI088Instance *bmi088)
{
    INS_Instance *INSinstance = (INS_Instance *)zmalloc(sizeof(INS_Instance));
    INSinstance->BMI088       = bmi088;

    for (uint8_t i = 0; i < 3; i++)
        gyro_offset[i] = bmi088->gyro_offset[i];

    BMI088_Data_t raw_data;
    BMI088Acquire(bmi088, &raw_data);
    imu_cali_slove(INSinstance->INS_data.INS_gyro, INSinstance->INS_data.INS_accel, INSinstance->INS_data.INS_mag, &raw_data);
    //  赋值四元数初值
    INSinstance->INS_data.INS_quat[0] = 1;
    INSinstance->INS_data.INS_quat[1] = 0;
    INSinstance->INS_data.INS_quat[2] = 0;
    INSinstance->INS_data.INS_quat[3] = 0;

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INSinstance->INS_data.INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INSinstance->INS_data.INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INSinstance->INS_data.INS_accel[2];

    IMU_QuaternionEKF_Init(INSinstance->INS_data.INS_quat,
                           QEKF_PROCESS_NOISE1,
                           QEKF_PROCESS_NOISE2,
                           QEKF_MEASURE_NOISE,
                           QEKF_LAMBDA,
                           QEKF_ACCEL_LPF);
    qekf_initialized = 1;

    INS = INSinstance;


    return INSinstance;
}

void INS_Task()
{
    static BMI088_Data_t raw_data;
    BMI088Acquire(INS->BMI088, &raw_data);

    // 设置环境温度 冷启动适用
    // if (INS->BMI088->ambient_temperature < -270) {
    //     INS->BMI088->ambient_temperature = INS->BMI088->temperature;
    // }
    INS->BMI088->ambient_temperature = BMI088_AMBIENT_TEMPERATURE;
    BMI088_temp_control(INS->BMI088);
    // 旋转与零漂
    imu_cali_slove(INS->INS_data.INS_gyro, INS->INS_data.INS_accel, INS->INS_data.INS_mag, &raw_data);

    // 加速度计低通滤波
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS->INS_data.INS_accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS->INS_data.INS_accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS->INS_data.INS_accel[2] * fliter_num[2];

    INS->timing_time = DWT_GetDeltaT(&INS->BMI088->bias_dwt_cnt);
    if (!qekf_initialized) {
        IMU_QuaternionEKF_Init(INS->INS_data.INS_quat,
                               QEKF_PROCESS_NOISE1,
                               QEKF_PROCESS_NOISE2,
                               QEKF_MEASURE_NOISE,
                               QEKF_LAMBDA,
                               QEKF_ACCEL_LPF);
        qekf_initialized = 1;
    }

    IMU_QuaternionEKF_Update(INS->INS_data.INS_gyro[0],
                             INS->INS_data.INS_gyro[1],
                             INS->INS_data.INS_gyro[2],
                             accel_fliter_3[0],
                             accel_fliter_3[1],
                             accel_fliter_3[2],
                             INS->timing_time);

    for (uint8_t i = 0; i < 4; i++) {
        INS->INS_data.INS_quat[i] = QEKF_INS.q[i];
    }
    for (uint8_t i = 0; i < 3; i++) {
        INS->INS_data.INS_gyro[i] = QEKF_INS.Gyro[i];
        INS->INS_data.INS_imu_6axis[i] = INS->INS_data.INS_gyro[i];
        INS->INS_data.INS_imu_6axis[i + 3] = accel_fliter_3[i];
        INS->BMI088->imu_6axis[i] = INS->INS_data.INS_imu_6axis[i];
        INS->BMI088->imu_6axis[i + 3] = INS->INS_data.INS_imu_6axis[i + 3];
    }

    INS->output.INS_angle[INS_YAW_ADDRESS_OFFSET] = QEKF_INS.Yaw * DEGREE_2_RAD;
    INS->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] = QEKF_INS.Pitch * DEGREE_2_RAD;
    INS->output.INS_angle[INS_ROLL_ADDRESS_OFFSET] = QEKF_INS.Roll * DEGREE_2_RAD;

    INS->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET] = QEKF_INS.Yaw;
    INS->output.INS_angle_deg[INS_PITCH_ADDRESS_OFFSET] = QEKF_INS.Pitch;
    INS->output.INS_angle_deg[INS_ROLL_ADDRESS_OFFSET] = QEKF_INS.Roll;
    INS->output.Yaw_total_angle_deg = QEKF_INS.YawTotalAngle;
    INS->output.Yaw_total_angle = QEKF_INS.YawTotalAngle * DEGREE_2_RAD;
}
