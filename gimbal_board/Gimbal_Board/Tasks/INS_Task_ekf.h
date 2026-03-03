/**
  ****************************(C) COPYRIGHT 2026 Shiki****************************
  * @file       INS_task_ekf.c/h
  * @brief      主要利用陀螺仪bmi088，使用ekf完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088
  *  V3.0.0     Feb-7-2026      Shiki           1. transfer to ekf(refer to wanghongxi)
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Shiki****************************
  */
#ifndef INS_Task_EKF_H
#define INS_Task_EKF_H

#include "main.h"
#if USE_EKF == 1

#include "struct_typedef.h"
#include "bmi088driver.h"
#include "pid.h"

/* INS_task工作模式选择：1为校准模式（标定零漂），0为正常模式（姿态解算） */
#define IMU_CALIBRATION_MODE 0

/* 温度稳定判断参数（IMU校准模式下使用） */
#define TEMP_STABLE_THRESHOLD    0.5f    /* 温度稳定判断阈值（度） */
#define TEMP_STABLE_TIME_COUNT   3000     /* 温度需要稳定的次数 */

#define IMU_Temp_Set 25

#ifndef AXIS_X
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#endif

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

typedef struct
{
  float Gyro[3];          // 角速度
  float Accel[3];         // 加速度

  // 位姿
  float Roll;
  float Pitch;
  float Yaw;
  float YawTotalAngle;

  uint32_t INS_DWT_Count;

  pid_type_def imu_temp_pid; //bmi088温度控制PID
} INS_ekf_t;
extern INS_ekf_t INS;

extern bmi088_real_data_t bmi088_real_data;

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_Task(void const *pvParameters);

void DMA2_Stream2_IRQHandler_1(void);

#endif

#endif