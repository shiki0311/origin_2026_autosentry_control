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

#include "INS_Task_ekf.h"

#if USE_EKF == 1

#include "cmsis_os.h"

#include "bsp_Cboard_imu_pwm.h"
#include "bsp_spi.h"
#include "bsp_dwt.h"
#include "bmi088driver.h"
#include "QuaternionEKF.h"
#include "detect_task.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {-1.0f, 0.0f, 0.0f},                 \
        {0.0f, 0.0f, -1.0f},             \
        {0.0f, -1.0f, 0.0f}

/**
 * @description: 初始化ins task
 * @return {*}
 */
static void INS_init(void);

/**
 * @brief          旋转陀螺仪,加速度计,将加速度计和减去零偏后的陀螺仪数据从imu坐标系转换到云台坐标系
 * @param[out]     gyro: 陀螺仪数据，顺序：roll pitch yaw
 * @param[out]     accel: 加速度计数据，顺序：roll pitch yaw
 * @param[in]      bmi088: 陀螺仪和加速度计的原始数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088, const fp32 gyro_offset[3]);

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp);

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

/* 等待任务通知并读取最新的IMU数据到 bmi088_real_data */
static void imu_read_wait_and_fetch(void);

/**
 * @brief          加速度计三轴数据二阶递归滤波（输出参数版本，推荐）
 * @param[in]      raw_accel: 原始加速度数据数组指针，长度为3
 * @param[in]      coeff:     滤波器系数数组指针，长度为3
 * @param[out]     out:       输出滤波结果数组指针，长度为3
 * @retval         none
 */
static void Accel_Filter_Update_2D_Out(const float raw_accel[3], const float coeff[3], float out[3]);

/**
 * @brief          计算IMU零漂
 * @param[in]      none
 * @retval         none
 */
static void mpu_offset_clc(void);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_Task_local_handler;

INS_ekf_t INS;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

volatile uint8_t imu_read_flag = 0;

uint8_t calibration_done = 0;

bmi088_real_data_t bmi088_real_data;

fp32 gyro_offset_data[3] = {0.00039f, 0.00498476837f, 0.0030584302f}; // 陀螺仪零偏补偿
static const fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static const fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
// 加速度计低通滤波系数
static const fp32 accel_fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/**
 * @description: 初始化ins task
 * @return {*}
 */
static void INS_init(void)
{
    const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
    PID_init(&INS.imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    IMU_QuaternionEKF_Init(10, 0.001, 6000000, 0.997, 0); // 初始化卡尔曼滤波

    DWT_Init(CPU_FREQ_MHZ); // 启动DWT，用于高精度计时

    // get the handle of task
    // 获取当前任务的任务句柄，
    INS_Task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
}
void INS_Task(void const *pvParameters)
{
    while (BMI088_init())
    {
        osDelay(100);
    }

    while (ist8310_init())
    {
        osDelay(100);
    }

    INS_init();

#if IMU_CALIBRATION_MODE
    /* ============ IMU校准模式 ============ */
    // 等待温度稳定,然后进行IMU零漂标定
    static uint16_t temp_stable_count = 0;
    static uint8_t calibration_start = 0;

    while (1)
    {
        imu_read_wait_and_fetch();

        // 温度控制
        imu_temp_control(bmi088_real_data.temp);

        if (calibration_start && !calibration_done)
            mpu_offset_clc();
        // 检查温度是否稳定（在目标温度附近）
        if (!calibration_start)
        {
            if (fabsf(bmi088_real_data.temp - IMU_Temp_Set) < TEMP_STABLE_THRESHOLD)
            {
                temp_stable_count++;
                if (temp_stable_count >= TEMP_STABLE_TIME_COUNT)
                {
                    // 温度稳定,开始IMU零漂标定
                    calibration_start = 1;
                }
            }
            else
            {
                // 温度变化超过阈值,重新计数
                temp_stable_count = 0;
            }
        }
    }

#else
    /* ============ 正常姿态解算模式 ============ */
    // 标记是否是第一次进入while(1)循环

    while (1)
    {
        static uint8_t first_loop_done = 1;

        // wait spi DMA tansmit done
        // 等待SPI DMA传输并读取IMU数据
        imu_read_wait_and_fetch();

        float dt; // 时间间隔
        if (!first_loop_done)
        {
            dt = DWT_GetDeltaT(&INS.INS_DWT_Count);
        }
        else
        {
            dt = 0.001;
            INS.INS_DWT_Count = DWT_GetTimeline_s();
            first_loop_done = 0;
        }

        // 陀螺仪恒温控制
        imu_temp_control(bmi088_real_data.temp);
        // 将加速度计和减去零漂以后的陀螺仪数据从imu坐标系转换到云台坐标系
        imu_cali_slove(INS.Gyro, INS.Accel, &bmi088_real_data, gyro_offset_data);

        // 加速度计低通滤波
        fp32 INS_accel_filtered[3] = {0};
        Accel_Filter_Update_2D_Out(INS.Accel, accel_fliter_num, INS_accel_filtered);

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[AXIS_X], INS.Gyro[AXIS_Y], INS.Gyro[AXIS_Z], INS_accel_filtered[AXIS_X], INS_accel_filtered[AXIS_Y], INS_accel_filtered[AXIS_Z], dt);

        // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        // 让 EKF 运行若干次以收敛（warm-up），然后再通知 Gimbal_Task 启动
        {
            static uint16_t ekf_warmup_counter = 0;
            if (ekf_warmup_counter >= 1000)
            {
                xSemaphoreGive(ins_init_done_semaphore); // 释放信号量
            }
            else
                ekf_warmup_counter++;
        }
    }

#endif
}

static void imu_read_wait_and_fetch(void)
{
    // wait spi DMA transmit done
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    {
    }

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
    }
}

/**
 * @brief          旋转陀螺仪,加速度计,将加速度计和减去零偏后的陀螺仪数据从imu坐标系转换到云台坐标系
 * @param[out]     gyro: 陀螺仪数据，顺序：roll pitch yaw
 * @param[out]     accel: 加速度计数据，顺序：roll pitch yaw
 * @param[in]      bmi088: 陀螺仪和加速度计的原始数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088, const fp32 gyro_offset[3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = (bmi088->gyro[0] - gyro_offset[0]) * gyro_scale_factor[i][0] + (bmi088->gyro[1] - gyro_offset[1]) * gyro_scale_factor[i][1] + (bmi088->gyro[2] - gyro_offset[2]) * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2];
    }
}

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    static uint8_t first_temperate = 0;

    if (first_temperate)
    {
        PID_calc(&INS.imu_temp_pid, temp, IMU_Temp_Set);
        if (INS.imu_temp_pid.out < 0.0f)
        {
            INS.imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)INS.imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        // 在没有达到设置的温度，一直最大功率加热
        // in beginning, max power
        if (temp > IMU_Temp_Set)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // 达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                INS.imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief          加速度计三轴数据二阶递归滤波（输出参数版本，推荐）
 * @param[in]      raw_accel: 原始加速度数据数组指针，长度为3
 * @param[in]      coeff:     滤波器系数数组指针，长度为3
 * @param[out]     out:       输出滤波结果数组指针，长度为3
 * @retval         none
 */
void Accel_Filter_Update_2D_Out(const float raw_accel[3], const float coeff[3], float out[3])
{
    static float filter_states[3][3] = {0};
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        filter_states[axis][0] = filter_states[axis][1];
        filter_states[axis][1] = filter_states[axis][2];
        filter_states[axis][2] = filter_states[axis][1] * coeff[0] +
                                 filter_states[axis][0] * coeff[1] +
                                 raw_accel[axis] * coeff[2];
        out[axis] = filter_states[axis][2];
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        imu_read_flag = 1;
        // wake up the task
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_Task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{

    // 开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    // 开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

void DMA2_Stream2_IRQHandler_1(void)
{

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // gyro read over
        // 陀螺仪读取完毕
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // accel read over
        // 加速度计读取完毕
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // temperature read over
        // 温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
 * @brief          计算IMU零漂
 * @param[in]      none
 * @retval         none
 */
static void mpu_offset_clc(void)
{
    static uint16_t i = 0;
    if (i <= 8000)
    {
        if (i < 8000)
        {
            gyro_offset_data[0] += bmi088_real_data.gyro[0];
            gyro_offset_data[1] += bmi088_real_data.gyro[1];
            gyro_offset_data[2] += bmi088_real_data.gyro[2];
        }
        else if (i == 8000)
        {
            gyro_offset_data[0] = gyro_offset_data[0] / 8000.0f;
            gyro_offset_data[1] = gyro_offset_data[1] / 8000.0f;
            gyro_offset_data[2] = gyro_offset_data[2] / 8000.0f;

            // 标定完成
            calibration_done = 1;
        }
        i++;
    }
}

#endif