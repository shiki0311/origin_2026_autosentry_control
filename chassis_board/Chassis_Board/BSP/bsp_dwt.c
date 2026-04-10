/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Shiki
 * @version V2.0.0
 * @date    2025/9/13
 * @brief
 */

#include "bsp_dwt.h"
#include "cmsis_os.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;
static osMutexId DWT_MUTEX;

/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出,需要保证函数调用的间隔小于一次溢出时间
 *
 */
static void DWT_CNT_Update(void)
{
    if (__get_CONTROL()) // 不在中断中,使用互斥锁;在中断则直接执行即可
        if (osOK != osMutexWait(DWT_MUTEX, 0))
            return;

    volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++;

    CYCCNT_LAST = DWT->CYCCNT;
    osMutexRelease(DWT_MUTEX);
}

static uint8_t is_dwt_initialized = 0; // DWT初始化状态标志

uint8_t DWT_Init(uint32_t CPU_Freq_mHz)
{
    // 如果已经成功初始化过，不再重复初始化，直接返回0（成功状态）
    if (is_dwt_initialized)
    {
        return 0;
    }

    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    // 创建互斥锁前确保互斥锁未被创建，避免内存泄漏
    if (DWT_MUTEX == NULL)
    {
        osMutexDef(dwt_mutex);
        DWT_MUTEX = osMutexCreate(osMutex(dwt_mutex));
    }

    DWT_CNT_Update();
    if (DWT->CYCCNT)
    {
        is_dwt_initialized = 1; // 标记初始化成功
        return 0;               /*clock cycle counter started*/
    }
    else
    {
        return 1; /*clock cycle counter not started*/
    }
}

void DWT_DeInit(void)
{

    // 1. 关闭 CYCCNT 计数器（停止计数）
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; // 清除 CYCCNTENA 位（bit0）

    // 2. 关闭 DWT 模块总使能（停止 DWT 外设时钟）
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // 清除 TRCENA 位（bit24）

    // 3. 若使用 RTOS 互斥锁，释放互斥锁资源（避免内存泄漏）
    if (DWT_MUTEX != NULL)
    {
        osMutexDelete(DWT_MUTEX); // 删除互斥锁
        DWT_MUTEX = NULL;
    }

    // 4. 重置全局变量（可选，根据需求决定是否清零状态）
    CPU_FREQ_Hz = 0;
    CPU_FREQ_Hz_ms = 0;
    CPU_FREQ_Hz_us = 0;
    CYCCNT_RountCount = 0;
    CYCCNT_LAST = 0;
    CYCCNT64 = 0;
    SysTime.s = 0;
    SysTime.ms = 0;
    SysTime.us = 0;

    // 5. 清除初始化标志
    is_dwt_initialized = 0; // 清除初始化标志
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

void DWT_Delay_ms(float Delay_ms)
{
    if (Delay_ms <= 0.0f)
        return;

    float start_ms = DWT_GetTimeline_ms();

    float target_ms = start_ms + Delay_ms;

    while (1)
    {
        float current_ms = DWT_GetTimeline_ms();
        if (current_ms >= target_ms)
            break;
    }
}

void DWT_Delay_us(uint64_t Delay_us)
{
    uint64_t start_us = DWT_GetTimeline_us();

    uint64_t target_us = start_us + Delay_us;

    while (1)
    {
        float current_us = DWT_GetTimeline_us();
        if (current_us >= target_us)
            break;
    }
}
