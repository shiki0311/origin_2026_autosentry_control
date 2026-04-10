/********************************************************************************************************************************************
 * @file: Can_Send_Task.c
 * @author: ushadff
 * @date: 2025.10.22
- * @brief:	在freertos定时器任务中取出can发送队列中的can消息并发送
 * *******************************************************************************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "Can_Timer_Task.h"
#include "bsp_can_chassis.h"
#include "can.h"

TimerHandle_t CAN1_Timer_Handle; // 周期定时器句柄
TimerHandle_t CAN2_Timer_Handle; // 周期定时器句柄

/**
 * @brief  软件定时回调函数
 * @retval void
 * @attention can队列禁用等待，禁用delay
 */
void CAN1_Timer_Callback(TimerHandle_t xTimer)
{
    CanTxMsgTypeDef SendCanTxMsg;
    uint32_t send_mail_box;

    if (xQueueReceive(CAN1_send_queue, &SendCanTxMsg, 0)) // 接收队列信息,禁止等待！！！
    {
        // 寻空邮箱发送数据，发送失败继续尝试发送，最多发三次
        if (HAL_CAN_AddTxMessage(&hcan1, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box) != HAL_OK)
        {
            if (HAL_CAN_AddTxMessage(&hcan1, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box) != HAL_OK)
            {
                HAL_CAN_AddTxMessage(&hcan1, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box);
            }
        }
    }
}

/**
 * @brief  软件定时回调函数
 * @retval void
 * @attention can队列禁用等待，禁用delay
 */
void CAN2_Timer_Callback(TimerHandle_t xTimer)
{
    CanTxMsgTypeDef SendCanTxMsg;
    uint32_t send_mail_box;

    if (xQueueReceive(CAN2_send_queue, &SendCanTxMsg, 0)) // 接收队列信息,禁止等待！！！
    {
        // 寻空邮箱发送数据，发送失败继续尝试发送，最多发三次
        if (HAL_CAN_AddTxMessage(&hcan2, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box) != HAL_OK)
        {
            if (HAL_CAN_AddTxMessage(&hcan2, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box) != HAL_OK)
            {
                HAL_CAN_AddTxMessage(&hcan2, &SendCanTxMsg.tx_header, SendCanTxMsg.data, &send_mail_box);
            }
        }
    }
}

void Create_Can_Send_Timers(void)
{

    taskENTER_CRITICAL(); // 进入临界区


    // 创建can1发送定时器
    CAN1_Timer_Handle = xTimerCreate((const char *)"CAN1_Timer",
                                     (TickType_t)TIME_STAMP_1MS,                    // 1ms
                                     (UBaseType_t)pdTRUE,                           // 周期执行
                                     (void *)0,                                     // 编号一般给0
                                     (TimerCallbackFunction_t)CAN1_Timer_Callback); // 回调函数

    // 开启CAN1定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
    if (CAN1_Timer_Handle != NULL)
    {
        xTimerStart(CAN1_Timer_Handle, 0); // 不等待
    }

    // 创建can2发送定时器
    CAN2_Timer_Handle = xTimerCreate((const char *)"CAN2_Timer",
                                     (TickType_t)TIME_STAMP_1MS,                    // 1ms
                                     (UBaseType_t)pdTRUE,                           // 周期执行
                                     (void *)1,                                     // 编号一般给0
                                     (TimerCallbackFunction_t)CAN2_Timer_Callback); // 回调函数

    // 开启CAN2定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
    if (CAN2_Timer_Handle != NULL)
    {
        xTimerStart(CAN2_Timer_Handle, 0); // 不等待
    }

    taskEXIT_CRITICAL(); // 退出临界区
}
