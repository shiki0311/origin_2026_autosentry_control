/********************************************************************************************************************************************
 * @file: Can_Timer_Task.h
 * @author: ushadff,shiki
 * @date: 2025.10.22
 * @brief:	在freertos定时器任务中取出can发送队列中的can消息并发送
 * *******************************************************************************************************************************************/

#ifndef CAN_SEND_TASK_H
#define CAN_SEND_TASK_H

void Create_Can_Send_Timers(void);

#define TIME_STAMP_1MS 1
#define TIME_STAMP_2MS 2

#endif
