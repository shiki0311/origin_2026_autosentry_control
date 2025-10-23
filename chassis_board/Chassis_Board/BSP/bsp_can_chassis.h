/*****************************************************************************************************************************
 * @file: bsp_can.h
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2026赛季CAN通讯支持包,此文件适配底盘C板
 *****************************************************************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

typedef enum
{
    CAN_STEER_GM6020_CMD,
    CAN_WHEEL_M3508_CMD,
    CAN_CAP_CMD,
} CAN_CMD_ID; // CAN发送命令类型,用于把不同的can消息送入对应消息队列统一发送

typedef struct
{
    char s[2];
    int16_t ch[5];
} chassis_rc_ctrl_t; // 底盘任务需要使用的遥控器数据，由云台C板通过can传到底盘C板

typedef struct
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} CanTxMsgTypeDef; // CAN报文结构体，用于can发送队列

extern chassis_rc_ctrl_t chassis_rc_ctrl;
extern QueueHandle_t CAN1_send_queue; // CAN1消息队列句柄,此队列用于储存CAN1的发送消息
extern QueueHandle_t CAN2_send_queue; // CAN2消息队列句柄，此队列用于储存CAN2的发送消息

void Can_Filter_Init(void);
void Can_Msg_Init(void);
void Create_Can_Send_Queues(void);
void Allocate_Can_Msg(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id);

#endif
