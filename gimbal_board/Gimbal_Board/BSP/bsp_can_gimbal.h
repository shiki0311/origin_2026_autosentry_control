/*****************************************************************************************************************************
 * @file: bsp_can.h
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2025赛季CAN通讯支持包，此文件适配云台C板
 *****************************************************************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define BIG_YAW_DM6006_TransID 0x01 // CAN2
#define CAN_TX_DIV2 2
#define CAN_TX_DIV3 3
#define CAN_TX_DIV4 4
#define CAN_TX_DIV5 5

typedef enum
{
    CAN_RC_TO_CHASSIS_FIRST_CMD,
    CAN_RC_TO_CHASSIS_SECOND_CMD,
    CAN_BIG_YAW_CMD,
    CAN_SMALL_YAW_AND_PITCH_CMD,
    CAN_FRIC_CMD,
    CAN_DIAL_CMD
} CAN_CMD_ID; // CAN发送命令类型,用于把不同的can消息送入对应消息队列统一发送

typedef struct
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8];
} CanTxMsgTypeDef; // CAN报文结构体，用于can发送队列中

extern QueueHandle_t CAN1_send_queue; // CAN1消息队列句柄,此队列用于储存CAN1的发送消息
extern QueueHandle_t CAN2_send_queue; // CAN2消息队列句柄，此队列用于储存CAN2的发送消息

void Can_Filter_Init(void);
void Can_Msg_Init(void);
void Create_Can_Send_Queues(void);
void Allocate_Can_Msg(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id);
void Ctrl_DM_Motor(float _pos, float _vel, float _KP, float _KD, float _torq);
void enable_DM(uint8_t id, uint8_t ctrl_mode);
void disable_DM(uint8_t id, uint8_t ctrl_mode);

#endif
