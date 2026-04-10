/*****************************************************************************************************************************
 * @file: bsp_can_chassis.h
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

/*********************************CAN接收ID*******************************************/
#define BIG_YAW_DM6006_RecID 0x300           // CAN2,下板需要接收大yaw的电机位置数据用于底盘跟随云台
#define GIMBAL_TO_CHASSIS_FIRST_RecID 0x10  // CAN2
#define GIMBAL_TO_CHASSIS_SECOND_RecID 0x11 // CAN2
#define STEER1_GM6020_RecID 0x205            // CAN1
#define STEER2_GM6020_RecID 0x206            // CAN1
#define STEER3_GM6020_RecID 0x207            // CAN1
#define STEER4_GM6020_RecID 0x208            // CAN1

#define WHEEL1_M3508_RecID 0x201 // CAN1
#define WHEEL2_M3508_RecID 0x202 // CAN1
#define WHEEL3_M3508_RecID 0x203 // CAN1
#define WHEEL4_M3508_RecID 0x204 // CAN1
#define CAP_RecID 0x130          // CAN1 超电
#define POWER_METER_RecID 0x123  // CAN1,功率计
/*********************************CAN发送ID*******************************************/
#define STEER_GM6020_TransID 0x1FE // CAN1,4个6020一起发
#define WHEEL_M3508_TransID 0x200  // CAN1,4个3508一起发
#define CAP_TransID 0x140          // CAN1 超电
#define POWER_METER_TransID 0x124  // CAN1,功率计

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
extern uint32_t real_power; //功率计数据

void Can_Filter_Init(void);
void Can_Msg_Init(void);
void Create_Can_Send_Queues(void);
void Allocate_Can_Msg(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id);
void CAN_cmd_power_meter(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

#endif
