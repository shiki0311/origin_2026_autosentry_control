/*****************************************************************************************************************************
 * @file: bsp_can_gimbal.h
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2025赛季CAN通讯支持包，此文件适配云台C板
 *****************************************************************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "pid.h"

/*********************************CAN接收ID*******************************************/
#define BIG_YAW_DM6006_RecID 0x300 // CAN2
#define BIG_YAW_DMIMU_RecID 0x16   // CAN2, master id
#define DIAL_RecID 0x141             // CAN2

#define SMALL_YAW_GM6020_RecID 0x205 // CAN1
#define PITCH_GM6020_RecID 0x206     // CAN1
#define FRIC1_M3508_RecID 0x201      // CAN1
#define FRIC2_M3508_RecID 0x202      // CAN1
/*********************************CAN发送ID*******************************************/
#define GIMBAL_TO_CHASSIS_FIRST_ID 0x10  // CAN2,向下板发送遥控器数据
#define GIMBAL_TO_CHASSIS_SECOND_ID 0x11 // CAN2,向下板发送导航xy轴目标速度，导航的目标底盘模式，哨兵健康状态，是否正在上坡等标志位
#define GIMBAL_TO_CHASSIS_THIRD_ID 0x101  // CAN2,发什么待定
#define BIG_YAW_DM6006_TransID 0x01       // CAN2,DM6006
#define BIG_YAW_DMIMU_TransID 0x15        // CAN2,CAN id
#define DIAL_TransID 0x141                // CAN2,拨盘电机

#define SMALL_YAW_AND_PITCH_TransID 0x1FE // CAN1,两个6020一起发
#define FRIC_M3508_TransID 0x200          // CAN1,两个3508一起发
/***********************************************************************************/
#define CAN_TX_DIV2 2
#define CAN_TX_DIV3 3
#define CAN_TX_DIV4 4
#define CAN_TX_DIV5 5

#define LK_MOTOR_TORQUE_CONTROL_CMD_ID 0xA1 // 领控拨弹盘电机命令ID

typedef enum
{
    CAN_GIMBAL_TO_CHASSIS_FIRST_CMD,
    CAN_GIMBAL_TO_CHASSIS_SECOND_CMD,
    CAN_GIMBAL_TO_CHASSIS_THIRD_CMD,
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
