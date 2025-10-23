/*******************************************************************************
 * @file       Cboard_To_Nuc_usbd_communication.h
 * @brief      向上位机NUC发送陀螺仪，裁判系统数据。解包上位机发下来的自瞄和导航数据
 * @note
 * @history
 *  Version    Date                 Author          Modification
 *  V1.0usbd   October-28-2024		captainwu		1.transform to usbd
 *  V2.0tim    2025-7               Shiki           2.从使用freertos的task发送数据改为使用定时器中断发送数据
 *****************************************************************************************/

#ifndef NMANIFOLD_USBD_TASK_H
#define NMANIFOLD_USBD_TASK_H

#include <stdint.h>

#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"

#include "INS_task.h"
#include "arm_math.h"

#include "usbd_cdc_if.h"

/****************************************常量定义段********************************************/
#define USBD_RX_BUF_LENGHT 256
#define USBD_TX_BUF_LENGHT 128

// 命令字ID
#define CMD_ID_AUTOAIM_DATA_RX 0x81 // 接收自瞄数据（上>下）
#define CMD_ID_AUTOAIM_DATA_TX 0x14 // 发送自瞄数据（下>上）
#define CMD_ID_REFEREE_DATA_TX 0x18 // 发送裁判系统（下>上）

// 数据段长度，以字节为单位
#define LENGTH_AUTOAIM_DATA_TX 12 // 自瞄所需的陀螺仪的数据长度（3个float)
#define LENGTH_REFEREE_DATA_TX 50 // 决策所需的裁判系统数据长度
#define PROTOCAL_HEAD_LENGTH 3  // 协议头长度
#define CRC_TAIL_LENGTH 1      // 一帧数据末尾的CRC校验码长度

/*******************************************END**********************************************/

#pragma pack(push, 1)

/*************************************向上位机nuc发送的结构体****************************************/
typedef struct
{
	uint8_t Header;								  // 帧头
	uint8_t Length;								  // 帧长（包含帧头与校验位）
	uint8_t Cmd_ID;								  // 命令字
} __attribute__((__packed__)) Protocol_Head_Data; // 通信协议数据流前段数据

typedef struct
{
	float Yaw;	 // 当前yaw（°）
	float Pitch; // 当前pitch（°）
	float Roll;	 // 当前Roll（°）
} __attribute__((__packed__)) AutoAim_Data_Tx;

typedef struct
{
	uint16_t remain_HP;
	uint16_t max_HP;

	uint8_t game_progress;
	uint16_t stage_remain_time;
	uint16_t coin_remaining_num;
	uint16_t bullet_remaining_num_17mm;

	uint16_t red_1_HP;
	uint16_t red_2_HP;
	uint16_t red_3_HP;
	uint16_t red_4_HP;
	uint16_t red_7_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;

	uint16_t blue_1_HP;
	uint16_t blue_2_HP;
	uint16_t blue_3_HP;
	uint16_t blue_4_HP;
	uint16_t blue_7_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;

	uint32_t rfid_status;
	uint32_t event_data;
	uint8_t hurt_reason;
	uint8_t enemy_hero_position; // 敌方英雄所在区域的编号
	bool_t defend_fortress;		 // 是否要回防入侵我方堡垒的敌人
} __attribute__((__packed__)) Referee_Data_Tx;
/*******************************************END**********************************************/

/*************************************nuc发送下来的数据结构体******************************************/
typedef struct
{
	float yaw_aim;
	float pitch_aim;
	bool_t fire_or_not;
	bool_t track;
	float vx;
	float vy;

	float rotate;
	float yaw_speed;
	float pitch_speed;
	bool_t uphill_flag;		 // 是否正在上坡
	bool_t yaw_rotate_flag;	 // 是否要进行大回环
	bool_t ready_catch_hero; // 是否到达抓英雄的点位
} __attribute__((__packed__)) AutoAim_Data_Rx;

#pragma pack(pop)
/*******************************************END**********************************************/

extern uint8_t NUC_USBD_AutoAim_TxBuf[USBD_TX_BUF_LENGHT],NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT];
extern AutoAim_Data_Rx AutoAim_Data_Receive;
extern uint8_t yaw_rotate_flag_last; // 记录上一次的yaw_rotate_flag，用于判断当前是不是刚刚退出大回环模式

void Send_To_NUC(TIM_HandleTypeDef *htim);
void USBD_IRQHandler(uint8_t *Buf, uint16_t Len);
void Process_Transmit_Complete(uint8_t *Buf);
uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len);

#endif
