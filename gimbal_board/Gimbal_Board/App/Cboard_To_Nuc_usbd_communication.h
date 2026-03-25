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

#if USE_EKF == 1
#include "INS_Task_ekf.h"
#else
#include "INS_Task_Mahony.h"
#endif

#include "arm_math.h"

#include "usbd_cdc_if.h"

#define USBD_RX_BUF_LENGHT 256
#define USBD_TX_BUF_LENGHT 128
/*************************************向上位机nuc发送的结构体****************************************/
typedef struct
{
	uint8_t Header;								  // 帧头
	uint8_t Length;								  // 帧长（包含帧头与校验位）
	uint8_t Cmd_ID;								  // 命令字
} __attribute__((__packed__)) Protocol_Head_Data; // 通信协议数据流前段数据

typedef struct
{
	float Pitch; // 当前pitch（°）
	float Roll;	 // 当前Roll（°）
	float Small_Yaw;	 // 当前小yaw（°）
	float Big_Yaw;    //当前大yaw（°）

	uint8_t mode;//自瞄用不到，发0就行

	// 四元数
	float w; 
	float x;
	float y;
	float z;

	float small_yaw_speed;
	float pitch_speed;

	float bullet_speed;
	uint8_t bullet_shoot_num; //没用，发0就行
} __attribute__((__packed__)) AutoAim_Data_Tx;

typedef struct
{
	//自身信息
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_cooling_speed;  //热量冷却速率
	uint16_t shooter_heat_limit; //枪口热量上限
	uint16_t shooter_heat_now; //当前枪口热量
	uint8_t remain_energy;  //剩余底盘能量
	uint8_t health_state;		 // 健康状态
	uint8_t state_now; //当前姿态
	//全局信息
	uint16_t stage_remain_time;
	uint8_t game_progress;
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
	uint32_t rfid_status;
	uint32_t event_data;
	bool_t defend_fortress;		 // 是否要回防入侵我方堡垒的敌人
} __attribute__((__packed__)) Referee_Data_Tx;
/*******************************************END**********************************************/

/*************************************nuc发送下来的数据结构体******************************************/
typedef struct
{
	//模式切换相关数据
	uint8_t chassis_mode;  //1跟头2陀螺
	uint8_t pass_bumpy_mode;
	uint8_t pitch_mode; 	//打人0打前哨1
	uint8_t target_mode;  //哨兵目标姿态
	uint8_t occupy_middle_section; //rmul开局抢中
	//自瞄数据
	float small_yaw_aim;
	float big_yaw_aim;
	float pitch_aim;
	bool_t fire_or_not;
	//导航数据
	float vx;
	float vy;
	float pass_bumpy_yaw_angle;
	bool_t updownhill_state;		 // 是否正在上坡
	bool_t ready_catch_hero; // 是否到达抓英雄的点位
} __attribute__((__packed__)) NUC_Data_Rx;

/*******************************************END**********************************************/
extern uint8_t NUC_USBD_AutoAim_TxBuf[USBD_TX_BUF_LENGHT],NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT];
extern NUC_Data_Rx NUC_Data_Receive;
extern Referee_Data_Tx Referee_Data_Transmit;

void Send_To_NUC(TIM_HandleTypeDef *htim);
void USBD_IRQHandler(uint8_t *Buf, uint16_t Len);
void Process_Transmit_Complete(uint8_t *Buf);
uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len);

#endif
