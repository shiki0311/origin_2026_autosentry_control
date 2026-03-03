/*******************************************************************************
 * @file       Cboard_To_Nuc_usbd_communication.c
 * @brief      向上位机NUC发送陀螺仪，裁判系统数据。解包上位机发下来的自瞄和导航数据
 * @note
 * @history
 *  Version    Date                 Author          Modification
 *  V1.0usbd   October-28-2024		captainwu		1.transform to usbd
 *  V2.0tim    2025-7               Shiki           2.从在freertos task发送数据改为使用定时器中断发送数据
 *****************************************************************************************/
#include "Cboard_To_Nuc_usbd_communication.h"
#include "referee.h"
#include "tim.h"
#include "usb_device.h"

/****************************************通讯协议相关定义********************************************/
// 帧头id
#define PROTOCOL_HEAD_ID 0xAA

// 命令字ID
#define CMD_ID_NUC_DATA_RX 0x81 // 接收自瞄数据（上>下）
#define CMD_ID_AUTOAIM_DATA_TX 0x14 // 发送自瞄数据（下>上）
#define CMD_ID_REFEREE_DATA_TX 0x18 // 发送裁判系统（下>上）

// 数据段长度，以字节为单位
#define LENGTH_AUTOAIM_DATA_TX 12 // 自瞄所需的陀螺仪的数据长度（3个float)
#define LENGTH_REFEREE_DATA_TX 50 // 决策所需的裁判系统数据长度
#define PROTOCAL_HEAD_LENGTH 3	  // 协议头长度
#define CRC_TAIL_LENGTH 1		  // 一帧数据末尾的CRC校验码长度

/*******************************************END**********************************************/

uint8_t NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT], NUC_USBD_AutoAim_TxBuf[USBD_TX_BUF_LENGHT], NUC_USBD_Referee_TxBuf[USBD_TX_BUF_LENGHT];
uint8_t yaw_rotate_flag_last = 0;

AutoAim_Data_Tx AutoAim_Data_Transmit;
NUC_Data_Rx NUC_Data_Receive;
Referee_Data_Tx Referee_Data_Tramsit;
// 发送数据状态跟踪
bool_t is_sending_referee = FALSE; // 标记是否正在发送referee数据
bool_t autoaim_pending = FALSE;	   // 标记是否有等待发送的autoaim数据
bool_t referee_pending = FALSE;	   // 标记是否有等待发送的referee数据
// 8位版本CRC表
static const uint8_t CRC08_Table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/**************************接收并解包NUC发来的数据********************************/
void NUC_Data_Unpack(void)
{
	switch (NUC_USBD_RxBuf[2])
	{
	case CMD_ID_NUC_DATA_RX:
	{
		static fp32 auto_aim_yaw_last;

		yaw_rotate_flag_last = NUC_Data_Receive.yaw_rotate_flag;
		memcpy(&NUC_Data_Receive, NUC_USBD_RxBuf + 3, sizeof(NUC_Data_Rx));
			break;
	}

	default:
		return;
	}
}

void USBD_IRQHandler(uint8_t *Buf, uint16_t Len)
{
	memcpy(NUC_USBD_RxBuf, Buf, Len);
	if (NUC_USBD_RxBuf[0] != PROTOCOL_HEAD_ID || NUC_USBD_RxBuf[1] > 128) // 帧头不匹配或者数据包长度错误，丢弃这一帧
		return;

	if (NUC_USBD_RxBuf[1] == Len) // 校验数据包长度
		NUC_Data_Unpack();		  // 解包NUC数据
}

/**************************打包数据并向nuc发送******************************* */

void NUC_USBD_Tx(uint8_t cmdid)
{
	Protocol_Head_Data Protocol_Head;
	Protocol_Head.Header = PROTOCOL_HEAD_ID;
	Protocol_Head.Cmd_ID = cmdid;
	switch (cmdid)
	{
	case CMD_ID_AUTOAIM_DATA_TX:
		Protocol_Head.Length = LENGTH_AUTOAIM_DATA_TX + PROTOCAL_HEAD_LENGTH + CRC_TAIL_LENGTH;
		memcpy(NUC_USBD_AutoAim_TxBuf, (uint8_t *)(&Protocol_Head), PROTOCAL_HEAD_LENGTH);
		AutoAim_Data_Transmit.Pitch = INS.Pitch;
		AutoAim_Data_Transmit.Roll = INS.Roll;
		AutoAim_Data_Transmit.Yaw = INS.Yaw;

		memcpy(NUC_USBD_AutoAim_TxBuf + PROTOCAL_HEAD_LENGTH, (uint8_t *)(&AutoAim_Data_Transmit), LENGTH_AUTOAIM_DATA_TX);

		NUC_USBD_AutoAim_TxBuf[LENGTH_AUTOAIM_DATA_TX + PROTOCAL_HEAD_LENGTH] = CRC_Calculation(NUC_USBD_AutoAim_TxBuf, LENGTH_AUTOAIM_DATA_TX + PROTOCAL_HEAD_LENGTH);
		if (is_sending_referee)
		{
			autoaim_pending = TRUE;
		}
		else
		{
			if (CDC_Transmit_FS(NUC_USBD_AutoAim_TxBuf, Protocol_Head.Length) != USBD_OK)
				autoaim_pending = TRUE;
		}
		break;

	case CMD_ID_REFEREE_DATA_TX:
		Protocol_Head.Length = LENGTH_REFEREE_DATA_TX + PROTOCAL_HEAD_LENGTH + CRC_TAIL_LENGTH;
		memcpy(NUC_USBD_Referee_TxBuf, (uint8_t *)(&Protocol_Head), PROTOCAL_HEAD_LENGTH);

		Referee_Data_Tramsit.remain_HP = Game_Robot_State.current_HP;
		Referee_Data_Tramsit.max_HP = Game_Robot_State.maximum_HP;
		Referee_Data_Tramsit.game_progress = Game_Status.game_progress;
		Referee_Data_Tramsit.stage_remain_time = Game_Status.stage_remain_time;
		Referee_Data_Tramsit.coin_remaining_num = Bullet_Remaining.coin_remaining_num;
		Referee_Data_Tramsit.bullet_remaining_num_17mm = Bullet_Remaining.bullet_remaining_num_17mm;

//		Referee_Data_Tramsit.red_1_HP = Game_Robot_HP.red_1_robot_HP;
//		Referee_Data_Tramsit.red_2_HP = Game_Robot_HP.red_2_robot_HP;
//		Referee_Data_Tramsit.red_3_HP = Game_Robot_HP.red_3_robot_HP;
//		Referee_Data_Tramsit.red_4_HP = Game_Robot_HP.red_4_robot_HP;
//		Referee_Data_Tramsit.red_7_HP = Game_Robot_HP.red_7_robot_HP;
//		Referee_Data_Tramsit.red_outpost_HP = Game_Robot_HP.red_outpost_HP;
//		Referee_Data_Tramsit.red_base_HP = Game_Robot_HP.red_base_HP;

//		Referee_Data_Tramsit.blue_1_HP = Game_Robot_HP.blue_1_robot_HP;
//		Referee_Data_Tramsit.blue_2_HP = Game_Robot_HP.blue_2_robot_HP;
//		Referee_Data_Tramsit.blue_3_HP = Game_Robot_HP.blue_3_robot_HP;
//		Referee_Data_Tramsit.blue_4_HP = Game_Robot_HP.blue_4_robot_HP;
//		Referee_Data_Tramsit.blue_7_HP = Game_Robot_HP.blue_7_robot_HP;
//		Referee_Data_Tramsit.blue_outpost_HP = Game_Robot_HP.blue_outpost_HP;
//		Referee_Data_Tramsit.blue_base_HP = Game_Robot_HP.blue_base_HP;

		Referee_Data_Tramsit.rfid_status = RFID_Status.rfid_status;
		Referee_Data_Tramsit.event_data = Event_Data.event_type;
		Referee_Data_Tramsit.hurt_reason = Robot_Hurt.hurt_type;
		Referee_Data_Tramsit.enemy_hero_position = Student_Interactive_Data.enemy_hero_position_data;
		Referee_Data_Tramsit.defend_fortress = Student_Interactive_Data.check_defend_fortress;
		memcpy(NUC_USBD_Referee_TxBuf + PROTOCAL_HEAD_LENGTH, (uint8_t *)(&Referee_Data_Tramsit), LENGTH_REFEREE_DATA_TX);

		NUC_USBD_Referee_TxBuf[LENGTH_REFEREE_DATA_TX + PROTOCAL_HEAD_LENGTH] = CRC_Calculation(NUC_USBD_Referee_TxBuf, LENGTH_REFEREE_DATA_TX + PROTOCAL_HEAD_LENGTH);

		if (CDC_Transmit_FS(NUC_USBD_Referee_TxBuf, Protocol_Head.Length) != USBD_OK)
		{
			referee_pending = TRUE;
		}
		else
		{
			is_sending_referee = TRUE;
		}

		break;
	default:
		return;
	}
}

void Send_To_NUC(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		static uint8_t cnt = 0;

		if (cnt == 9)
		{
			NUC_USBD_Tx(CMD_ID_REFEREE_DATA_TX); // 50ms trigger
			cnt = 0;
		}
		else
			cnt++;

		NUC_USBD_Tx(CMD_ID_AUTOAIM_DATA_TX); // 5ms trigger
	}
	else
		return;
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len)
{
	uint8_t crc = 0xff;
	while (len--)
	{
		crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}

/**************************处理发送完成后的状态更新和积压数据发送******************************* */
/**
 * @brief  处理发送完成后的状态更新和积压数据发送
 * @param  Buf: 发送完成的缓冲区地址（用于判断是referee还是autoaim）
 * @retval None
 */
void Process_Transmit_Complete(uint8_t *Buf)
{
	// 若完成的是referee数据发送
	if (Buf == NUC_USBD_Referee_TxBuf)
	{
		is_sending_referee = 0; // 清除referee发送状态

		// 若有积压的autoaim数据，立即发送
		if (autoaim_pending)
		{
			// 发送缓存的autoaim数据
			CDC_Transmit_FS(NUC_USBD_AutoAim_TxBuf, LENGTH_AUTOAIM_DATA_TX + PROTOCAL_HEAD_LENGTH + CRC_TAIL_LENGTH);
			autoaim_pending = 0; // 清除积压标记
		}
	}
	// 若完成的是autoaim数据发送
	else if (Buf == NUC_USBD_AutoAim_TxBuf)
	{
		// 若有积压的referee数据，立即发送
		if (referee_pending)
		{
			CDC_Transmit_FS(NUC_USBD_Referee_TxBuf, LENGTH_REFEREE_DATA_TX + PROTOCAL_HEAD_LENGTH + CRC_TAIL_LENGTH);
			referee_pending = 0; // 清除积压标记
		}
		else if (autoaim_pending)
		{
			// 发送缓存的autoaim数据
			CDC_Transmit_FS(NUC_USBD_AutoAim_TxBuf, LENGTH_AUTOAIM_DATA_TX + PROTOCAL_HEAD_LENGTH + CRC_TAIL_LENGTH);
			autoaim_pending = 0; // 清除积压标记
		}
	}
}