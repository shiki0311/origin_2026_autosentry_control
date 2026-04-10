/**
 ******************************************************************************
 * @file    referee.c
 * @author  Shiki,Xujinming
 * @version V1.3.0
 * @date    2026/2/24
 ******************************************************************************
 * @attention
 *
 *   依据裁判系统 2026串口协议附录 V1.2.0
 *
 ******************************************************************************
 */

/* Private includes ----------------------------------------------------------*/
#include "referee.h"
#include "string.h"
#include "usart.h"
#include "crcs.h"
#include "bsp_usart.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "detect_task.h"
typedef enum
{
	GAME_STATE_CMD_ID = 0x0001,			 // 比赛状态数据
	GAME_RESULT_CMD_ID = 0x0002,		 // 比赛结果数据
	GAME_ROBOT_HP_CMD_ID = 0x0003,		 // 己方机器人血量数据
	FIELD_EVENTS_CMD_ID = 0x0101,		 // 场地事件数据
	REFEREE_WARNING_CMD_ID = 0x0104,	 // 裁判警告信息
	DART_REMAINING_TIME_CMD_ID = 0x0105, // 飞镖发射口倒计时

	ROBOT_STATE_CMD_ID = 0x0201,			  // 比赛机器人状态
	POWER_HEAT_DATA_CMD_ID = 0x0202,		  // 实时功率热量数据
	ROBOT_POS_CMD_ID = 0x0203,				  // 机器人位置
	BUFF_MUSK_CMD_ID = 0x0204,				  // 机器人增益
	ROBOT_HURT_CMD_ID = 0x0206,				  // 伤害状态
	SHOOT_DATA_CMD_ID = 0x0207,				  // 实时射击信息
	BULLET_REMAINING_CMD_ID = 0x0208,		  // 子弹剩余发射数
	ROBOT_RFID_STATE_CMD_ID = 0x0209,		  // 机器人RFID状态
	DART_CLIENT_CMD_CMD_ID = 0x020A,		  // 飞镖站状态
	ROBOT_POSITION_TO_SENTRY = 0x020B,		  // 己方地面机器人位置
	LADAR_MARK_DATA_CMD_ID = 0x020C,		  // 双方机器人特殊状态
	SENTRY_INFO_CMD_ID = 0x020D,			  // 哨兵机器人信息
	LADAR_INFO_CMD_ID = 0x020E,				  // 雷达相关信息
	STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301, // 机器人间通信
	ROBOT_COMMAND_CMD_ID = 0x0303,			  // 小地图下发信息标识
	CLIENT_MAP_COMMAND_CMD_ID = 0x0305,		  // 小地图接收雷达数据
	IDCustomData,
} referee_cmd_id_e;

/* protocol包头结构体 */
frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
ext_game_status_t Game_Status;
ext_game_result_t Game_Result;
ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
ext_event_data_t Event_Data;
ext_referee_warning_t Referee_Warning;
ext_dart_remaining_time_t Dart_Remaining_Time;

/* 0x020X */
ext_game_robot_state_t Game_Robot_State;
ext_power_heat_data_t Power_Heat_Data;
ext_game_robot_pos_t Game_Robot_Pos;
ext_buff_musk_t Buff_Musk;
ext_robot_hurt_t Robot_Hurt;
ext_shoot_data_t Shoot_Data;
ext_bullet_remaining_t Bullet_Remaining;
ext_rfid_status_t RFID_Status;
dart_client_cmd_t Dart_Client_Cmd;
ext_ground_robot_position_t Ground_Robot_Position;
radar_mark_data_t Radar_Mark_Data;
ext_sentry_info_t Sentry_Info;
radar_info_t Radar_Info;

/* 0x030X */
ext_student_interactive_data_t Student_Interactive_Data;
ext_robot_command_t Robot_Command;
ext_client_map_command_t Client_Map_Command;

/* 绘制UI专用结构体 */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;

/* 哨兵专用结构体 */
Sentry_Auto_Cmd_Send_t Sentry_Auto_Cmd_Send;
Sentry_Interactive_With_Liadr_t Sentry_Interactive_With_Liadr;
/* Functions -----------------------------------------------------------------*/
/*==============================================================================
			  ##### 裁判系统初始化函数 #####
  ==============================================================================
	[..]  该部分提供如下函数:
		  (+) 裁判系统结构体初始化函数 Referee_StructInit
			(+) 裁判系统串口初始化函数 Referee_UARTInit
*/
void Referee_StructInit(void)
{
	// 包头结构体初始化
	memset(&Referee_Receive_Header, 0, sizeof(Referee_Receive_Header));
	/* 0x000X */
	memset(&Game_Status, 0, sizeof(Game_Status));
	memset(&Game_Result, 0, sizeof(Game_Result));
	memset(&Game_Robot_HP, 0, sizeof(Game_Robot_HP));
	/* 0x010X */
	memset(&Event_Data, 0, sizeof(Event_Data));
	memset(&Referee_Warning, 0, sizeof(Referee_Warning));
	memset(&Dart_Remaining_Time, 0, sizeof(Dart_Remaining_Time));
	/* 0x020X */
	memset(&Game_Robot_State, 0, sizeof(Game_Robot_State));
	memset(&Power_Heat_Data, 0, sizeof(Power_Heat_Data));
	memset(&Game_Robot_Pos, 0, sizeof(Game_Robot_Pos));
	memset(&Buff_Musk, 0, sizeof(Buff_Musk));
	memset(&Robot_Hurt, 0, sizeof(Robot_Hurt));
	memset(&Shoot_Data, 0, sizeof(Shoot_Data));
	memset(&Bullet_Remaining, 0, sizeof(Bullet_Remaining));
	memset(&RFID_Status, 0, sizeof(RFID_Status));
	memset(&Ground_Robot_Position, 0, sizeof(Ground_Robot_Position));
	memset(&Sentry_Info, 0, sizeof(Sentry_Info));
	/* 0x030X */
	memset(&Student_Interactive_Data, 0, sizeof(Student_Interactive_Data));
	memset(&Robot_Command, 0, sizeof(Robot_Command));
	memset(&Client_Map_Command, 0, sizeof(Client_Map_Command));
}


/*==============================================================================
			  ##### 裁判系统数据解析函数 #####
  ==============================================================================
	[..]  该部分提供如下函数:
		  (+) 裁判系统队列数据解压函数 Referee_UnpackFifoData
	  (+) 裁判系统队列数据处理函数 Referee_SolveFifoData
*/
void Referee_UnpackFifoData(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
	uint8_t byte = 0;
	uint8_t sof = HEADER_SOF;

	while (fifo_s_used(referee_fifo))
	{
		byte = fifo_s_get(referee_fifo);
		switch (referee_unpack_obj->unpack_step)
		{
		case STEP_HEADER_SOF:
		{
			if (byte == sof)
			{
				referee_unpack_obj->unpack_step = STEP_LENGTH_LOW;
				referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			}
			else
			{
				referee_unpack_obj->index = 0;
			}
		}
		break;

		case STEP_LENGTH_LOW:
		{
			referee_unpack_obj->data_len = byte;
			referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			referee_unpack_obj->unpack_step = STEP_LENGTH_HIGH;
		}
		break;

		case STEP_LENGTH_HIGH:
		{
			referee_unpack_obj->data_len |= (byte << 8);
			referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			if (referee_unpack_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
			{
				referee_unpack_obj->unpack_step = STEP_FRAME_SEQ;
			}
			else
			{
				referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
				referee_unpack_obj->index = 0;
			}
		}
		break;

		case STEP_FRAME_SEQ:
		{
			referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			referee_unpack_obj->unpack_step = STEP_HEADER_CRC8;
		}
		break;

		case STEP_HEADER_CRC8:
		{
			referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			if (referee_unpack_obj->index == REF_PROTOCOL_HEADER_SIZE)
			{
				if (CRC08_Verify(referee_unpack_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
				{
					referee_unpack_obj->unpack_step = STEP_DATA_CRC16;
				}
				else
				{
					referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
					referee_unpack_obj->index = 0;
				}
			}
		}
		break;

		case STEP_DATA_CRC16:
		{
			if (referee_unpack_obj->index < (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
			{
				referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
			}
			if (referee_unpack_obj->index >= (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
			{
				referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
				referee_unpack_obj->index = 0;
				if (CRC16_Verify(referee_unpack_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
				{
					Referee_SolveFifoData(referee_unpack_obj->protocol_packet);
				}
			}
		}
		break;

		default:
		{
			referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
			referee_unpack_obj->index = 0;
		}
		break;
		}
	}
}

void Referee_SolveFifoData(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;

	memcpy(&Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	switch (cmd_id)
	{
	case GAME_STATE_CMD_ID:
		memcpy(&Game_Status, frame + index, sizeof(ext_game_status_t));
		break;
	case GAME_RESULT_CMD_ID:
		memcpy(&Game_Result, frame + index, sizeof(ext_game_result_t));
		break;
	case GAME_ROBOT_HP_CMD_ID:
		memcpy(&Game_Robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
		break;
	case FIELD_EVENTS_CMD_ID:
		memcpy(&Event_Data, frame + index, sizeof(ext_event_data_t));
		break;
	case REFEREE_WARNING_CMD_ID:
		memcpy(&Referee_Warning, frame + index, sizeof(ext_referee_warning_t));
		break;
	case DART_REMAINING_TIME_CMD_ID:
		memcpy(&Dart_Remaining_Time, frame + index, sizeof(ext_dart_remaining_time_t));
		break;

	case ROBOT_STATE_CMD_ID:
		memcpy(&Game_Robot_State, frame + index, sizeof(ext_game_robot_state_t));
		break;
	case POWER_HEAT_DATA_CMD_ID:
		memcpy(&Power_Heat_Data, frame + index, sizeof(ext_power_heat_data_t));
		break;
	case ROBOT_POS_CMD_ID:
		memcpy(&Game_Robot_Pos, frame + index, sizeof(ext_game_robot_pos_t));
		break;
	case BUFF_MUSK_CMD_ID:
		memcpy(&Buff_Musk, frame + index, sizeof(ext_buff_musk_t));
		break;
	case ROBOT_HURT_CMD_ID:
		memcpy(&Robot_Hurt, frame + index, sizeof(ext_robot_hurt_t));
		break;
	case SHOOT_DATA_CMD_ID:
		memcpy(&Shoot_Data, frame + index, sizeof(ext_shoot_data_t));
		break;
	case BULLET_REMAINING_CMD_ID:
		memcpy(&Bullet_Remaining, frame + index, sizeof(ext_bullet_remaining_t));
		break;
	case ROBOT_RFID_STATE_CMD_ID:
		memcpy(&RFID_Status, frame + index, sizeof(ext_rfid_status_t));
		break;
	case SENTRY_INFO_CMD_ID:
		memcpy(&Sentry_Info, frame + index, sizeof(ext_sentry_info_t));
		break;
	case STUDENT_INTERACTIVE_DATA_CMD_ID:
		memcpy(&Student_Interactive_Data, frame + index, sizeof(ext_student_interactive_data_t));
		break;
	case ROBOT_COMMAND_CMD_ID:
		memcpy(&Robot_Command, frame + index, sizeof(ext_robot_command_t));
		break;
	case CLIENT_MAP_COMMAND_CMD_ID:
		memcpy(&Client_Map_Command, frame + index, sizeof(ext_client_map_command_t));
		break;

	default:
		break;
	}
}
/*==============================================================================
			  ##### 哨兵自主决策发送函数 #####
  ==============================================================================*/
/**
 * @description: 向裁判系统请求复活
 * @return {*}
 * @param {Sentry_Auto_Cmd_Send_t} *Sentry_Auto_Cmd
 * @param {uint8_t} RobotID
 */
void Sentry_PushUp_Cmd(Sentry_Auto_Cmd_Send_t *Sentry_Auto_Cmd, uint8_t RobotID)
{
	/* 填充 frame_header */
	Sentry_Auto_Cmd->Referee_Transmit_Header.SOF = HEADER_SOF;
	Sentry_Auto_Cmd->Referee_Transmit_Header.data_length = 10;
	Sentry_Auto_Cmd->Referee_Transmit_Header.seq = Sentry_Auto_Cmd->Referee_Transmit_Header.seq + 1;
	Sentry_Auto_Cmd->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Sentry_Auto_Cmd->Referee_Transmit_Header), 4);

	/* 填充 cmd_id */
	Sentry_Auto_Cmd->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID; // cmd id命令码

	/* 填充 student_interactive_header */
	Sentry_Auto_Cmd->Interactive_Header.data_cmd_id = SENTRY_AUTO_SEND; // 子内容id
	Sentry_Auto_Cmd->Interactive_Header.sender_ID = RobotID;			// 发送者ID
	Sentry_Auto_Cmd->Interactive_Header.receiver_ID = Referee_Server;	// 接收者ID

	/* 填充 sentry_cmd */
	Sentry_Auto_Cmd->sentry_cmd.change_sentry_mode = toe_is_error(NUC_DATA_TOE)? ATTACK_MODE : NUC_Data_Receive.target_mode; // 设置哨兵为进攻模式,测试用
	Sentry_Auto_Cmd->sentry_cmd.ensure_revive = 1;  // 请求复活，一直发就行，不会有什么别的影响
	Sentry_Auto_Cmd->CRC16 = CRC16_Calculate((uint8_t *)Sentry_Auto_Cmd, sizeof(Sentry_Auto_Cmd_Send_t) - 2); // frame_tail CRC16校验

	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Sentry_Auto_Cmd, sizeof(Sentry_Auto_Cmd_Send_t));
}

/**
 * @description: 向雷达发送哨兵剩余发单量和是否到达抓英雄的目标点
 * @return {*}
 * @param {Sentry_Interactive_With_Liadr_t} *Sentry_Interactive_With_Lidar_Cmd
 * @param {uint8_t} RobotID
 */
void Sentry_To_Lidar_Cmd(Sentry_Interactive_With_Liadr_t *Sentry_Interactive_With_Lidar_Cmd, uint8_t RobotID)
{
	/* 填充 frame_header */
	Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header.SOF = HEADER_SOF;
	Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header.data_length = 9;
	Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header.seq = Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header.seq + 1;
	Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Sentry_Interactive_With_Lidar_Cmd->Referee_Transmit_Header), 4);

	/* 填充 cmd_id */
	Sentry_Interactive_With_Lidar_Cmd->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID; // cmd id命令码

	/* 填充 student_interactive_header */
	Sentry_Interactive_With_Lidar_Cmd->Interactive_Header.data_cmd_id = SEND_TO_LIDAR;																	// 子内容id
	Sentry_Interactive_With_Lidar_Cmd->Interactive_Header.sender_ID = RobotID;																			// 发送者ID
	Sentry_Interactive_With_Lidar_Cmd->Interactive_Header.receiver_ID = ((RobotID == Robot_ID_Blue_Sentry) ? Robot_ID_Blue_Radar : Robot_ID_Red_Radar); // 接收者ID

	/* 填充 sentry_cmd */
	Sentry_Interactive_With_Lidar_Cmd->sentry_interactive_data.bullet_remaining_num = Bullet_Remaining.bullet_remaining_num_17mm;
	Sentry_Interactive_With_Lidar_Cmd->sentry_interactive_data.reach_enemy_hero = NUC_Data_Receive.ready_catch_hero;
	//	Sentry_Interactive_With_Lidar_Cmd->sentry_interactive_data.bullet_remaining_num = 100;
	//	Sentry_Interactive_With_Lidar_Cmd->sentry_interactive_data.reach_enemy_hero = 1;
	Sentry_Interactive_With_Lidar_Cmd->CRC16 = CRC16_Calculate((uint8_t *)Sentry_Interactive_With_Lidar_Cmd, sizeof(Sentry_Interactive_With_Liadr_t) - 2); // frame_tail CRC16校验

	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Sentry_Interactive_With_Lidar_Cmd, sizeof(Sentry_Interactive_With_Liadr_t));
}
/*==============================================================================
			  ##### UI基本图形绘制函数 #####
  ==============================================================================
	[..]  该部分提供如下函数:
		  (+) 绘制直线 UI_Draw_Line
	  (+) 绘制矩形 UI_Draw_Rectangle
	  (+) 绘制整圆 UI_Draw_Circle
	  (+) 绘制椭圆 UI_Draw_Ellipse
	  (+) 绘制圆弧 UI_Draw_Arc
	  (+) 绘制小数 UI_Draw_Float
	  (+) 绘制整数 UI_Draw_Int
	  (+) 绘制字符 UI_Draw_String
*/
void UI_Draw_Line(graphic_data_struct_t *Graph, // UI图形数据结构体指针
				  char GraphName[3],			// 图形名 作为客户端的索引
				  uint8_t GraphOperate,			// UI图形操作 对应UI_Graph_XXX的4种操作
				  uint8_t Layer,				// UI图形图层 [0,9]
				  uint8_t Color,				// UI图形颜色 对应UI_Color_XXX的9种颜色
				  uint16_t Width,				// 线宽
				  uint16_t StartX,				// 起始坐标X
				  uint16_t StartY,				// 起始坐标Y
				  uint16_t EndX,				// 截止坐标X
				  uint16_t EndY)				// 截止坐标Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Line;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph, // UI图形数据结构体指针
					   char GraphName[3],			 // 图形名 作为客户端的索引
					   uint8_t GraphOperate,		 // UI图形操作 对应UI_Graph_XXX的4种操作
					   uint8_t Layer,				 // UI图形图层 [0,9]
					   uint8_t Color,				 // UI图形颜色 对应UI_Color_XXX的9种颜色
					   uint16_t Width,				 // 线宽
					   uint16_t StartX,				 // 起始坐标X
					   uint16_t StartY,				 // 起始坐标Y
					   uint16_t EndX,				 // 截止坐标X
					   uint16_t EndY)				 // 截止坐标Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Rectangle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->end_x = EndX;
	Graph->end_y = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph, // UI图形数据结构体指针
					char GraphName[3],			  // 图形名 作为客户端的索引
					uint8_t GraphOperate,		  // UI图形操作 对应UI_Graph_XXX的4种操作
					uint8_t Layer,				  // UI图形图层 [0,9]
					uint8_t Color,				  // UI图形颜色 对应UI_Color_XXX的9种颜色
					uint16_t Width,				  // 线宽
					uint16_t CenterX,			  // 圆心坐标X
					uint16_t CenterY,			  // 圆心坐标Y
					uint16_t Radius)			  // 半径
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Circle;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->radius = Radius;
}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph, // UI图形数据结构体指针
					 char GraphName[3],			   // 图形名 作为客户端的索引
					 uint8_t GraphOperate,		   // UI图形操作 对应UI_Graph_XXX的4种操作
					 uint8_t Layer,				   // UI图形图层 [0,9]
					 uint8_t Color,				   // UI图形颜色 对应UI_Color_XXX的9种颜色
					 uint16_t Width,			   // 线宽
					 uint16_t CenterX,			   // 圆心坐标X
					 uint16_t CenterY,			   // 圆心坐标Y
					 uint16_t XHalfAxis,		   // X半轴长
					 uint16_t YHalfAxis)		   // Y半轴长
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Ellipse;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph, // UI图形数据结构体指针
				 char GraphName[3],			   // 图形名 作为客户端的索引
				 uint8_t GraphOperate,		   // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,				   // UI图形图层 [0,9]
				 uint8_t Color,				   // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t StartAngle,		   // 起始角度 [0,360]
				 uint16_t EndAngle,			   // 截止角度 [0,360]
				 uint16_t Width,			   // 线宽
				 uint16_t CenterX,			   // 圆心坐标X
				 uint16_t CenterY,			   // 圆心坐标Y
				 uint16_t XHalfAxis,		   // X半轴长
				 uint16_t YHalfAxis)		   // Y半轴长
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Arc;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = StartAngle;
	Graph->end_angle = EndAngle;
	Graph->width = Width;
	Graph->start_x = CenterX;
	Graph->start_y = CenterY;
	Graph->end_x = XHalfAxis;
	Graph->end_y = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph, // UI图形数据结构体指针
				   char GraphName[3],			 // 图形名 作为客户端的索引
				   uint8_t GraphOperate,		 // UI图形操作 对应UI_Graph_XXX的4种操作
				   uint8_t Layer,				 // UI图形图层 [0,9]
				   uint8_t Color,				 // UI图形颜色 对应UI_Color_XXX的9种颜色
				   uint16_t NumberSize,			 // 字体大小
				   uint16_t Significant,		 // 有效位数
				   uint16_t Width,				 // 线宽
				   uint16_t StartX,				 // 起始坐标X
				   uint16_t StartY,				 // 起始坐标Y
				   float FloatData)				 // 数字内容
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Float;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->end_angle = Significant;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = (IntData & 0x001ffc00) >> 10;
	Graph->end_y = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data_struct_t *Graph, // UI图形数据结构体指针
				 char GraphName[3],			   // 图形名 作为客户端的索引
				 uint8_t GraphOperate,		   // UI图形操作 对应UI_Graph_XXX的4种操作
				 uint8_t Layer,				   // UI图形图层 [0,9]
				 uint8_t Color,				   // UI图形颜色 对应UI_Color_XXX的9种颜色
				 uint16_t NumberSize,		   // 字体大小
				 uint16_t Width,			   // 线宽
				 uint16_t StartX,			   // 起始坐标X
				 uint16_t StartY,			   // 起始坐标Y
				 int32_t IntData)			   // 数字内容
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye = GraphOperate;
	Graph->graphic_tpye = UI_Graph_Int;
	Graph->layer = Layer;
	Graph->color = Color;
	Graph->start_angle = NumberSize;
	Graph->width = Width;
	Graph->start_x = StartX;
	Graph->start_y = StartY;
	Graph->radius = (IntData & 0x000003ff) >> 0;
	Graph->end_x = (IntData & 0x001ffc00) >> 10;
	Graph->end_y = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_String(string_data_struct_t *String, // UI图形数据结构体指针
					char StringName[3],			  // 图形名 作为客户端的索引
					uint8_t StringOperate,		  // UI图形操作 对应UI_Graph_XXX的4种操作
					uint8_t Layer,				  // UI图形图层 [0,9]
					uint8_t Color,				  // UI图形颜色 对应UI_Color_XXX的9种颜色
					uint16_t CharSize,			  // 字体大小
					uint16_t StringLength,		  // 字符串长度
					uint16_t Width,				  // 线宽
					uint16_t StartX,			  // 起始坐标X
					uint16_t StartY,			  // 起始坐标Y
					char *StringData)			  // 字符串内容
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye = StringOperate;
	String->graphic_tpye = UI_Graph_String;
	String->layer = Layer;
	String->color = Color;
	String->start_angle = CharSize;
	String->end_angle = StringLength;
	String->width = Width;
	String->start_x = StartX;
	String->start_y = StartY;
	for (int i = 0; i < StringLength; i++)
		String->stringdata[i] = *StringData++;
}

/*==============================================================================
			  ##### UI完整图案推送函数 #####
  ==============================================================================
	[..]  该部分提供如下函数:
		  (+) 推送图案 UI_PushUp_Graphs
			(+) 推送字符 UI_PushUp_String
			(+) 删除图层 UI_PushUp_Delete
*/
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* 与Counter相一致的UI_Graphx结构体头指针 */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; // 假设只发一个基本图形

	/* 填充 frame_header */
	Graph->Referee_Transmit_Header.SOF = HEADER_SOF;
	if (Counter == 1)
		Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if (Counter == 2)
		Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if (Counter == 5)
		Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if (Counter == 7)
		Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);

	/* 填充 cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 填充 student_interactive_header */
	if (Counter == 1)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if (Counter == 2)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if (Counter == 5)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if (Counter == 7)
		Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID = RobotID;		   // 当前机器人ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; // 对应操作手ID

	/* 填充 frame_tail 即CRC16 */
	if (Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if (Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if (Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if (Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}

	/* 使用串口PushUp到裁判系统 */
	if (Counter == 1)
		HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if (Counter == 2)
		HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if (Counter == 5)
		HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if (Counter == 7)
		HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph7_t));
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* 填充 frame_header */
	String->Referee_Transmit_Header.SOF = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);

	/* 填充 cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 填充 student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID = RobotID;			// 当前机器人ID
	String->Interactive_Header.receiver_ID = RobotID + 256; // 对应操作手ID

	/* 填充 frame_tail 即CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);

	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* 填充 frame_header */
	Delete->Referee_Transmit_Header.SOF = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);

	/* 填充 cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;

	/* 填充 student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID = RobotID;			// 当前机器人ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; // 对应操作手ID

	/* 填充 frame_tail 即CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);

	/* 使用串口PushUp到裁判系统 */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Delete, sizeof(UI_Delete_t));
}
