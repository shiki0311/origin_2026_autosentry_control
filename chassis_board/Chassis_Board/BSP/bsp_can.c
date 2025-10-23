/********************************************************************************************************************************************
 * @file: bsp_can.c
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2026赛季CAN总线支持包，此为底盘C板相关的CAN接收和发送代码
 * *******************************************************************************************************************************************
 * @attention: 1.哨兵can报文发送的流程是在各个task计算出需要向电机或其他设备（超电，另一块C板）需要发送的数据后，在xxx_task.c中调用
 *             Allocate_Can_Queue（）或者Ctrl_DM_Motor()。调用规则为如果是向达秒电机发送can报文，则调用Ctrl_DM_Motor()，其他直接
 *             调用Allocate_Can_Queue()。这两个函数的最终目的是将各个task要发送的can报文填充入对应的can发送队列（队列使用freertos实现)，
 * 			   最后can报文会在freertos的定时器任务中统一发送。
 *
 *             2.本文件为各个task提供接口函数和全局变量（注：定时器任务的创建和实现在Can_Send_Task.c/h)。
 **********************************************************************************************************************************************/
#include "bsp_can.h"
#include "bsp_cap.h"
#include "bsp_dwt.h"
#include "task.h"
#include "main.h"
#include "motor.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "string.h"
/************************达秒电机控制参数******************************/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -15.0f
#define T_MAX 15.0f
#define P_MIN -12.56637f
#define P_MAX 12.56637f
/*********************************CAN接收ID*******************************************/
#define BIG_YAW_DM6006_RecID 0x300		 // CAN2,下板需要接收大yaw的电机位置数据用于底盘跟随云台
#define RC_TO_CHASSIS_FIRST_RecID 0x102	 // CAN2
#define RC_TO_CHASSIS_SECOND_RecID 0x100 // CAN2
#define STEER1_GM6020_RecID 0x205		 // CAN2
#define STEER2_GM6020_RecID 0x206		 // CAN2
#define STEER3_GM6020_RecID 0x207		 // CAN2
#define STEER4_GM6020_RecID 0x208		 // CAN2

#define WHEEL1_M3508_RecID 0x201 // CAN1
#define WHEEL2_M3508_RecID 0x202 // CAN1
#define WHEEL3_M3508_RecID 0x203 // CAN1
#define WHEEL4_M3508_RecID 0x204 // CAN1
#define CAP_RecID 0x130			 // CAN1 超电
/*********************************CAN发送ID*******************************************/
#define STEER_GM6020_TransID 0x1FF // CAN2,4个6020一起发
#define WHEEL_M3508_TransID 0x200  // CAN1,4个3508一起发
#define CAP_TransID 0x140		   // CAN1 超电
/*******************************CAN发送时的外设实例映射********************************/
#define STEER_GM6020_TransCAN hcan2
#define WHEEL_M3508_TransCAN hcan1
#define CAP_TransCAN hcan1
/*******************************CAN发送相关宏定义********************************/
#define CAN_TX_TIM_DIV2 2
#define CAN_TX_TIM_DIV3 3
#define CAN_TX_TIM_DIV4 4
#define CAN_TX_TIM_DIV5 5
#define CAN_TX_QUEUE_LENGTH 64
/*******************************************************************************/
CAN_RxHeaderTypeDef rx_header; // debug用，看can接收正不正常
chassis_rc_ctrl_t chassis_rc_ctrl = {0};
//int32_t trans_freq = 0; // can发送定时器的中断回调函数每秒执行次数，配合dwt使用
//int i1,i2,i3;
/*********************************************CAN发送队列*********************************************************************/
QueueHandle_t CAN1_resend_queue; // CAN1消息队列句柄,此队列用于储存CAN1第一次发送失败的消息
QueueHandle_t CAN2_resend_queue; // CAN2消息队列句柄，此队列用于储存CAN2第一次发送失败的消息

#define STEER_GM6020_RESEND_QUEUE CAN2_resend_queue
#define WHEEL_M3508_RESEND_QUEUE CAN1_resend_queue
#define CAP_RESEND_QUEUE CAN1_resend_queue
/*********************************************CAN发送缓冲区*********************************************************************/
CanTxMsgTypeDef steer_send_buffer; // 传rc_ctrl.rc.ch数组的前四个元素的can报文全局缓冲区
CanTxMsgTypeDef wheel_send_buffer; // 传rc_ctrl.rc.ch数组的第五个元素和rc_ctrl.rc.s数组的can报文全局缓冲区
CanTxMsgTypeDef cap_send_buffer;   // 大yaw轴达妙6006 can报文全局缓冲区

/**
 * @description: 配置can过滤器，开启can外设以及需要的中断
 * @return 无
 */
void Can_Filter_Init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	// HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @description: CAN缓冲区初始化
 * @return {*} 无
 */
void Can_Buffer_Init(void)
{
	// 定义缓冲区与对应StdId的映射关系（数组批量处理）
	struct
	{
		CanTxMsgTypeDef *buffer;
		uint32_t stdId;
	} buffer_list[] = {
		{&steer_send_buffer, STEER_GM6020_TransID},
		{&wheel_send_buffer, WHEEL_M3508_TransID},
		{&cap_send_buffer, CAP_TransID}};

	// 遍历数组，批量初始化所有缓冲区
	for (size_t i = 0; i < sizeof(buffer_list) / sizeof(buffer_list[0]); i++)
	{
		buffer_list[i].buffer->tx_header.IDE = CAN_ID_STD;							 // 标准帧
		buffer_list[i].buffer->tx_header.RTR = CAN_RTR_DATA;						 // 数据帧
		buffer_list[i].buffer->tx_header.DLC = 0x08;								 // 数据长度8字节
		buffer_list[i].buffer->tx_header.StdId = buffer_list[i].stdId;				 // CAN Id
		memset(buffer_list[i].buffer->data, 0, sizeof(buffer_list[i].buffer->data)); // 数据缓冲区清零
	}
}

/**
 * @description: 基于freertos创建CAN发送队列
 * @return 无
 */
void Create_Can_Send_Queues()
{
	taskENTER_CRITICAL(); // 进入临界区

	CAN1_resend_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));
	CAN2_resend_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));

	taskEXIT_CRITICAL(); // 退出临界区

	if (CAN1_resend_queue == NULL || CAN2_resend_queue == NULL)

		Error_Handler();
}
/*********************************************CAN接收函数*********************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	static int flag = 0;
//	static float start_time = 0;

//	if (flag == 0)
//	{
//		start_time = DWT_GetTimeline_ms();
//		flag = 1;
//	}
//	if (flag == 1)
//	{
//		if (DWT_GetTimeline_ms() - start_time <= 1000)
//		{
//			trans_freq++;
//		}
//		else
//		{
//			flag = 0;
//			i1=i2=i3=0;
//			trans_freq = 0;
//		}
//	}
	uint8_t rx_data[8];
	if (hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
		case WHEEL1_M3508_RecID:
		case WHEEL2_M3508_RecID:
		case WHEEL3_M3508_RecID:
		case WHEEL4_M3508_RecID:
		{
			uint8_t i = rx_header.StdId - WHEEL1_M3508_RecID;
			get_motor_measure(&motor_measure_wheel[i], rx_data);
			// detect_hook(CHASSIS_MOTOR1_TOE + i);
			break;
		}

		case CAP_RecID:
		{
			update_cap(rx_data);
			break;
		}
		default:
		{
			break;
		}
		}
	}
	if (hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
		case STEER1_GM6020_RecID:
		case STEER2_GM6020_RecID:
		case STEER3_GM6020_RecID:
		case STEER4_GM6020_RecID:
		{
			uint8_t i = rx_header.StdId - STEER1_GM6020_RecID;
			get_motor_measure(&motor_measure_steer[i], rx_data);

			break;
		}
		case BIG_YAW_DM6006_RecID: // 下板只需要DM6006的位置数据完成底盘跟随云台
		{
//			i1++;
			DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
			DM_big_yaw_motor.state = (rx_data[0]) >> 4;
			DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
			break;
		}
		case RC_TO_CHASSIS_FIRST_RecID:
		{
			uint8_t rc_connected;
//			i2++;
			chassis_rc_ctrl.s[1] = rx_data[0];
			rc_connected = rx_data[1];
			chassis_rc_ctrl.ch[2] = (rx_data[2] << 8) | rx_data[3];
			chassis_rc_ctrl.ch[3] = (rx_data[4] << 8) | rx_data[5];
			chassis_rc_ctrl.ch[4] = (rx_data[6] << 8) | rx_data[7];

			if (rc_connected)
				detect_hook(RC_FIRST_TOE);

			break;
		}
		case RC_TO_CHASSIS_SECOND_RecID:
		{
//			i3++;//此can报文还未确定会发什么数据过来
			break;
		}
		default:
		{
			break;
		}
		}
	}
}

/*********************************************************填充CAN消息缓冲区************************************************************************/
/**
 * @description: 将待发送的can报文送入对应的缓冲区，在CAN_TX_TimerIRQHandler()函数中将缓冲区中的can报文统一定时发送
 * @return 无
 * @param {int16_t} data1/2/3/4  标准can数据帧的数据域，一共八个字节，这边用int16_t接收是考虑兼容大疆系列电机can数据发送协议，对于别的can报文需要做一些类型转换处理
 * @param {CAN_TX_ID} can_id can报文id
 * @attention 将DM电机can数据帧送入缓冲区需调用Ctrl_DM_Motor（）函数，原因是达秒电机的can数据帧传输的数据有五个，需要先在Ctrl_DM_Motor（）进行处理后再在其内部调用Allocate_Can_Buffer（）
 */
void Allocate_Can_Buffer(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id)
{
	CanTxMsgTypeDef *pTxMsg = NULL;						  // 声明缓冲区指针
	int16_t data_array[4] = {data1, data2, data3, data4}; // 数据打包成数组，方便循环处理

	// 根据命令ID，让指针指向对应的缓冲区
	switch (can_cmd_id)
	{
	case CAN_STEER_GM6020_CMD:
		pTxMsg = &steer_send_buffer;
		break;
	case CAN_WHEEL_M3508_CMD:
		pTxMsg = &wheel_send_buffer;
		break;
	case CAN_CAP_CMD:
		pTxMsg = &cap_send_buffer;
		break;
	default:
		return; // 无效命令，直接返回
	}

	// 统一处理数据填充（超电命令和其他命令分开处理）
	if (can_cmd_id == CAN_CAP_CMD)
	{
		// 处理CAP命令的特殊逻辑
		for (int i = 0; i < 4; i++)
		{
			uint16_t temp = data_array[i] * 100;
			pTxMsg->data[2 * i] = temp & 0xFF;			  // 低8位
			pTxMsg->data[2 * i + 1] = (temp >> 8) & 0xFF; // 高8位
		}
	}
	else
	{
		// 处理其他命令的通用逻辑（直接拆分int16_t）
		for (int i = 0; i < 4; i++)
		{
			pTxMsg->data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
			pTxMsg->data[2 * i + 1] = data_array[i] & 0xFF;	   // 低8位
		}
	}
}

/*********************************************定时发送CAN报文的中断回调函数***********************************************************/
/**
 * @description: tim5定时器计数溢出中断回调函数，用于以固定频率发送can报文，函数体每2ms执行一次
 * @return {*}
 */
void CAN_TX_TimerIRQHandler()
{
	static uint8_t div = 1; // 在函数整体执行频率（500hz）的基础上分频，使不同can报文适配相应的不同频率
	uint32_t send_mail_box;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 调度标志

	// 500hz 发送can报文
	if (HAL_CAN_AddTxMessage(&STEER_GM6020_TransCAN, &steer_send_buffer.tx_header, steer_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(STEER_GM6020_RESEND_QUEUE, &steer_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}
	if (HAL_CAN_AddTxMessage(&WHEEL_M3508_TransCAN, &wheel_send_buffer.tx_header, wheel_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(WHEEL_M3508_RESEND_QUEUE, &wheel_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}

	// // 250hz 发送can报文
	if (div % CAN_TX_TIM_DIV2 == 0)
	{
		if (HAL_CAN_AddTxMessage(&CAP_TransCAN, &cap_send_buffer.tx_header, cap_send_buffer.data, &send_mail_box) != HAL_OK)
		{
			xQueueSendFromISR(CAP_RESEND_QUEUE, &cap_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
		}
	}

	div == 120 ? div = 1 : div++; // div等于2,3,4,5的最小公倍数时重置

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 触发任务切换
	}
}
/*********************************************定时检测需要重新发送的CAN消息的中断回调函数***********************************************************/

/**
 * @description: tim3定时器计数溢出中断回调函数，用于检测CAN1和CAN2有无第一次发送失败的消息，如有尝试重发。每10ms执行一次
 * @return {*}
 */
void CAN_Resend_Timer_IRQHandler()
{
	CanTxMsgTypeDef resend_msg;
	uint32_t send_mail_box;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 调度标志

	// 尝试发送队列中的消息
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && uxQueueMessagesWaitingFromISR(CAN1_resend_queue) > 0) || (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0 && uxQueueMessagesWaitingFromISR(CAN2_resend_queue) > 0))
	{
		if ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && uxQueueMessagesWaitingFromISR(CAN1_resend_queue) > 0))
		{
			xQueueReceiveFromISR(CAN1_resend_queue, &resend_msg, &xHigherPriorityTaskWoken);
			HAL_CAN_AddTxMessage(&hcan1, &resend_msg.tx_header, resend_msg.data, &send_mail_box);
		}

		if ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0 && uxQueueMessagesWaitingFromISR(CAN2_resend_queue) > 0))
		{
			xQueueReceiveFromISR(CAN2_resend_queue, &resend_msg, &xHigherPriorityTaskWoken);
			HAL_CAN_AddTxMessage(&hcan2, &resend_msg.tx_header, resend_msg.data, &send_mail_box);
		}
	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 触发任务切换
	}
}
