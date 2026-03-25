/**
 ******************************************************************************
 * @file    refereetask.h
 * @author  Shiki
 * @version V2.0.0
 * @date    2025/09/16
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "bsp_usart.h"
/* Private define ------------------------------------------------------------*/
#define Referee_FIFOInit fifo_s_init
#define Max(a, b) ((a) > (b) ? (a) : (b))

/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

void referee_usart_task(void const *argument)
{
	while (1)
	{
		/* 解析裁判系统数据 */
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		vTaskDelay(4);
		/* 向裁判系统发送哨兵自主决策数据 */
		Sentry_PushUp_Cmd(&Sentry_Auto_Cmd_Send, Game_Robot_State.robot_id);
		vTaskDelay(8);
//		Sentry_To_Lidar_Cmd(&Sentry_Interactive_With_Liadr, Game_Robot_State.robot_id);
		vTaskDelay(8);
	}
}

void Referee_IRQHandler(void)
{
	if (Referee_UART.Instance->SR & UART_FLAG_RXNE)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
	}
	else if (Referee_UART.Instance->SR & UART_FLAG_IDLE)
	{
		static uint16_t Size = 0;

		/* 清空标志位 */
		__HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);

		if ((Referee_UART.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
			//			Size =  Referee_UART.hdmarx->Instance->NDTR;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);

			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[0], Size);
		}
		else
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			//			Size = Referee_UART.hdmarx->Instance->NDTR;
			Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);

			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[1], Size);
		}
		return;
	}

	HAL_UART_IRQHandler(&Referee_UART);
}
