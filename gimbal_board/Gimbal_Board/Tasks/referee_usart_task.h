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
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREETASK_H__
#define __REFEREETASK_H__
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "protocol.h"

#define REFEREE_USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH     1024

/* 裁判系统串口双缓冲区 */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

void Referee_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __REFEREETASK_H__ */
