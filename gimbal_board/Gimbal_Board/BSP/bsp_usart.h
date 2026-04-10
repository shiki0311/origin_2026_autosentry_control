#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

#define Referee_UART huart6

void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num); //暂未使用。usart1目前只用来发vofa,直接调用hal_transmit_dma即可，该函数已经包括dma和中断使能
void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void Referee_USART6_Init(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength); //usart6用作裁判系统串口
void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
#endif
