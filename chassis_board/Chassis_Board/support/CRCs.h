/**
  ******************************************************************************
  * @file    crcs.h
  * @author  Karolance Future
  * @version V1.0.0
  * @date    2022/03/02
  * @brief   Header file of crcs.c
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRCS_H__
#define __CRCS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


 uint8_t CRC08_Calculate(uint8_t  *Data, uint32_t Lenth /* Without check code lenth 1 */);
uint32_t CRC08_Verify   (uint8_t  *Data, uint32_t Lenth /* With    check code lenth 1 */);
uint32_t CRC08_Append   (uint8_t  *Data, uint32_t Lenth /* With    check code lenth 1 */);

uint16_t CRC16_Calculate(uint8_t  *Data, uint32_t Lenth /* Without check code lenth 2 */);
uint32_t CRC16_Verify   (uint8_t  *Data, uint32_t Lenth /* With    check code lenth 2 */);
uint32_t CRC16_Append   (uint8_t  *Data, uint32_t Lenth /* With    check code lenth 2 */);

uint32_t CRC32_Calculate(uint32_t *Data, uint32_t Lenth /* Without check code lenth 1 */);
uint32_t CRC32_Verify   (uint32_t *Data, uint32_t Lenth /* With    check code lenth 1 */);
uint32_t CRC32_Append   (uint32_t *Data, uint32_t Lenth /* With    check code lenth 1 */);


#ifdef __cplusplus
}
#endif

#endif /* __CRCS_H__ */
