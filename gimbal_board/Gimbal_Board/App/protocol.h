/**
  ******************************************************************************
  * @file    protocol.h
  * @author  Karolance Future
  * @version V1.3.0
  * @date    2025/03/1
  * @brief   依据裁判系统 串口协议附录 V1.7
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef __packed struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
