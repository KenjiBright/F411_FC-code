/**
  * @file    sbus.c
  * @brief   SBUS RC receiver frame decoder (USART1, 100000 8E2 inverted)
  */
#include "drivers/sbus.h"

#define SBUS_FRAME_LEN      25U
#define SBUS_START_BYTE     0x0FU

#define SBUS_FLAG_CH17      0x01U
#define SBUS_FLAG_CH18      0x02U
#define SBUS_FLAG_FRAME_LOST 0x04U
#define SBUS_FLAG_FAILSAFE  0x08U

static UART_HandleTypeDef *sbus_huart;

static uint8_t  rx_byte;
static uint8_t  frame_buf[SBUS_FRAME_LEN];
static uint8_t  frame_idx;

static volatile SBUS_Frame_t latest_frame;
static volatile bool         frame_ready;

static void SBUS_DecodeFrame(const uint8_t *data)
{
  SBUS_Frame_t frame;
  const uint8_t *p = &data[1]; /* 22 payload bytes start at index 1 */
  uint8_t flags = data[23];

  frame.channels[0]  = (uint16_t)((p[0]       | (p[1]  << 8))                    & 0x07FFU);
  frame.channels[1]  = (uint16_t)(((p[1] >> 3) | (p[2]  << 5))                    & 0x07FFU);
  frame.channels[2]  = (uint16_t)(((p[2] >> 6) | (p[3]  << 2) | (p[4]  << 10))    & 0x07FFU);
  frame.channels[3]  = (uint16_t)(((p[4] >> 1) | (p[5]  << 7))                    & 0x07FFU);
  frame.channels[4]  = (uint16_t)(((p[5] >> 4) | (p[6]  << 4))                    & 0x07FFU);
  frame.channels[5]  = (uint16_t)(((p[6] >> 7) | (p[7]  << 1) | (p[8]  << 9))     & 0x07FFU);
  frame.channels[6]  = (uint16_t)(((p[8] >> 2) | (p[9]  << 6))                    & 0x07FFU);
  frame.channels[7]  = (uint16_t)(((p[9] >> 5) | (p[10] << 3))                    & 0x07FFU);
  frame.channels[8]  = (uint16_t)((p[11]       | (p[12] << 8))                    & 0x07FFU);
  frame.channels[9]  = (uint16_t)(((p[12] >> 3)| (p[13] << 5))                    & 0x07FFU);
  frame.channels[10] = (uint16_t)(((p[13] >> 6)| (p[14] << 2) | (p[15] << 10))    & 0x07FFU);
  frame.channels[11] = (uint16_t)(((p[15] >> 1)| (p[16] << 7))                    & 0x07FFU);
  frame.channels[12] = (uint16_t)(((p[16] >> 4)| (p[17] << 4))                    & 0x07FFU);
  frame.channels[13] = (uint16_t)(((p[17] >> 7)| (p[18] << 1) | (p[19] << 9))     & 0x07FFU);
  frame.channels[14] = (uint16_t)(((p[19] >> 2)| (p[20] << 6))                    & 0x07FFU);
  frame.channels[15] = (uint16_t)(((p[20] >> 5)| (p[21] << 3))                    & 0x07FFU);

  frame.ch17       = (flags & SBUS_FLAG_CH17) != 0U;
  frame.ch18       = (flags & SBUS_FLAG_CH18) != 0U;
  frame.frame_lost = (flags & SBUS_FLAG_FRAME_LOST) != 0U;
  frame.failsafe   = (flags & SBUS_FLAG_FAILSAFE) != 0U;

  latest_frame = frame;
  frame_ready = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (frame_idx == 0U)
    {
      if (rx_byte == SBUS_START_BYTE)
      {
        frame_buf[frame_idx++] = rx_byte;
      }
    }
    else
    {
      frame_buf[frame_idx++] = rx_byte;

      if (frame_idx == SBUS_FRAME_LEN)
      {
        SBUS_DecodeFrame(frame_buf);
        frame_idx = 0U;
      }
    }

    HAL_UART_Receive_IT(sbus_huart, &rx_byte, 1);
  }
}

void SBUS_Init(UART_HandleTypeDef *huart)
{
  sbus_huart = huart;
  frame_idx = 0U;
  frame_ready = false;

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_UART_Receive_IT(sbus_huart, &rx_byte, 1);
}

bool SBUS_GetFrame(SBUS_Frame_t *frame)
{
  if (!frame_ready)
  {
    return false;
  }

  *frame = latest_frame;
  frame_ready = false;

  return true;
}
