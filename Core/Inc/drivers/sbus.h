/**
  * @file    sbus.h
  * @brief   SBUS RC receiver frame decoder (USART1, 100000 8E2 inverted)
  */
#ifndef DRIVERS_SBUS_H
#define DRIVERS_SBUS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t channels[16];
  bool     ch17;
  bool     ch18;
  bool     frame_lost;
  bool     failsafe;
} SBUS_Frame_t;

void SBUS_Init(UART_HandleTypeDef *huart);
bool SBUS_GetFrame(SBUS_Frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SBUS_H */
