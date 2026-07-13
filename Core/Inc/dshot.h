/**
  * @file    dshot.h
  * @brief   DSHOT300 motor output on TIM2 CH1-4 (PA0-PA3), one DMA stream per
  *          channel. Hand-configured (no CubeMX timer exists in the .ioc) -
  *          same self-contained pattern as scheduler.c (TIM4).
  */
#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 0 = motor-stop command. Valid throttle range is 48-2047. */
#define DSHOT_CMD_STOP   0U
#define DSHOT_THROTTLE_MIN 48U
#define DSHOT_THROTTLE_MAX 2047U

void DShot_Init(void);
void DShot_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H */
