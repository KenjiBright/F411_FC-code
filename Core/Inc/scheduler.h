/**
  * @file    scheduler.h
  * @brief   TIM4-driven 1kHz control-loop tick (self-contained, hand-configured
  *          since no timer is set up in the CubeMX .ioc).
  */
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool g_control_tick_flag;

void Scheduler_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
