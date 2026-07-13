/**
  * @file    pid.h
  * @brief   Generic PID controller: anti-windup, derivative-on-measurement,
  *          D-term filtering, setpoint feedforward, and a per-call gain_scale
  *          (used by TPA to attenuate P/D at high throttle without needing
  *          separate "scaled copy of gains" bookkeeping).
  */
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include "filter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;
  float kff;

  float integrator;
  float integrator_min;
  float integrator_max;

  float prev_measurement;
  bool  has_prev;

  float prev_setpoint;
  bool  has_prev_setpoint;

  LPF1_t d_filter;

  float output_min;
  float output_max;
} PID_t;

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt, float gain_scale);
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
