/**
  * @file    pid.c
  * @brief   Generic PID controller: anti-windup, derivative-on-measurement,
  *          D-term filtering, setpoint feedforward, per-call gain_scale (TPA).
  */
#include "pid.h"

static float clampf(float value, float min, float max)
{
  if (value < min) { return min; }
  if (value > max) { return max; }
  return value;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt, float gain_scale)
{
  float error = setpoint - measurement;
  float raw_derivative;
  float filtered_derivative;
  float feedforward;
  float output;

  pid->integrator += pid->ki * error * dt;
  pid->integrator = clampf(pid->integrator, pid->integrator_min, pid->integrator_max);

  if (pid->has_prev)
  {
    raw_derivative = -(measurement - pid->prev_measurement) / dt;
  }
  else
  {
    raw_derivative = 0.0f;
  }

  pid->prev_measurement = measurement;
  pid->has_prev = true;

  filtered_derivative = LPF1_Update(&pid->d_filter, raw_derivative);

  if (pid->has_prev_setpoint)
  {
    feedforward = pid->kff * (setpoint - pid->prev_setpoint) / dt;
  }
  else
  {
    feedforward = 0.0f;
  }

  pid->prev_setpoint = setpoint;
  pid->has_prev_setpoint = true;

  output = (gain_scale * pid->kp * error) + pid->integrator
         + (gain_scale * pid->kd * filtered_derivative) + feedforward;

  return clampf(output, pid->output_min, pid->output_max);
}

void PID_Reset(PID_t *pid)
{
  pid->integrator = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->has_prev = false;
  pid->prev_setpoint = 0.0f;
  pid->has_prev_setpoint = false;
}
