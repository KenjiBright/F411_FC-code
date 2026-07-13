/**
  * @file    filter.c
  * @brief   Reusable first-order low-pass filter
  */
#include "filter.h"
#include <math.h>

#define TWO_PI_F  6.283185307179586f

void LPF1_Init(LPF1_t *f, float cutoff_hz, float dt)
{
  float rc = 1.0f / (TWO_PI_F * cutoff_hz);

  f->alpha = dt / (rc + dt);
  f->state = 0.0f;
  f->has_state = false;
}

float LPF1_Update(LPF1_t *f, float input)
{
  if (!f->has_state)
  {
    f->state = input;
    f->has_state = true;
    return f->state;
  }

  f->state += f->alpha * (input - f->state);

  return f->state;
}
