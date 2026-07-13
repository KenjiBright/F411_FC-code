/**
  * @file    filter.h
  * @brief   Reusable first-order low-pass filter
  */
#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float alpha;
  float state;
  bool  has_state;
} LPF1_t;

void  LPF1_Init(LPF1_t *f, float cutoff_hz, float dt);
float LPF1_Update(LPF1_t *f, float input);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
