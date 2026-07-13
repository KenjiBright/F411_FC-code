/**
  * @file    attitude.h
  * @brief   Complementary-filter roll/pitch attitude estimator
  */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "drivers/icm20602.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float roll;   /* deg */
  float pitch;  /* deg */
} Attitude_t;

void Attitude_Init(void);
void Attitude_Update(const ICM20602_Data_t *imu, float dt, Attitude_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
