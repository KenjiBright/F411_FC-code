/**
  * @file    attitude.c
  * @brief   Complementary-filter roll/pitch attitude estimator
  */
#include "attitude.h"
#include <math.h>

#define ATTITUDE_GYRO_WEIGHT   0.98f
#define ATTITUDE_ACCEL_WEIGHT  (1.0f - ATTITUDE_GYRO_WEIGHT)
#define RAD_TO_DEG_F           57.29577951308232f

static float est_roll;
static float est_pitch;

void Attitude_Init(void)
{
  est_roll = 0.0f;
  est_pitch = 0.0f;
}

void Attitude_Update(const ICM20602_Data_t *imu, float dt, Attitude_t *out)
{
  float accel_roll  = atan2f(imu->ay, imu->az) * RAD_TO_DEG_F;
  float accel_pitch = atan2f(-imu->ax, sqrtf((imu->ay * imu->ay) + (imu->az * imu->az))) * RAD_TO_DEG_F;

  float gyro_roll  = est_roll + (imu->gx * dt);
  float gyro_pitch = est_pitch + (imu->gy * dt);

  est_roll  = (ATTITUDE_GYRO_WEIGHT * gyro_roll)  + (ATTITUDE_ACCEL_WEIGHT * accel_roll);
  est_pitch = (ATTITUDE_GYRO_WEIGHT * gyro_pitch) + (ATTITUDE_ACCEL_WEIGHT * accel_pitch);

  out->roll = est_roll;
  out->pitch = est_pitch;
}
