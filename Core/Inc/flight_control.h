/**
  * @file    flight_control.h
  * @brief   Cascaded angle->rate PID control (roll/pitch) + rate-only yaw.
  *          No motor output yet - produces per-axis correction values only.
  */
#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "attitude.h"
#include "drivers/icm20602.h"
#include "drivers/sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float roll;
  float pitch;
  float yaw;
} FlightControlOutput_t;

void FlightControl_Init(void);
void FlightControl_Update(const ICM20602_Data_t *imu, const SBUS_Frame_t *rc, float dt,
                           Attitude_t *attitude_out, FlightControlOutput_t *out);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_CONTROL_H */
