/**
  * @file    motor_mixer.h
  * @brief   Quad-X motor mixer. Combines PID axis corrections with the RC
  *          throttle into 4 DSHOT motor values, gated by an arm switch (aux1).
  */
#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdint.h>
#include "flight_control.h"
#include "drivers/sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorMixer_Update(const FlightControlOutput_t *fc, const SBUS_Frame_t *rc, uint16_t motor_out[4]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */
