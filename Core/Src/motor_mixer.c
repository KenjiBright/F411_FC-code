/**
  * @file    motor_mixer.c
  * @brief   Quad-X motor mixer with arm-switch and throttle-stop gating.
  *          Arm/disarm gated on SBUS ch5 (aux1). Failsafe-on-signal-loss is
  *          deferred to a later phase (SBUS driver exposes frame_lost/failsafe).
  */
#include "motor_mixer.h"
#include "dshot.h"

#define ARM_CHANNEL_THRESHOLD     1500U  /* raw SBUS ch5 (aux1) */
#define THROTTLE_STOP_THRESHOLD    300U  /* raw SBUS ch3, near ~172 floor */
#define RC_THROTTLE_MIN             172U
#define RC_THROTTLE_MAX            1811U
#define MIXER_SCALE                 1.0f /* conservative start, tune later */

static uint16_t ClampU16(int32_t v, uint16_t lo, uint16_t hi)
{
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint16_t)v;
}

void MotorMixer_Update(const FlightControlOutput_t *fc, const SBUS_Frame_t *rc, uint16_t motor_out[4])
{
  if (rc->channels[4] <= ARM_CHANNEL_THRESHOLD)
  {
    motor_out[0] = motor_out[1] = motor_out[2] = motor_out[3] = 0;
    return;
  }

  if (rc->channels[2] <= THROTTLE_STOP_THRESHOLD)
  {
    motor_out[0] = motor_out[1] = motor_out[2] = motor_out[3] = 0;
    return;
  }

  float throttle_norm = ((float)rc->channels[2] - RC_THROTTLE_MIN) / (RC_THROTTLE_MAX - RC_THROTTLE_MIN);
  if (throttle_norm < 0.0f) throttle_norm = 0.0f;
  if (throttle_norm > 1.0f) throttle_norm = 1.0f;
  float throttle = DSHOT_THROTTLE_MIN + throttle_norm * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN);

  float roll  = fc->roll  * MIXER_SCALE;
  float pitch = fc->pitch * MIXER_SCALE;
  float yaw   = fc->yaw   * MIXER_SCALE;

  /* Quad-X mix. SIGN CONVENTION NOT YET VERIFIED ON HARDWARE - confirm motor
   * position + CW/CCW spin direction once motors are mounted, before props. */
  float m1 = throttle - roll + pitch - yaw; /* front-right */
  float m2 = throttle - roll - pitch + yaw; /* rear-right  */
  float m3 = throttle + roll - pitch - yaw; /* rear-left   */
  float m4 = throttle + roll + pitch + yaw; /* front-left  */

  motor_out[0] = ClampU16((int32_t)m1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  motor_out[1] = ClampU16((int32_t)m2, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  motor_out[2] = ClampU16((int32_t)m3, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  motor_out[3] = ClampU16((int32_t)m4, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
}
