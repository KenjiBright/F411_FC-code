/**
  * @file    flight_control.c
  * @brief   Cascaded angle->rate PID control (roll/pitch) + rate-only yaw.
  *          Gyro/D-term low-pass filtering, feedforward, and TPA (throttle
  *          PID attenuation). No motor output yet - produces per-axis
  *          correction values only.
  */
#include "flight_control.h"
#include "pid.h"
#include "filter.h"

/* SBUS raw channel range: ~172-1811, center ~992 */
#define RC_MID              992.0f
#define RC_HALF_RANGE       819.5f

#define MAX_ANGLE_DEG       30.0f
#define MAX_YAW_RATE_DPS    180.0f

#define ANGLE_RATE_LIMIT_DPS 250.0f
#define AXIS_OUTPUT_LIMIT    500.0f

/* Nominal loop period used to initialize filter time constants (Phase A.5: 1kHz timer tick) */
#define CONTROL_DT           0.001f

#define GYRO_LPF_CUTOFF_HZ   90.0f
#define DTERM_LPF_CUTOFF_HZ  70.0f

#define TPA_BREAKPOINT       0.5f
#define TPA_FLOOR            0.5f

static Attitude_t attitude;

static PID_t roll_angle_pid;
static PID_t pitch_angle_pid;
static PID_t roll_rate_pid;
static PID_t pitch_rate_pid;
static PID_t yaw_rate_pid;

static LPF1_t gyro_lpf_x;
static LPF1_t gyro_lpf_y;
static LPF1_t gyro_lpf_z;

static float RC_ToNormalized(uint16_t raw)
{
  return ((float)raw - RC_MID) / RC_HALF_RANGE;
}

static float TPA_GainScale(uint16_t throttle_raw)
{
  float throttle = RC_ToNormalized(throttle_raw) * 0.5f + 0.5f; /* -1..1 -> 0..1 */

  if (throttle <= TPA_BREAKPOINT)
  {
    return 1.0f;
  }

  float span = (throttle - TPA_BREAKPOINT) / (1.0f - TPA_BREAKPOINT);
  float scale = 1.0f - (span * (1.0f - TPA_FLOOR));

  return (scale < TPA_FLOOR) ? TPA_FLOOR : scale;
}

void FlightControl_Init(void)
{
  Attitude_Init();

  /* Placeholder starting gains - NOT tuned for any real frame, retune from blackbox data */
  roll_angle_pid  = (PID_t){ .kp = 4.5f, .ki = 0.0f, .kd = 0.0f, .kff = 0.0f,
                              .integrator_min = -ANGLE_RATE_LIMIT_DPS, .integrator_max = ANGLE_RATE_LIMIT_DPS,
                              .output_min = -ANGLE_RATE_LIMIT_DPS, .output_max = ANGLE_RATE_LIMIT_DPS };
  pitch_angle_pid = (PID_t){ .kp = 4.5f, .ki = 0.0f, .kd = 0.0f, .kff = 0.0f,
                              .integrator_min = -ANGLE_RATE_LIMIT_DPS, .integrator_max = ANGLE_RATE_LIMIT_DPS,
                              .output_min = -ANGLE_RATE_LIMIT_DPS, .output_max = ANGLE_RATE_LIMIT_DPS };

  roll_rate_pid   = (PID_t){ .kp = 0.6f, .ki = 0.3f, .kd = 0.01f, .kff = 0.02f,
                              .integrator_min = -AXIS_OUTPUT_LIMIT, .integrator_max = AXIS_OUTPUT_LIMIT,
                              .output_min = -AXIS_OUTPUT_LIMIT, .output_max = AXIS_OUTPUT_LIMIT };
  pitch_rate_pid  = (PID_t){ .kp = 0.6f, .ki = 0.3f, .kd = 0.01f, .kff = 0.02f,
                              .integrator_min = -AXIS_OUTPUT_LIMIT, .integrator_max = AXIS_OUTPUT_LIMIT,
                              .output_min = -AXIS_OUTPUT_LIMIT, .output_max = AXIS_OUTPUT_LIMIT };
  yaw_rate_pid    = (PID_t){ .kp = 0.8f, .ki = 0.3f, .kd = 0.0f, .kff = 0.02f,
                              .integrator_min = -AXIS_OUTPUT_LIMIT, .integrator_max = AXIS_OUTPUT_LIMIT,
                              .output_min = -AXIS_OUTPUT_LIMIT, .output_max = AXIS_OUTPUT_LIMIT };

  LPF1_Init(&roll_rate_pid.d_filter, DTERM_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&pitch_rate_pid.d_filter, DTERM_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&yaw_rate_pid.d_filter, DTERM_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&roll_angle_pid.d_filter, DTERM_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&pitch_angle_pid.d_filter, DTERM_LPF_CUTOFF_HZ, CONTROL_DT);

  LPF1_Init(&gyro_lpf_x, GYRO_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&gyro_lpf_y, GYRO_LPF_CUTOFF_HZ, CONTROL_DT);
  LPF1_Init(&gyro_lpf_z, GYRO_LPF_CUTOFF_HZ, CONTROL_DT);
}

void FlightControl_Update(const ICM20602_Data_t *imu, const SBUS_Frame_t *rc, float dt,
                           Attitude_t *attitude_out, FlightControlOutput_t *out)
{
  ICM20602_Data_t filtered_imu = *imu;
  float roll_setpoint_deg;
  float pitch_setpoint_deg;
  float yaw_rate_setpoint;
  float roll_rate_setpoint;
  float pitch_rate_setpoint;
  float gain_scale = TPA_GainScale(rc->channels[2]);

  filtered_imu.gx = LPF1_Update(&gyro_lpf_x, imu->gx);
  filtered_imu.gy = LPF1_Update(&gyro_lpf_y, imu->gy);
  filtered_imu.gz = LPF1_Update(&gyro_lpf_z, imu->gz);

  roll_setpoint_deg  = RC_ToNormalized(rc->channels[0]) * MAX_ANGLE_DEG;
  pitch_setpoint_deg = RC_ToNormalized(rc->channels[1]) * MAX_ANGLE_DEG;
  yaw_rate_setpoint  = RC_ToNormalized(rc->channels[3]) * MAX_YAW_RATE_DPS;

  Attitude_Update(&filtered_imu, dt, &attitude);

  roll_rate_setpoint  = PID_Update(&roll_angle_pid, roll_setpoint_deg, attitude.roll, dt, 1.0f);
  pitch_rate_setpoint = PID_Update(&pitch_angle_pid, pitch_setpoint_deg, attitude.pitch, dt, 1.0f);

  out->roll  = PID_Update(&roll_rate_pid, roll_rate_setpoint, filtered_imu.gx, dt, gain_scale);
  out->pitch = PID_Update(&pitch_rate_pid, pitch_rate_setpoint, filtered_imu.gy, dt, gain_scale);
  out->yaw   = PID_Update(&yaw_rate_pid, yaw_rate_setpoint, filtered_imu.gz, dt, gain_scale);

  *attitude_out = attitude;
}
