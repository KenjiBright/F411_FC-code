# Session Handoff — Phase D (DSHOT300 motor output + mixer + arm switch)

Last updated: 2026-07-09. Read this first in the next session, then see the full
approved plan at `/home/kenji/.claude/plans/atomic-cuddling-frost.md` (has all
phases, including finished A/A.5 and not-yet-started B/C).

## Where things stand

Phase D code is **fully written and builds clean** (2026-07-09,
`make clean && make -j4`, zero warnings/errors, 57KB flash). All firmware is on
disk. What remains is **hardware bench verification with props OFF** (see "Safety
verification" below) — nothing has been flashed or verified on hardware yet.

### Done (all written + compiled)
- `Core/Inc/dshot.h` — public API: `DShot_Init(void)`,
  `DShot_Write(uint16_t m1, m2, m3, m4)`, `DSHOT_CMD_STOP=0`,
  `DSHOT_THROTTLE_MIN=48`, `DSHOT_THROTTLE_MAX=2047`.
- `Core/Src/dshot.c` — TIM2 CH1-4 (PA0-PA3), 1 DMA1 stream/channel, DSHOT300
  encode. Design transcribed as-is from below.
- `Core/Inc/motor_mixer.h` / `Core/Src/motor_mixer.c` — Quad-X mix + arm-switch
  (ch5) + throttle-stop gating.
- `Core/Src/main.c` — includes, `static uint16_t motor_out[4];`, `DShot_Init();`
  after `FlightControl_Init()`, `MotorMixer_Update()` + `DShot_Write()` in the
  control-tick block, `m1..m4` added to JSON telemetry (buffer bumped 224→288).
- `Makefile` — `Core/Src/dshot.c` + `Core/Src/motor_mixer.c` added to C_SOURCES.

### Remaining (hardware, needs the user + a bench)
1. Flash props-OFF and walk the "Safety verification" steps below. In
   particular the DMA stream mapping (RM0383 Table 28) and the Quad-X sign
   convention are correct on paper but **unconfirmed on hardware** — the bench
   test is where you catch a backwards sign or a bad DMA stream. Comments in
   `dshot.c` (DShot_DmaInit) and `motor_mixer.c` (the mix block) flag exactly
   what to double-check.
2. Optional: CubeIDE headless build if that's still the project's
   dual-verification habit (Makefile build already passes).

## Why hand-written, not CubeMX

Grepped the `.ioc`: no timer peripheral configured at all, and PA0-PA3 are not
claimed by any CubeMX peripheral. Also, `MX_USART1_UART_Init()` in `main.c` was
already hand-edited outside USER CODE markers for SBUS (100000 baud, 8E2) —
regenerating via the CubeMX GUI would silently wipe that. So TIM2 + PA0-PA3 are
configured by hand in `dshot.c`, same self-contained pattern already used for
TIM4 in `scheduler.c`.

## Design: `Core/Src/dshot.c`

DSHOT300 bit timing: APB1 timer clock 96MHz ÷ 300kHz = 320 ticks/bit. TIM2 is a
32-bit timer on STM32F4 so `Period=319` fits natively (`Prescaler=0`). Duty
encodes the bit: logical `1` = 75% high (~240/320 ticks), logical `0` = 37.5%
high (~120/320 ticks). 16-bit DSHOT frame = 11-bit throttle + 1 telemetry bit +
4-bit CRC, sent MSB-first, followed by a few zero-padding entries for the
inter-frame gap.

```c
#include "dshot.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define MOTOR_COUNT       4U
#define DSHOT_BIT_PERIOD  320U
#define DSHOT_BIT1_DUTY   240U
#define DSHOT_BIT0_DUTY   120U
#define DSHOT_FRAME_BITS  16U
#define DSHOT_BUF_LEN     18U   /* 16 bits + 2 zero-padding for inter-frame gap */

static TIM_HandleTypeDef htim2;
static DMA_HandleTypeDef hdma_tim2_ch1;
static DMA_HandleTypeDef hdma_tim2_ch2;
static DMA_HandleTypeDef hdma_tim2_ch3;
static DMA_HandleTypeDef hdma_tim2_ch4;

static uint32_t dshot_buf[MOTOR_COUNT][DSHOT_BUF_LEN];

static const uint32_t kMotorChannel[MOTOR_COUNT] = {
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static void DShot_GpioInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
}

static void DShot_TimerInit(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = DSHOT_BIT_PERIOD - 1U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim2);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);
}

/* DMA1 request mapping for TIM2 (RM0383 Table 27) - VERIFY against the
 * datasheet before flashing. Deliberately picked one distinct stream per
 * channel so all 4 channels can transfer independently every control tick. */
static void DShot_DmaInit(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  static struct { DMA_HandleTypeDef *h; DMA_Stream_TypeDef *stream; uint32_t tim_dma_id; } cfg[MOTOR_COUNT] = {
    { &hdma_tim2_ch1, DMA1_Stream5, TIM_DMA_ID_CC1 },
    { &hdma_tim2_ch2, DMA1_Stream6, TIM_DMA_ID_CC2 },
    { &hdma_tim2_ch3, DMA1_Stream1, TIM_DMA_ID_CC3 },
    { &hdma_tim2_ch4, DMA1_Stream7, TIM_DMA_ID_CC4 },
  };

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    DMA_HandleTypeDef *h = cfg[i].h;
    h->Instance = cfg[i].stream;
    h->Init.Channel = DMA_CHANNEL_3;
    h->Init.Direction = DMA_MEMORY_TO_PERIPH;
    h->Init.PeriphInc = DMA_PINC_DISABLE;
    h->Init.MemInc = DMA_MINC_ENABLE;
    h->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    h->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    h->Init.Mode = DMA_NORMAL;
    h->Init.Priority = DMA_PRIORITY_HIGH;
    h->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(h);
    __HAL_LINKDMA(&htim2, hdma[cfg[i].tim_dma_id], *h);
  }
}

static void DShot_EncodeFrame(uint32_t *buf, uint16_t throttle)
{
  if (throttle > DSHOT_THROTTLE_MAX) throttle = DSHOT_THROTTLE_MAX;

  uint16_t value = (uint16_t)((throttle << 1) | 0U); /* telemetry bit = 0 */
  uint16_t crc = (uint16_t)((value ^ (value >> 4) ^ (value >> 8)) & 0x0FU);
  uint16_t packet = (uint16_t)((value << 4) | crc);

  for (int i = 0; i < DSHOT_FRAME_BITS; i++)
  {
    uint16_t bit = (packet >> (DSHOT_FRAME_BITS - 1U - i)) & 0x1U;
    buf[i] = bit ? DSHOT_BIT1_DUTY : DSHOT_BIT0_DUTY;
  }
  buf[16] = 0;
  buf[17] = 0;
}

void DShot_Init(void)
{
  DShot_GpioInit();
  DShot_TimerInit();
  DShot_DmaInit();
  memset(dshot_buf, 0, sizeof(dshot_buf));
}

void DShot_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
  uint16_t throttle[MOTOR_COUNT] = { m1, m2, m3, m4 };

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    DShot_EncodeFrame(dshot_buf[i], throttle[i]);
    HAL_TIM_PWM_Stop_DMA(&htim2, kMotorChannel[i]);
    HAL_TIM_PWM_Start_DMA(&htim2, kMotorChannel[i], dshot_buf[i], DSHOT_BUF_LEN);
  }
}
```

Why no NVIC/DMA-IRQ handling: a DSHOT300 frame takes ~18 bits x 3.33us ~= 60us
to transmit, far shorter than the 1kHz (1ms) control tick. `DShot_Write` is
called once per tick and does `Stop_DMA` + `Start_DMA` (one-shot, non-circular)
each time, which naturally supersedes the previous transfer — no completion
interrupt needed.

## Design: `motor_mixer.h` / `motor_mixer.c`

Arm/disarm gated on SBUS ch5 (`rc_frame.channels[4]`, user's choice — aux1).
Hard failsafe-on-signal-loss explicitly deferred by the user to a later phase
(SBUS driver already exposes `frame_lost`/`failsafe` flags in `SBUS_Frame_t`
for whenever that phase happens).

```c
// motor_mixer.h
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
#endif
```

```c
// motor_mixer.c
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
```

## Relevant existing types (for reference, no changes needed)

- `SBUS_Frame_t` (`Core/Inc/drivers/sbus.h`): `channels[16]`, `ch17`, `ch18`,
  `frame_lost`, `failsafe`.
- `FlightControlOutput_t` (`Core/Inc/flight_control.h`): `{ float roll, pitch, yaw; }`,
  range ±500 (`AXIS_OUTPUT_LIMIT` in `flight_control.c`).
- RC channel mapping used throughout: `channels[0]`=roll, `[1]`=pitch,
  `[2]`=throttle, `[3]`=yaw, `[4]`=arm switch (aux1). Raw SBUS range ~172-1811,
  center ~992 (`RC_MID`/`RC_HALF_RANGE` in `flight_control.c`).

## Safety verification (props OFF for all of this — tell the user explicitly)

1. Clean build (Makefile + CubeIDE headless if that's still the habit).
2. Props off. Flash. With arm switch (ch5) low, confirm `m1..m4` in JSON
   telemetry stay 0 regardless of stick movement.
3. Arm switch high, throttle stick at minimum: `m1..m4` should stay 0
   (throttle-stop threshold).
4. Raise throttle slowly: `m1..m4` should rise together roughly in sync;
   motors (props off) should spin smoothly, no erratic revving. Erratic
   behavior means bad DMA/bit-timing — stop and recheck `Period`/duty
   constants and the DMA stream mapping (verify against RM0383 Table 27)
   before continuing.
5. Throttle held steady, move roll/pitch/yaw sticks one at a time: confirm
   each produces the expected differential change across `m1..m4` matching
   the mix table above — this is the point to flip signs if the convention is
   backwards, before ever mounting props.
6. Only mount props after all of the above matches expectations on the bench.

## Not started (later phases, do not start without user direction)

- Phase B: blackbox MVP logger (W25Q128-backed flight log).
- Phase C: I-term relax + anti-gravity.
- RPM-based notch filtering (needs bidirectional DSHOT telemetry — Phase D is
  FC→ESC only).
- Dynamic notch filtering (FFT/CMSIS-DSP) — blocked on carving CMSIS-DSP out of
  the currently-excluded `Drivers/CMSIS/` folder in the CubeIDE `.cproject`.
