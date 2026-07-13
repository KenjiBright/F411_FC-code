/**
  * @file    dshot.c
  * @brief   DSHOT300 motor output on TIM2 CH1-4 (PA0-PA3), one DMA1 stream per
  *          channel. Hand-configured (no CubeMX timer exists in the .ioc) -
  *          same self-contained pattern as scheduler.c (TIM4).
  *
  * DSHOT300 bit timing: APB1 timer clock 96MHz / 300kHz = 320 ticks/bit. TIM2
  * is a 32-bit timer so Period=319 fits natively (Prescaler=0). Duty encodes the
  * bit: logical 1 = 75% high (240/320), logical 0 = 37.5% high (120/320). The
  * 16-bit DSHOT frame = 11-bit throttle + 1 telemetry bit + 4-bit CRC, sent
  * MSB-first, followed by zero-padding entries for the inter-frame gap.
  */
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

/* DMA1 request mapping for TIM2 (RM0383 Table 28, DMA1, channel 3):
 *   Stream5 -> TIM2_CH1, Stream6 -> TIM2_CH2, Stream1 -> TIM2_CH3,
 *   Stream7 -> TIM2_CH4. One distinct stream per channel so all 4 channels
 *   transfer independently every control tick. */
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
