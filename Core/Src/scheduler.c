/**
  * @file    scheduler.c
  * @brief   TIM4-driven 1kHz control-loop tick (self-contained, hand-configured
  *          since no timer is set up in the CubeMX .ioc). TIM2 is deliberately
  *          avoided here since it's earmarked for motor PWM (PA0-3) later.
  */
#include "scheduler.h"

/* APB1 timer clock = 96MHz (APB1 prescaler != 1, so timer clock = 2x APB1).
 * PSC=95 -> 1MHz counter clock. ARR=999 -> 1kHz update event. */
#define TIM4_PRESCALER  95U
#define TIM4_PERIOD     999U

volatile bool g_control_tick_flag = false;

TIM_HandleTypeDef htim4;

void Scheduler_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM4_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = TIM4_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim4);

  HAL_NVIC_SetPriority(TIM4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  HAL_TIM_Base_Start_IT(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    g_control_tick_flag = true;
  }
}
