#include "encoder.h"

#define ENCODER_LEFT_A_PORT        GPIOA
#define ENCODER_LEFT_A_PIN         GPIO_PIN_6
#define ENCODER_LEFT_B_PORT        GPIOA
#define ENCODER_LEFT_B_PIN         GPIO_PIN_7

#define ENCODER_RIGHT_A_PORT       GPIOB
#define ENCODER_RIGHT_A_PIN        GPIO_PIN_12
#define ENCODER_RIGHT_B_PORT       GPIOB
#define ENCODER_RIGHT_B_PIN        GPIO_PIN_13

#define ENCODER_UPDATE_PERIOD_MS   100U
#define ENCODER_TIM3_FILTER        6U

volatile int32_t g_left_encoder_count = 0;
volatile int32_t g_right_encoder_count = 0;
volatile int32_t g_left_encoder_speed = 0;
volatile int32_t g_right_encoder_speed = 0;

static TIM_HandleTypeDef s_htim3;
static uint16_t s_last_left_timer_count = 0;
static int32_t s_last_right_count = 0;
static uint32_t s_last_update_tick = 0;

static void Encoder_LeftTIM3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  GPIO_InitStruct.Pin = ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  s_htim3.Instance = TIM3;
  s_htim3.Init.Prescaler = 0;
  s_htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim3.Init.Period = 0xFFFF;
  s_htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = ENCODER_TIM3_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = ENCODER_TIM3_FILTER;

  if (HAL_TIM_Encoder_Init(&s_htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&s_htim3, 0);
  s_last_left_timer_count = 0;

  if (HAL_TIM_Encoder_Start(&s_htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Encoder_RightGPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = ENCODER_RIGHT_A_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_RIGHT_A_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ENCODER_RIGHT_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_RIGHT_B_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Encoder_Init(void)
{
  Encoder_LeftTIM3_Init();
  Encoder_RightGPIO_Init();
  Encoder_Reset();
}

void Encoder_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - s_last_update_tick;
  uint16_t left_now;
  int16_t left_delta;
  int32_t right_now;
  int32_t right_delta;

  if (elapsed < ENCODER_UPDATE_PERIOD_MS)
  {
    return;
  }

  left_now = (uint16_t)__HAL_TIM_GET_COUNTER(&s_htim3);
  left_delta = (int16_t)(left_now - s_last_left_timer_count);
  s_last_left_timer_count = left_now;

  right_now = g_right_encoder_count;
  right_delta = right_now - s_last_right_count;
  s_last_right_count = right_now;

  g_left_encoder_count += left_delta;
  g_left_encoder_speed = ((int32_t)left_delta * 1000L) / (int32_t)elapsed;
  g_right_encoder_speed = (right_delta * 1000L) / (int32_t)elapsed;

  s_last_update_tick = now;
}

int32_t Encoder_GetLeftPulse(void)
{
  return g_left_encoder_count;
}

int32_t Encoder_GetRightPulse(void)
{
  return g_right_encoder_count;
}

int32_t Encoder_GetLeftSpeed(void)
{
  return g_left_encoder_speed;
}

int32_t Encoder_GetRightSpeed(void)
{
  return g_right_encoder_speed;
}

int32_t Encoder_GetVelocity_mmps(void)
{
  int32_t left_speed = g_left_encoder_speed;
  int32_t right_speed = g_right_encoder_speed;
  int32_t average_pulse_per_second;

  if (left_speed < 0)
  {
    left_speed = -left_speed;
  }
  if (right_speed < 0)
  {
    right_speed = -right_speed;
  }

  average_pulse_per_second = (left_speed + right_speed) / 2;

#if ENCODER_OUTPUT_PULSE_PER_REV <= 0
#error "ENCODER_OUTPUT_PULSE_PER_REV must be greater than 0"
#endif

  return (int32_t)(((int64_t)average_pulse_per_second * WHEEL_CIRCUMFERENCE_MM) /
                   ENCODER_OUTPUT_PULSE_PER_REV);
}

void Encoder_Reset(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __HAL_TIM_SET_COUNTER(&s_htim3, 0);
  s_last_left_timer_count = 0;
  s_last_right_count = 0;
  s_last_update_tick = HAL_GetTick();
  g_left_encoder_count = 0;
  g_right_encoder_count = 0;
  g_left_encoder_speed = 0;
  g_right_encoder_speed = 0;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Encoder_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENCODER_RIGHT_A_PIN);
}

void Encoder_EXTI_Callback(uint16_t GPIO_Pin)
{
  GPIO_PinState a_state;
  GPIO_PinState b_state;

  if (GPIO_Pin != ENCODER_RIGHT_A_PIN)
  {
    return;
  }

  a_state = HAL_GPIO_ReadPin(ENCODER_RIGHT_A_PORT, ENCODER_RIGHT_A_PIN);
  b_state = HAL_GPIO_ReadPin(ENCODER_RIGHT_B_PORT, ENCODER_RIGHT_B_PIN);

  if (a_state == b_state)
  {
    g_right_encoder_count++;
  }
  else
  {
    g_right_encoder_count--;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Encoder_EXTI_Callback(GPIO_Pin);
}
