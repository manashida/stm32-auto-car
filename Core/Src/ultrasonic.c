#include "ultrasonic.h"

#define ULTRASONIC_TRIG_PORT GPIOA
#define ULTRASONIC_TRIG_PIN  GPIO_PIN_4
#define ULTRASONIC_ECHO_PORT GPIOA
#define ULTRASONIC_ECHO_PIN  GPIO_PIN_5

#define ULTRASONIC_TRIG_HIGH_US     10U
#define ULTRASONIC_ECHO_TIMEOUT_US  30000U
#define ULTRASONIC_CM_DIVIDER       58U
#define ULTRASONIC_NEAR_ZERO_CM     2U

typedef enum
{
  ULTRASONIC_STATE_IDLE = 0,
  ULTRASONIC_STATE_TRIG_HIGH,
  ULTRASONIC_STATE_WAIT_RISE,
  ULTRASONIC_STATE_WAIT_FALL
} UltrasonicState_t;

static volatile UltrasonicState_t s_state = ULTRASONIC_STATE_IDLE;
static volatile uint16_t s_distance_cm = ULTRASONIC_TIMEOUT_CM;
static volatile uint8_t s_has_new_distance = 0;
static uint32_t s_state_start_us = 0;
static uint32_t s_echo_start_us = 0;

static void Ultrasonic_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Ultrasonic_Micros(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static uint32_t Ultrasonic_ElapsedUs(uint32_t start_us, uint32_t now_us)
{
  return now_us - start_us;
}

static void Ultrasonic_Finish(uint16_t distance_cm)
{
  s_distance_cm = distance_cm;
  s_has_new_distance = 1;
  s_state = ULTRASONIC_STATE_IDLE;
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
}

static uint16_t Ultrasonic_PulseToCm(uint32_t pulse_width_us)
{
  uint32_t distance = pulse_width_us / ULTRASONIC_CM_DIVIDER;

  if (distance <= ULTRASONIC_NEAR_ZERO_CM)
  {
    return 0U;
  }

  if (distance > ULTRASONIC_TIMEOUT_CM)
  {
    return ULTRASONIC_TIMEOUT_CM;
  }

  return (uint16_t)distance;
}

void Ultrasonic_Init(void)
{
  Ultrasonic_DwtInit();
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
  s_distance_cm = ULTRASONIC_TIMEOUT_CM;
  s_has_new_distance = 0;
  s_state = ULTRASONIC_STATE_IDLE;
}

void Ultrasonic_Start(void)
{
  if (s_state != ULTRASONIC_STATE_IDLE)
  {
    return;
  }

  s_has_new_distance = 0;
  s_state_start_us = Ultrasonic_Micros();
  HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
  s_state = ULTRASONIC_STATE_TRIG_HIGH;
}

void Ultrasonic_Task(void)
{
  uint32_t now_us = Ultrasonic_Micros();

  switch (s_state)
  {
    case ULTRASONIC_STATE_TRIG_HIGH:
      if (Ultrasonic_ElapsedUs(s_state_start_us, now_us) >= ULTRASONIC_TRIG_HIGH_US)
      {
        HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
        s_state_start_us = now_us;
        s_state = ULTRASONIC_STATE_WAIT_RISE;
      }
      break;

    case ULTRASONIC_STATE_WAIT_RISE:
      if (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN) == GPIO_PIN_SET)
      {
        s_echo_start_us = now_us;
        s_state = ULTRASONIC_STATE_WAIT_FALL;
      }
      else if (Ultrasonic_ElapsedUs(s_state_start_us, now_us) >= ULTRASONIC_ECHO_TIMEOUT_US)
      {
        Ultrasonic_Finish(ULTRASONIC_TIMEOUT_CM);
      }
      break;

    case ULTRASONIC_STATE_WAIT_FALL:
      if (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN) == GPIO_PIN_RESET)
      {
        Ultrasonic_Finish(Ultrasonic_PulseToCm(Ultrasonic_ElapsedUs(s_echo_start_us, now_us)));
      }
      else if (Ultrasonic_ElapsedUs(s_echo_start_us, now_us) >= ULTRASONIC_ECHO_TIMEOUT_US)
      {
        Ultrasonic_Finish(ULTRASONIC_TIMEOUT_CM);
      }
      break;

    case ULTRASONIC_STATE_IDLE:
    default:
      break;
  }
}

uint8_t Ultrasonic_IsBusy(void)
{
  return (s_state != ULTRASONIC_STATE_IDLE) ? 1U : 0U;
}

uint8_t Ultrasonic_IsDistanceValid(uint16_t distance_cm)
{
  return ((distance_cm > 0U) && (distance_cm < ULTRASONIC_TIMEOUT_CM)) ? 1U : 0U;
}

uint8_t Ultrasonic_TakeDistanceCm(uint16_t *distance_cm)
{
  if ((distance_cm == NULL) || (s_has_new_distance == 0U))
  {
    return 0U;
  }

  *distance_cm = s_distance_cm;
  s_has_new_distance = 0;
  return 1U;
}

uint16_t Ultrasonic_GetDistanceCm(void)
{
  return s_distance_cm;
}
