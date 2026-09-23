#include "buzzer.h"
#include "app_config.h"
#if BUZZER_DEBUG_ENABLE
#include "car_mode.h"
#include "transport_task.h"
#include <stdio.h>
#endif

#define BUZZER_PORT         GPIOA
#define BUZZER_PIN          GPIO_PIN_11
#define BUZZER_ACTIVE_LEVEL    GPIO_PIN_RESET
#define BUZZER_INACTIVE_LEVEL  GPIO_PIN_SET
#define BUZZER_ALARM_ON_MS     100U
#define BUZZER_ALARM_OFF_MS    900U

typedef enum
{
  BUZZER_MODE_IDLE = 0,
  BUZZER_MODE_ON,
  BUZZER_MODE_BEEP,
  BUZZER_MODE_BEEP_TIMES,
  BUZZER_MODE_ALARM
} BuzzerMode_t;

static BuzzerMode_t s_buzzer_mode = BUZZER_MODE_IDLE;
static uint8_t s_beep_remaining = 0;
static uint8_t s_phase_on = 0;
static uint32_t s_next_tick = 0;
static uint32_t s_on_ms = 0;
static uint32_t s_off_ms = 0;

#if BUZZER_DEBUG_ENABLE
extern volatile CarMode_t g_car_mode;
static void Buzzer_DebugPrint(const char *event, uint32_t value, void *caller)
{
  printf("[BUZZ] tick=%lu event=%s value=%lu mode=%d transport=%s caller=%p\r\n",
         (unsigned long)HAL_GetTick(),
         event,
         (unsigned long)value,
         (int)g_car_mode,
         Transport_GetStateName(),
         caller);
}
#else
static void Buzzer_DebugPrint(const char *event, uint32_t value, void *caller)
{
  (void)event;
  (void)value;
  (void)caller;
}
#endif

static void Buzzer_Write(uint8_t on)
{
  HAL_GPIO_WritePin(BUZZER_PORT,
                    BUZZER_PIN,
                    (on != 0U) ? BUZZER_ACTIVE_LEVEL : BUZZER_INACTIVE_LEVEL);
}

static uint8_t Buzzer_TimeReached(uint32_t tick)
{
  return ((int32_t)(HAL_GetTick() - tick) >= 0) ? 1U : 0U;
}

static void Buzzer_ClearState(void)
{
  s_buzzer_mode = BUZZER_MODE_IDLE;
  s_beep_remaining = 0;
  s_phase_on = 0;
  s_next_tick = 0;
  s_on_ms = 0;
  s_off_ms = 0;
}

static void Buzzer_StartAlarm(uint32_t on_ms, uint32_t off_ms, void *caller)
{
  if ((on_ms == 0U) || (off_ms == 0U))
  {
    if (s_buzzer_mode == BUZZER_MODE_ALARM)
    {
      Buzzer_Off();
    }
    return;
  }

  if ((s_buzzer_mode == BUZZER_MODE_ALARM) &&
      (s_on_ms == on_ms) &&
      (s_off_ms == off_ms))
  {
    return;
  }

  Buzzer_DebugPrint("alarm", on_ms, caller);
  s_buzzer_mode = BUZZER_MODE_ALARM;
  s_phase_on = 1U;
  s_on_ms = on_ms;
  s_off_ms = off_ms;
  s_next_tick = HAL_GetTick() + s_on_ms;
  Buzzer_Write(1U);
}

void Buzzer_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_INACTIVE_LEVEL);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  Buzzer_Off();
}

void Buzzer_On(void)
{
  Buzzer_DebugPrint("on", 1U, __builtin_return_address(0));
  s_buzzer_mode = BUZZER_MODE_ON;
  s_beep_remaining = 0;
  s_phase_on = 1U;
  Buzzer_Write(1U);
}

void Buzzer_Off(void)
{
  Buzzer_ClearState();
  Buzzer_Write(0U);
}

void Buzzer_Stop(void)
{
  Buzzer_Off();
}

void Buzzer_Beep(uint16_t duration_ms)
{
  if (duration_ms == 0U)
  {
    Buzzer_Off();
    return;
  }

  Buzzer_DebugPrint("beep", duration_ms, __builtin_return_address(0));
  s_buzzer_mode = BUZZER_MODE_BEEP;
  s_beep_remaining = 0;
  s_phase_on = 1U;
  s_next_tick = HAL_GetTick() + duration_ms;
  Buzzer_Write(1U);
}

void Buzzer_BeepTimes(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
  if ((times == 0U) || (on_ms == 0U))
  {
    Buzzer_Off();
    return;
  }

  s_buzzer_mode = BUZZER_MODE_BEEP_TIMES;
  s_beep_remaining = times;
  s_phase_on = 1U;
  s_on_ms = on_ms;
  s_off_ms = off_ms;
  s_next_tick = HAL_GetTick() + s_on_ms;
  Buzzer_DebugPrint("beep_times", times, __builtin_return_address(0));
  Buzzer_Write(1U);
}

void Buzzer_SetAlarm(uint8_t enable)
{
  if (enable == 0U)
  {
    if (s_buzzer_mode == BUZZER_MODE_ALARM)
    {
      Buzzer_Off();
    }
    return;
  }

  Buzzer_StartAlarm(BUZZER_ALARM_ON_MS,
                    BUZZER_ALARM_OFF_MS,
                    __builtin_return_address(0));
}

void Buzzer_SetPattern(uint16_t on_ms, uint16_t off_ms)
{
  Buzzer_StartAlarm(on_ms, off_ms, __builtin_return_address(0));
}

void Buzzer_SetAlarmPattern(uint32_t on_ms, uint32_t off_ms)
{
  Buzzer_StartAlarm(on_ms, off_ms, __builtin_return_address(0));
}

void Buzzer_Task(void)
{
  if (s_buzzer_mode == BUZZER_MODE_BEEP)
  {
    if (Buzzer_TimeReached(s_next_tick) != 0U)
    {
      Buzzer_Off();
    }
  }
  else if (s_buzzer_mode == BUZZER_MODE_BEEP_TIMES)
  {
    if (Buzzer_TimeReached(s_next_tick) == 0U)
    {
      return;
    }

    if (s_phase_on != 0U)
    {
      if (s_beep_remaining > 0U)
      {
        s_beep_remaining--;
      }

      Buzzer_Write(0U);
      s_phase_on = 0U;

      if (s_beep_remaining == 0U)
      {
        s_buzzer_mode = BUZZER_MODE_IDLE;
        return;
      }

      s_next_tick = HAL_GetTick() + s_off_ms;
    }
    else
    {
      Buzzer_Write(1U);
      s_phase_on = 1U;
      s_next_tick = HAL_GetTick() + s_on_ms;
    }
  }
  else if (s_buzzer_mode == BUZZER_MODE_ALARM)
  {
    if (Buzzer_TimeReached(s_next_tick) == 0U)
    {
      return;
    }

    if (s_phase_on != 0U)
    {
      Buzzer_Write(0U);
      s_phase_on = 0U;
      s_next_tick = HAL_GetTick() + s_off_ms;
    }
    else
    {
      Buzzer_Write(1U);
      s_phase_on = 1U;
      s_next_tick = HAL_GetTick() + s_on_ms;
    }
  }
}
