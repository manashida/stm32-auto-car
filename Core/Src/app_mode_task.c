#include "app_mode_task.h"
#include "app_config.h"
#include "buzzer.h"
#include "car.h"
#include "car_mode.h"
#include "follow_task.h"
#include "motion_control.h"
#include "servo.h"
#include "status_led.h"
#include "transport_task.h"
#include <stdio.h>

/* Board buttons: KEY1 on PC15 unloads manually, KEY2 on PC14 cycles
 * AUTO -> Bluetooth/manual -> follow-light-only. Calibrate pull direction if
 * the real board differs.
 */
#define APP_MODE_UNLOAD_KEY_PORT     GPIOC
#define APP_MODE_UNLOAD_KEY_PIN      GPIO_PIN_15
#define APP_MODE_SELECT_KEY_PORT     GPIOC
#define APP_MODE_SELECT_KEY_PIN      GPIO_PIN_14
#define APP_MODE_KEY_ACTIVE_LEVEL    GPIO_PIN_RESET
#define APP_MODE_KEY_DEBOUNCE_MS     30U
#define APP_MODE_UNLOAD_BEEP_MS      180U
#define APP_MODE_AUTO_BEEP_TIMES     1U
#define APP_MODE_BT_BEEP_TIMES       2U
#define APP_MODE_FOLLOW_BEEP_TIMES   3U
#define APP_MODE_BEEP_ON_MS          80U
#define APP_MODE_BEEP_OFF_MS         80U
extern volatile CarMode_t g_car_mode;
extern int16_t g_car_speed;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t stable_pressed;
  uint8_t last_raw_pressed;
  uint32_t last_change_tick;
} AppModeKey_t;

static AppMode_t s_app_mode = APP_MODE_AUTO_TRANSPORT;
static uint8_t s_manual_unload_open = 0U;
static AppModeKey_t s_unload_key = {
  APP_MODE_UNLOAD_KEY_PORT,
  APP_MODE_UNLOAD_KEY_PIN,
  0U,
  0U,
  0U
};
static AppModeKey_t s_select_key = {
  APP_MODE_SELECT_KEY_PORT,
  APP_MODE_SELECT_KEY_PIN,
  0U,
  0U,
  0U
};

static uint8_t AppMode_ReadKeyPressed(const AppModeKey_t *key)
{
  return (HAL_GPIO_ReadPin(key->port, key->pin) == APP_MODE_KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

static uint8_t AppMode_UpdateKey(AppModeKey_t *key, uint32_t now)
{
  uint8_t raw_pressed = AppMode_ReadKeyPressed(key);

  if (raw_pressed != key->last_raw_pressed)
  {
    key->last_raw_pressed = raw_pressed;
    key->last_change_tick = now;
    return 0U;
  }

  if ((raw_pressed != key->stable_pressed) &&
      ((uint32_t)(now - key->last_change_tick) >= APP_MODE_KEY_DEBOUNCE_MS))
  {
    key->stable_pressed = raw_pressed;
    return (raw_pressed != 0U) ? 1U : 0U;
  }

  return 0U;
}

static void AppMode_InitButtons(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = APP_MODE_UNLOAD_KEY_PIN | APP_MODE_SELECT_KEY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void AppMode_PrintMode(const char *name)
{
#if APP_MODE_DEBUG_ENABLE
  printf("APP MODE %s\r\n", name);
#else
  (void)name;
#endif
}

static void AppMode_EnterAutoTransport(void)
{
  Motion_ClearEmergency();
  Motion_ClearNonEmergency();
  s_manual_unload_open = 0U;
  s_app_mode = APP_MODE_AUTO_TRANSPORT;
  Transport_EnterWaitLoad();
  AppMode_PrintMode("AUTO_TRANSPORT");
}

static void AppMode_EnterBluetoothControl(void)
{
  if (s_app_mode == APP_MODE_BLUETOOTH_CONTROL)
  {
    return;
  }

  Motion_ClearEmergency();

  if (g_car_mode == MODE_TRANSPORT)
  {
    Transport_Stop();
  }

  Motion_ClearNonEmergency();
  g_car_speed = 0;
  g_car_mode = MODE_MANUAL;
  Car_Stop();
  Servo_SetAngle(SERVO_CLOSE_ANGLE);
  s_manual_unload_open = 0U;
  StatusLed_SetState(STATUS_LED_MANUAL);
  (void)Motion_Submit(MOTION_OWNER_MANUAL, MOTION_CMD_STOP, 0, 0U);

  s_app_mode = APP_MODE_BLUETOOTH_CONTROL;
  AppMode_PrintMode("BLUETOOTH_CONTROL");
}

static void AppMode_EnterFollowLight(void)
{
  if (s_app_mode == APP_MODE_FOLLOW_LIGHT)
  {
    return;
  }

  Motion_ClearEmergency();

  if (g_car_mode == MODE_TRANSPORT)
  {
    Transport_Stop();
  }

  Motion_ClearNonEmergency();
  g_car_speed = 0;
  g_car_mode = MODE_FOLLOW;
  Follow_Reset();
  Car_Stop();
  Servo_SetAngle(SERVO_CLOSE_ANGLE);
  s_manual_unload_open = 0U;
  StatusLed_SetState(STATUS_LED_FOLLOW);

  s_app_mode = APP_MODE_FOLLOW_LIGHT;
  AppMode_PrintMode("FOLLOW_LIGHT");
}

static void AppMode_BeepMode(AppMode_t mode)
{
  uint8_t times = APP_MODE_AUTO_BEEP_TIMES;

  if (mode == APP_MODE_BLUETOOTH_CONTROL)
  {
    times = APP_MODE_BT_BEEP_TIMES;
  }
  else if (mode == APP_MODE_FOLLOW_LIGHT)
  {
    times = APP_MODE_FOLLOW_BEEP_TIMES;
  }

  Buzzer_BeepTimes(times, APP_MODE_BEEP_ON_MS, APP_MODE_BEEP_OFF_MS);
}

static void AppMode_CycleMode(void)
{
  if (s_app_mode == APP_MODE_AUTO_TRANSPORT)
  {
    AppMode_EnterBluetoothControl();
  }
  else if (s_app_mode == APP_MODE_BLUETOOTH_CONTROL)
  {
    AppMode_EnterFollowLight();
  }
  else
  {
    AppMode_EnterAutoTransport();
  }

  AppMode_BeepMode(s_app_mode);
}

static void AppMode_ManualUnload(void)
{
  if (s_manual_unload_open != 0U)
  {
    Servo_SetAngle(SERVO_CLOSE_ANGLE);
    s_manual_unload_open = 0U;
    StatusLed_SetState((s_app_mode == APP_MODE_FOLLOW_LIGHT) ? STATUS_LED_FOLLOW : STATUS_LED_MANUAL);
    Buzzer_Beep(APP_MODE_UNLOAD_BEEP_MS);
    AppMode_PrintMode("MANUAL_UNLOAD_CLOSE");
    return;
  }

  if (g_car_mode == MODE_TRANSPORT)
  {
    Transport_Stop();
  }

  Motion_ClearNonEmergency();
  g_car_speed = 0;
  g_car_mode = MODE_MANUAL;
  Car_Stop();
  Servo_SetAngle(SERVO_OPEN_ANGLE);
  s_manual_unload_open = 1U;
  StatusLed_SetState(STATUS_LED_UNLOAD);
  Buzzer_Beep(APP_MODE_UNLOAD_BEEP_MS);
  AppMode_PrintMode("MANUAL_UNLOAD");
}

void AppMode_Init(void)
{
  uint32_t now = HAL_GetTick();

  AppMode_InitButtons();
  s_unload_key.stable_pressed = AppMode_ReadKeyPressed(&s_unload_key);
  s_unload_key.last_raw_pressed = s_unload_key.stable_pressed;
  s_unload_key.last_change_tick = now;
  s_select_key.stable_pressed = AppMode_ReadKeyPressed(&s_select_key);
  s_select_key.last_raw_pressed = s_select_key.stable_pressed;
  s_select_key.last_change_tick = now;

  s_app_mode = APP_MODE_BLUETOOTH_CONTROL;
  AppMode_EnterAutoTransport();
}

void AppMode_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t unload_pressed = AppMode_UpdateKey(&s_unload_key, now);
  uint8_t select_pressed = AppMode_UpdateKey(&s_select_key, now);

  if (unload_pressed != 0U)
  {
    AppMode_ManualUnload();
  }
  else if (select_pressed != 0U)
  {
    AppMode_CycleMode();
  }
}

AppMode_t AppMode_GetMode(void)
{
  return s_app_mode;
}
