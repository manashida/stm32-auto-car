#include "bluetooth.h"
#include "app_health.h"
#include "app_mode_task.h"
#include "hx711.h"
#include "motion_control.h"
#include "servo.h"
#include "status_led.h"
#include "transport_task.h"
#include "usart.h"
#include "uwb_follow.h"
#include <stdio.h>

#define BLUETOOTH_SPEED_STEP     100
#define BLUETOOTH_DEFAULT_SPEED  500
#define BLUETOOTH_MAX_SPEED      1000
#define BLUETOOTH_TURN_SPEED_PERCENT 60
#define BLUETOOTH_TURN_MIN_SPEED     120
#define BLUETOOTH_LOAD_OPEN_ANGLE  SERVO_OPEN_ANGLE
#define BLUETOOTH_LOAD_CLOSE_ANGLE SERVO_CLOSE_ANGLE
#define BLUETOOTH_RX_QUEUE_SIZE    16U
#define BLUETOOTH_MANUAL_MOTION_TTL_MS 800U

extern int16_t g_car_speed;
extern uint16_t g_distance_cm;
extern int32_t g_weight_g;
static uint8_t s_rx_byte = 0;
static volatile uint8_t s_rx_queue[BLUETOOTH_RX_QUEUE_SIZE];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;
static volatile uint8_t s_rx_overflow = 0;
static volatile uint8_t s_rx_recover_requested = 0;
static volatile uint32_t s_rx_error_count = 0;
static volatile uint32_t s_rx_last_error = HAL_UART_ERROR_NONE;
static uint8_t s_motion_command = 'S';

static void Bluetooth_SendText(const char *text)
{
  uint16_t len = 0;

  while (text[len] != '\0')
  {
    len++;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)text, len, 100);
}

static void Bluetooth_SendSpeed(void)
{
  char text[32];

  snprintf(text, sizeof(text), "SPD=%d\r\n", g_car_speed);
  Bluetooth_SendText(text);
}

static void Bluetooth_SendHealth(void)
{
  AppHealthSnapshot_t health;
  char text[128];
  int len;

  AppHealth_GetSnapshot(&health);

  len = snprintf(text,
                 sizeof(text),
                 "HL loop=%lu last=%lu max=%lu over=%lu\r\n",
                 (unsigned long)health.loop_count,
                 (unsigned long)health.last_loop_ms,
                 (unsigned long)health.max_loop_ms,
                 (unsigned long)health.loop_overrun_count);
  if (len > 0)
  {
    Bluetooth_SendText(text);
  }

  len = snprintf(text,
                 sizeof(text),
                 "HS mode=%d tr=%s dis=%u wt=%ld hx=%u rxerr=%lu\r\n",
                 (int)g_car_mode,
                 Transport_GetStateName(),
                 g_distance_cm,
                 (long)g_weight_g,
                 (unsigned)HX711_GetLastStatus(),
                 (unsigned long)s_rx_error_count);
  if (len > 0)
  {
    Bluetooth_SendText(text);
  }

  len = snprintf(text,
                 sizeof(text),
                 "HT bt=%lu us=%lu wt=%lu mode=%lu oled=%lu mot=%lu\r\n",
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_BLUETOOTH],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_ULTRASONIC],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_WEIGHT],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_MODE_CONTROL],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_OLED],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_MOTION]);
  if (len > 0)
  {
    Bluetooth_SendText(text);
  }

  len = snprintf(text,
                 sizeof(text),
                 "HU uwb=%lu follow=%lu uwbrx=%lu uwbok=%lu uwberr=%lu uarterr=%lu\r\n",
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_UWB],
                 (unsigned long)health.task_max_ms[APP_HEALTH_TASK_FOLLOW],
                 (unsigned long)UwbFollow_GetRxCount(),
                 (unsigned long)UwbFollow_GetParseOkCount(),
                 (unsigned long)UwbFollow_GetParseErrCount(),
                 (unsigned long)UwbFollow_GetRxErrorCount());
  if (len > 0)
  {
    Bluetooth_SendText(text);
  }
}

static uint8_t Bluetooth_IsEmergencyLocked(void)
{
  return ((g_car_mode == MODE_EMERGENCY_STOP) ||
          (Motion_IsEmergencyLocked() != 0U)) ? 1U : 0U;
}

static uint8_t Bluetooth_BlockEmergencyMotion(void)
{
  if (Bluetooth_IsEmergencyLocked() == 0U)
  {
    return 0U;
  }

  Bluetooth_SendText("EMERGENCY LOCK\r\n");
  return 1U;
}

static uint8_t Bluetooth_RequireControlMode(void)
{
  if (AppMode_GetMode() == APP_MODE_BLUETOOTH_CONTROL)
  {
    return 1U;
  }

  Bluetooth_SendText("BT MODE OFF\r\n");
  return 0U;
}

static void Bluetooth_SetStatusLedByMode(void)
{
  switch (g_car_mode)
  {
    case MODE_AUTO:
      StatusLed_SetState(STATUS_LED_AUTO);
      break;

    case MODE_TRACK:
      StatusLed_SetState(STATUS_LED_TRACK);
      break;

    case MODE_FOLLOW:
      StatusLed_SetState(STATUS_LED_FOLLOW);
      break;

    case MODE_TRANSPORT:
      StatusLed_SetState(STATUS_LED_TRANSPORT);
      break;

    case MODE_EMERGENCY_STOP:
      StatusLed_SetState(STATUS_LED_EMERGENCY);
      break;

    case MODE_MANUAL:
    default:
      StatusLed_SetState(STATUS_LED_MANUAL);
      break;
  }
}

static int16_t Bluetooth_LimitSpeed(int16_t speed)
{
  if (speed > BLUETOOTH_MAX_SPEED)
  {
    return BLUETOOTH_MAX_SPEED;
  }
  if (speed < 0)
  {
    return 0;
  }

  return speed;
}

static int16_t Bluetooth_GetRunSpeed(void)
{
  if (g_car_speed <= 0)
  {
    g_car_speed = BLUETOOTH_DEFAULT_SPEED;
  }

  return Bluetooth_LimitSpeed(g_car_speed);
}

static int16_t Bluetooth_GetTurnSpeed(void)
{
  int16_t speed = Bluetooth_GetRunSpeed();

  speed = (int16_t)((speed * BLUETOOTH_TURN_SPEED_PERCENT) / 100);

  if ((speed > 0) && (speed < BLUETOOTH_TURN_MIN_SPEED))
  {
    speed = BLUETOOTH_TURN_MIN_SPEED;
  }

  return Bluetooth_LimitSpeed(speed);
}

static void Bluetooth_AddSpeed(void)
{
  g_car_speed = Bluetooth_LimitSpeed((int16_t)(g_car_speed + BLUETOOTH_SPEED_STEP));
}

static void Bluetooth_SubSpeed(void)
{
  g_car_speed = Bluetooth_LimitSpeed((int16_t)(g_car_speed - BLUETOOTH_SPEED_STEP));

  if (g_car_speed == 0)
  {
    Motion_Submit(MOTION_OWNER_MANUAL, MOTION_CMD_STOP, 0, 0U);
    s_motion_command = 'S';
  }
}

static uint8_t Bluetooth_SubmitManualMotion(MotionCommand_t command, int16_t speed)
{
  if (Motion_Submit(MOTION_OWNER_MANUAL,
                    command,
                    speed,
                    BLUETOOTH_MANUAL_MOTION_TTL_MS) == 0U)
  {
    Bluetooth_SendText("MOTION BUSY\r\n");
    return 0U;
  }

  return 1U;
}

static void Bluetooth_ApplyMotion(void)
{
  if ((g_car_mode != MODE_MANUAL) || (g_car_speed <= 0))
  {
    return;
  }

  switch (s_motion_command)
  {
    case 'F':
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_FORWARD, g_car_speed);
      break;

    case 'B':
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_BACKWARD, g_car_speed);
      break;

    case 'L':
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_LEFT, Bluetooth_GetTurnSpeed());
      break;

    case 'R':
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_RIGHT, Bluetooth_GetTurnSpeed());
      break;

    default:
      break;
  }
}

static uint8_t Bluetooth_SetManualMode(void)
{
  if (Bluetooth_BlockEmergencyMotion() != 0U)
  {
    return 0U;
  }

  if (g_car_mode == MODE_TRANSPORT)
  {
    Transport_Stop();
  }

  Motion_ClearNonEmergency();
  g_car_mode = MODE_MANUAL;
  StatusLed_SetState(STATUS_LED_MANUAL);
  return 1U;
}

static uint8_t Bluetooth_EnterStoppedMode(CarMode_t mode)
{
  if (Bluetooth_IsEmergencyLocked() != 0U)
  {
    if (mode != MODE_MANUAL)
    {
      Bluetooth_SendText("EMERGENCY LOCK\r\n");
      return 0U;
    }

    Motion_ClearEmergency();
  }

  if ((g_car_mode == MODE_TRANSPORT) && (mode != MODE_TRANSPORT))
  {
    Transport_Stop();
  }

  if (mode == MODE_MANUAL)
  {
    Motion_ClearNonEmergency();
  }

  g_car_mode = mode;
  s_motion_command = 'S';
  g_car_speed = 0;
  Motion_Submit(MOTION_OWNER_MANUAL, MOTION_CMD_STOP, 0, 0U);
  return 1U;
}

static uint8_t Bluetooth_IsLoadCommandAllowed(void)
{
  return ((AppMode_GetMode() == APP_MODE_BLUETOOTH_CONTROL) &&
          ((g_car_mode == MODE_MANUAL) ||
           (g_car_mode == MODE_EMERGENCY_STOP))) ? 1U : 0U;
}

static void Bluetooth_QueuePushFromISR(uint8_t command)
{
  uint8_t next_head = (uint8_t)((s_rx_head + 1U) % BLUETOOTH_RX_QUEUE_SIZE);

  if (next_head == s_rx_tail)
  {
    s_rx_overflow = 1U;
    return;
  }

  s_rx_queue[s_rx_head] = command;
  s_rx_head = next_head;
}

static uint8_t Bluetooth_QueuePop(uint8_t *command)
{
  uint8_t has_command = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (s_rx_tail != s_rx_head)
  {
    *command = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % BLUETOOTH_RX_QUEUE_SIZE);
    has_command = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return has_command;
}

static void Bluetooth_RecoverReceiveIfNeeded(void)
{
  uint8_t recover_requested;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  recover_requested = s_rx_recover_requested;
  s_rx_recover_requested = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (recover_requested == 0U)
  {
    return;
  }

  (void)HAL_UART_AbortReceive(&huart1);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  Bluetooth_StartReceive_IT();
}

static void Bluetooth_ProcessCommand(uint8_t command)
{
  if ((command == '\r') || (command == '\n'))
  {
    return;
  }

  switch (command)
  {
    case 'F':
    case 'f':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_SetManualMode() == 0U)
      {
        break;
      }
      s_motion_command = 'F';
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_FORWARD, Bluetooth_GetRunSpeed());
      break;

    case 'B':
    case 'b':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_SetManualMode() == 0U)
      {
        break;
      }
      s_motion_command = 'B';
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_BACKWARD, Bluetooth_GetRunSpeed());
      break;

    case 'L':
    case 'l':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_SetManualMode() == 0U)
      {
        break;
      }
      s_motion_command = 'L';
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_LEFT, Bluetooth_GetTurnSpeed());
      break;

    case 'R':
    case 'r':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_SetManualMode() == 0U)
      {
        break;
      }
      s_motion_command = 'R';
      (void)Bluetooth_SubmitManualMotion(MOTION_CMD_RIGHT, Bluetooth_GetTurnSpeed());
      break;

    case 'S':
    case 's':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      (void)Bluetooth_EnterStoppedMode(MODE_MANUAL);
      break;

    case 'A':
    case 'a':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_EnterStoppedMode(MODE_AUTO) != 0U)
      {
        StatusLed_SetState(STATUS_LED_AUTO);
        Bluetooth_SendText("AUTO AVOID\r\n");
      }
      break;

    case 'T':
    case 't':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_EnterStoppedMode(MODE_TRACK) != 0U)
      {
        StatusLed_SetState(STATUS_LED_TRACK);
        Bluetooth_SendText("AUTO TRACK\r\n");
      }
      break;

    case 'U':
    case 'u':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      if (Bluetooth_EnterStoppedMode(MODE_FOLLOW) != 0U)
      {
        StatusLed_SetState(STATUS_LED_FOLLOW);
        Bluetooth_SendText("UWB FOLLOW\r\n");
      }
      break;

    case '+':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      Bluetooth_AddSpeed();
      Bluetooth_ApplyMotion();
      Bluetooth_SendSpeed();
      break;

    case '-':
      if (Bluetooth_RequireControlMode() == 0U)
      {
        break;
      }
      Bluetooth_SubSpeed();
      Bluetooth_ApplyMotion();
      Bluetooth_SendSpeed();
      break;

    case 'O':
    case 'o':
      if (Bluetooth_IsLoadCommandAllowed() != 0U)
      {
        Servo_SetAngle(BLUETOOTH_LOAD_OPEN_ANGLE);
        StatusLed_SetState(STATUS_LED_UNLOAD);
        Bluetooth_SendText("LOAD OPEN\r\n");
      }
      else
      {
        Bluetooth_SendText("LOAD DENIED\r\n");
      }
      break;

    case 'C':
    case 'c':
      if (Bluetooth_IsLoadCommandAllowed() != 0U)
      {
        Servo_SetAngle(BLUETOOTH_LOAD_CLOSE_ANGLE);
        Bluetooth_SetStatusLedByMode();
        Bluetooth_SendText("LOAD CLOSE\r\n");
      }
      else
      {
        Bluetooth_SendText("LOAD DENIED\r\n");
      }
      break;

    case 'X':
    case 'x':
      Transport_EmergencyStop();
      Bluetooth_SendText("EMERGENCY STOP\r\n");
      break;

    case 'H':
    case 'h':
      Bluetooth_SendHealth();
      break;

    default:
      Bluetooth_SendText("UNKNOWN CMD\r\n");
      break;
  }
}

void Bluetooth_Init(void)
{
  s_rx_byte = 0;
  s_rx_head = 0;
  s_rx_tail = 0;
  s_rx_overflow = 0;
  s_rx_recover_requested = 0;
  s_rx_error_count = 0;
  s_rx_last_error = HAL_UART_ERROR_NONE;
  g_car_mode = MODE_MANUAL;
  s_motion_command = 'S';
  Motion_Submit(MOTION_OWNER_MANUAL, MOTION_CMD_STOP, 0, 0U);
  Bluetooth_StartReceive_IT();
}

void Bluetooth_Task(void)
{
  uint8_t command = 0;

  Bluetooth_RecoverReceiveIfNeeded();

  if (s_rx_overflow != 0U)
  {
    s_rx_overflow = 0U;
    Bluetooth_SendText("rx overflow\r\n");
  }

  while (Bluetooth_QueuePop(&command) != 0U)
  {
    Bluetooth_ProcessCommand(command);
  }
}

void Bluetooth_StartReceive_IT(void)
{
  if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1) != HAL_OK)
  {
    s_rx_recover_requested = 1U;
  }
}

void Bluetooth_OnRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  Bluetooth_QueuePushFromISR(s_rx_byte);
  Bluetooth_StartReceive_IT();
}

void Bluetooth_OnErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  s_rx_last_error = HAL_UART_GetError(huart);
  s_rx_error_count++;
  s_rx_recover_requested = 1U;
}

uint32_t Bluetooth_GetRxErrorCount(void)
{
  return s_rx_error_count;
}

uint32_t Bluetooth_GetLastRxError(void)
{
  return s_rx_last_error;
}

CarMode_t Bluetooth_GetMode(void)
{
  return g_car_mode;
}
