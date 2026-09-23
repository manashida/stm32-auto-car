#include "auto_run.h"
#include "buzzer.h"
#include "encoder.h"
#include "motion_control.h"
#include "status_led.h"
#include "ultrasonic.h"

#define AUTO_CLEAR_DISTANCE_CM 35U
#define AUTO_SLOW_DISTANCE_CM  20U

#define AUTO_FORWARD_SPEED 500
#define AUTO_SLOW_SPEED    300
#define AUTO_BACK_SPEED    300
#define AUTO_TURN_SPEED    400

#define AUTO_STOP_MS       200U
#define AUTO_BACK_MS       450U
#define AUTO_TURN_MS       550U
#define AUTO_FORWARD_MS    200U

#define AUTO_CRUISE_TTL_MS          250U
#define AUTO_STOP_TTL_MS            220U
#define AUTO_BACK_TTL_MS            500U
#define AUTO_TURN_TTL_MS            600U
#define AUTO_FORWARD_TTL_MS         250U

#define AUTO_OBSTACLE_CONFIRM_COUNT 2U
#define AUTO_CLEAR_CONFIRM_COUNT    2U
#define AUTO_STUCK_DELTA_CM         3U

#define AUTO_INVALID_HOLD_MS        300U
#define AUTO_INVALID_SLOW_MS        60000U
#define AUTO_STUCK_MIN_PULSE_DELTA  2L
#define AUTO_SPEED_DOWN_DISTANCE_CM 30U
#define AUTO_SPEED_UP_DISTANCE_CM   40U
#define AUTO_SPEED_HOLD_MS          300U

typedef enum
{
  AUTO_STATE_RUN = 0,
  AUTO_STATE_STOP,
  AUTO_STATE_BACK,
  AUTO_STATE_RIGHT,
  AUTO_STATE_FORWARD
} AutoState_t;

static AutoState_t s_auto_state = AUTO_STATE_RUN;
static uint32_t s_state_start_tick = 0;
static uint16_t s_filtered_distance_cm = ULTRASONIC_TIMEOUT_CM;
static uint16_t s_state_start_distance_cm = ULTRASONIC_TIMEOUT_CM;
static uint8_t s_distance_valid = 0U;
static uint8_t s_obstacle_count = 0U;
static uint8_t s_clear_count = 0U;
static uint8_t s_motion_accepted = 1U;
static uint32_t s_invalid_start_tick = 0U;
static uint8_t s_invalid_active = 0U;
static MotionCommand_t s_last_safe_command = MOTION_CMD_FORWARD;
static int16_t s_last_safe_speed = AUTO_SLOW_SPEED;
static int32_t s_state_start_left_pulse = 0;
static int32_t s_state_start_right_pulse = 0;
static int16_t s_cruise_speed = AUTO_FORWARD_SPEED;
static uint32_t s_cruise_speed_tick = 0U;

static void Auto_HoldAvoidStop(void);

static uint16_t Auto_AbsDistanceDelta(uint16_t a, uint16_t b)
{
  return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static int32_t Auto_Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static void Auto_UpdateSensorLayer(void)
{
  s_distance_valid = Ultrasonic_IsDistanceValid(g_distance_cm);

  if (s_distance_valid == 0U)
  {
    if (s_invalid_active == 0U)
    {
      s_invalid_active = 1U;
      s_invalid_start_tick = HAL_GetTick();
    }
    s_obstacle_count = 0U;
    s_clear_count = 0U;
    return;
  }

  s_invalid_active = 0U;
  s_filtered_distance_cm = g_distance_cm;

  if (g_distance_cm < AUTO_SLOW_DISTANCE_CM)
  {
    if (s_obstacle_count < AUTO_OBSTACLE_CONFIRM_COUNT)
    {
      s_obstacle_count++;
    }
    s_clear_count = 0U;
  }
  else if (g_distance_cm > AUTO_CLEAR_DISTANCE_CM)
  {
    if (s_clear_count < AUTO_CLEAR_CONFIRM_COUNT)
    {
      s_clear_count++;
    }
    s_obstacle_count = 0U;
  }
  else
  {
    s_obstacle_count = 0U;
    s_clear_count = 0U;
  }
}

static uint8_t Auto_ObstacleDetected(void)
{
  return (s_obstacle_count >= AUTO_OBSTACLE_CONFIRM_COUNT) ? 1U : 0U;
}

static uint8_t Auto_PathClear(void)
{
  return (s_clear_count >= AUTO_CLEAR_CONFIRM_COUNT) ? 1U : 0U;
}

static uint8_t Auto_BasicStuckDetected(void)
{
  int32_t left_delta = Auto_Abs32(Encoder_GetLeftPulse() - s_state_start_left_pulse);
  int32_t right_delta = Auto_Abs32(Encoder_GetRightPulse() - s_state_start_right_pulse);

  if ((s_distance_valid == 0U) ||
      (Ultrasonic_IsDistanceValid(s_state_start_distance_cm) == 0U))
  {
    return 0U;
  }

  if ((left_delta + right_delta) >= AUTO_STUCK_MIN_PULSE_DELTA)
  {
    return 0U;
  }

  return (Auto_AbsDistanceDelta(s_filtered_distance_cm,
                                s_state_start_distance_cm) <= AUTO_STUCK_DELTA_CM) ? 1U : 0U;
}

static int16_t Auto_SelectCruiseSpeed(uint16_t distance_cm)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - s_cruise_speed_tick) < AUTO_SPEED_HOLD_MS)
  {
    return s_cruise_speed;
  }

  if ((s_cruise_speed != AUTO_FORWARD_SPEED) &&
      (distance_cm > AUTO_SPEED_UP_DISTANCE_CM))
  {
    s_cruise_speed = AUTO_FORWARD_SPEED;
    s_cruise_speed_tick = now;
  }
  else if ((s_cruise_speed != AUTO_SLOW_SPEED) &&
           (distance_cm < AUTO_SPEED_DOWN_DISTANCE_CM))
  {
    s_cruise_speed = AUTO_SLOW_SPEED;
    s_cruise_speed_tick = now;
  }

  return s_cruise_speed;
}

static uint8_t Auto_RequestMotion(MotionCommand_t command,
                                  int16_t speed,
                                  uint32_t ttl_ms)
{
  s_motion_accepted = Motion_Submit(MOTION_OWNER_AUTO, command, speed, ttl_ms);
  if (s_motion_accepted == 0U)
  {
    s_motion_accepted = Motion_Submit(MOTION_OWNER_AUTO, command, speed, ttl_ms);
  }

  if ((s_motion_accepted != 0U) && (command != MOTION_CMD_STOP))
  {
    s_last_safe_command = command;
    s_last_safe_speed = speed;
  }

  return s_motion_accepted;
}

static void Auto_HandleInvalidDistance(void)
{
  uint32_t elapsed = (uint32_t)(HAL_GetTick() - s_invalid_start_tick);

  if (elapsed < AUTO_INVALID_HOLD_MS)
  {
    if (Auto_RequestMotion(s_last_safe_command, s_last_safe_speed, AUTO_CRUISE_TTL_MS) == 0U)
    {
      Auto_HoldAvoidStop();
    }
  }
  else if (elapsed < AUTO_INVALID_SLOW_MS)
  {
    if (Auto_RequestMotion(MOTION_CMD_FORWARD, AUTO_SLOW_SPEED, AUTO_CRUISE_TTL_MS) == 0U)
    {
      Auto_HoldAvoidStop();
    }
  }
  else
  {
    Auto_HoldAvoidStop();
  }
}

static void Auto_HoldAvoidStop(void)
{
  s_auto_state = AUTO_STATE_STOP;
  s_state_start_tick = HAL_GetTick();
  s_state_start_distance_cm = s_filtered_distance_cm;
  s_state_start_left_pulse = Encoder_GetLeftPulse();
  s_state_start_right_pulse = Encoder_GetRightPulse();
  Auto_RequestMotion(MOTION_CMD_STOP, 0, AUTO_STOP_TTL_MS);
  Buzzer_SetAlarmPattern(80U, 600U);
  StatusLed_SetState(STATUS_LED_OBSTACLE);
}

static void Auto_EnterState(AutoState_t state)
{
  s_auto_state = state;
  s_state_start_tick = HAL_GetTick();
  s_state_start_distance_cm = s_filtered_distance_cm;
  s_state_start_left_pulse = Encoder_GetLeftPulse();
  s_state_start_right_pulse = Encoder_GetRightPulse();

  switch (state)
  {
    case AUTO_STATE_STOP:
      Auto_RequestMotion(MOTION_CMD_STOP, 0, AUTO_STOP_TTL_MS);
      Buzzer_SetAlarmPattern(80U, 600U);
      StatusLed_SetState(STATUS_LED_OBSTACLE);
      break;

    case AUTO_STATE_BACK:
      if (Auto_RequestMotion(MOTION_CMD_BACKWARD, AUTO_BACK_SPEED, AUTO_BACK_TTL_MS) == 0U)
      {
        Auto_HoldAvoidStop();
        return;
      }
      Buzzer_SetAlarmPattern(80U, 600U);
      StatusLed_SetState(STATUS_LED_OBSTACLE);
      break;

    case AUTO_STATE_RIGHT:
      if (Auto_RequestMotion(MOTION_CMD_RIGHT, AUTO_TURN_SPEED, AUTO_TURN_TTL_MS) == 0U)
      {
        Auto_HoldAvoidStop();
        return;
      }
      Buzzer_SetAlarmPattern(80U, 600U);
      StatusLed_SetState(STATUS_LED_OBSTACLE);
      break;

    case AUTO_STATE_FORWARD:
      s_clear_count = 0U;
      if (Auto_RequestMotion(MOTION_CMD_FORWARD, AUTO_FORWARD_SPEED, AUTO_FORWARD_TTL_MS) == 0U)
      {
        Auto_HoldAvoidStop();
        return;
      }
      Buzzer_SetAlarm(0U);
      StatusLed_SetState(STATUS_LED_AUTO);
      break;

    case AUTO_STATE_RUN:
    default:
      Buzzer_SetAlarm(0U);
      StatusLed_SetState(STATUS_LED_AUTO);
      break;
  }
}

static uint8_t Auto_StateElapsed(uint32_t duration_ms)
{
  return ((uint32_t)(HAL_GetTick() - s_state_start_tick) >= duration_ms) ? 1U : 0U;
}

static void Auto_RunDecisionLayer(void)
{
  uint16_t distance_cm = s_filtered_distance_cm;
  int16_t cruise_speed;

  if (s_distance_valid == 0U)
  {
    Auto_HandleInvalidDistance();
  }
  else if (Auto_ObstacleDetected() != 0U)
  {
    Auto_EnterState(AUTO_STATE_STOP);
  }
  else if (distance_cm >= AUTO_SLOW_DISTANCE_CM)
  {
    cruise_speed = Auto_SelectCruiseSpeed(distance_cm);
    if (Auto_RequestMotion(MOTION_CMD_FORWARD, cruise_speed, AUTO_CRUISE_TTL_MS) == 0U)
    {
      Auto_HoldAvoidStop();
      return;
    }
    Buzzer_SetAlarm(0U);
    StatusLed_SetState(STATUS_LED_AUTO);
  }
  else
  {
    Auto_EnterState(AUTO_STATE_STOP);
  }
}

void Auto_Init(void)
{
  s_auto_state = AUTO_STATE_RUN;
  s_state_start_tick = HAL_GetTick();
  s_filtered_distance_cm = ULTRASONIC_TIMEOUT_CM;
  s_state_start_distance_cm = ULTRASONIC_TIMEOUT_CM;
  s_distance_valid = 0U;
  s_obstacle_count = 0U;
  s_clear_count = 0U;
  s_motion_accepted = 1U;
  s_invalid_start_tick = 0U;
  s_invalid_active = 0U;
  s_last_safe_command = MOTION_CMD_FORWARD;
  s_last_safe_speed = AUTO_SLOW_SPEED;
  s_state_start_left_pulse = Encoder_GetLeftPulse();
  s_state_start_right_pulse = Encoder_GetRightPulse();
  s_cruise_speed = AUTO_FORWARD_SPEED;
  s_cruise_speed_tick = HAL_GetTick();
  Buzzer_SetAlarm(0U);
}

void Auto_Task(void)
{
  if (g_car_mode != MODE_AUTO)
  {
    s_auto_state = AUTO_STATE_RUN;
    s_motion_accepted = 1U;
    Buzzer_SetAlarm(0U);
    return;
  }

  Auto_UpdateSensorLayer();

  if (s_distance_valid == 0U)
  {
    Auto_HandleInvalidDistance();
    return;
  }

  switch (s_auto_state)
  {
    case AUTO_STATE_RUN:
      Auto_RunDecisionLayer();
      break;

    case AUTO_STATE_STOP:
      if (s_motion_accepted == 0U)
      {
        Auto_HoldAvoidStop();
        break;
      }
      if (Auto_StateElapsed(AUTO_STOP_MS) != 0U)
      {
        Auto_EnterState(AUTO_STATE_BACK);
      }
      break;

    case AUTO_STATE_BACK:
      if (s_motion_accepted == 0U)
      {
        Auto_HoldAvoidStop();
        break;
      }
      if (Auto_StateElapsed(AUTO_BACK_MS) != 0U)
      {
        Auto_EnterState(AUTO_STATE_RIGHT);
      }
      break;

    case AUTO_STATE_RIGHT:
      if (s_motion_accepted == 0U)
      {
        Auto_HoldAvoidStop();
        break;
      }
      if (Auto_StateElapsed(AUTO_TURN_MS) != 0U)
      {
        Auto_EnterState(AUTO_STATE_FORWARD);
      }
      break;

    case AUTO_STATE_FORWARD:
      if (s_motion_accepted == 0U)
      {
        Auto_HoldAvoidStop();
        break;
      }
      if (Auto_StateElapsed(AUTO_FORWARD_MS) != 0U)
      {
        if (Auto_PathClear() != 0U)
        {
          Auto_EnterState(AUTO_STATE_RUN);
        }
        else if (Auto_BasicStuckDetected() != 0U)
        {
          Auto_EnterState(AUTO_STATE_BACK);
        }
        else
        {
          Auto_EnterState(AUTO_STATE_BACK);
        }
      }
      break;

    default:
      Auto_EnterState(AUTO_STATE_RUN);
      break;
  }
}
