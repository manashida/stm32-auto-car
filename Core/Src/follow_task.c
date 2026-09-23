#include "follow_task.h"
#include "app_config.h"
#include "car_mode.h"
#include "motion_control.h"
#include "usart.h"
#include "uwb_follow.h"
#include <stdio.h>

#define FOLLOW_MOTION_TTL_MS 250U

typedef enum
{
  FOLLOW_STATE_IDLE = 0,
  FOLLOW_STATE_NO_TARGET,
  FOLLOW_STATE_TIMEOUT,
  FOLLOW_STATE_LOW_CONFIDENCE,
  FOLLOW_STATE_TOO_CLOSE,
  FOLLOW_STATE_HOLD,
  FOLLOW_STATE_TRACK
} FollowState_t;

extern volatile CarMode_t g_car_mode;

static int16_t s_left_speed = 0;
static int16_t s_right_speed = 0;
static uint8_t s_was_active = 0U;
static FollowState_t s_state = FOLLOW_STATE_IDLE;

#if FOLLOW_DEBUG_ENABLE
static uint32_t s_debug_last_tick = 0;
#endif

static int16_t Follow_Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static int16_t Follow_Clamp(int16_t value, int16_t min_value, int16_t max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

static int16_t Follow_ApplySlew(int16_t current, int16_t target)
{
  int16_t delta = (int16_t)(target - current);

  if (delta > FOLLOW_SPEED_STEP)
  {
    return (int16_t)(current + FOLLOW_SPEED_STEP);
  }

  if (delta < -FOLLOW_SPEED_STEP)
  {
    return (int16_t)(current - FOLLOW_SPEED_STEP);
  }

  return target;
}

static int16_t Follow_CalcBaseSpeed(uint32_t distance_cm)
{
  uint32_t start_cm = FOLLOW_TARGET_DISTANCE_CM + FOLLOW_DISTANCE_DEADBAND_CM;
  uint32_t error_cm = 0;
  int32_t speed = 0;

  if (distance_cm <= start_cm)
  {
    return 0;
  }

  error_cm = distance_cm - start_cm;
  speed = (int32_t)error_cm * FOLLOW_DISTANCE_GAIN;
  if (speed > FOLLOW_MAX_SPEED)
  {
    speed = FOLLOW_MAX_SPEED;
  }

  return (int16_t)speed;
}

static int16_t Follow_CalcTurn(int16_t angle_deg)
{
  int32_t turn = 0;

  if (Follow_Abs16(angle_deg) <= FOLLOW_ANGLE_DEADBAND_DEG)
  {
    return 0;
  }

  turn = (int32_t)angle_deg * FOLLOW_TURN_GAIN;
  return Follow_Clamp((int16_t)turn, (int16_t)-FOLLOW_MAX_TURN, FOLLOW_MAX_TURN);
}

static void Follow_SubmitStop(FollowState_t state)
{
  s_state = state;
  s_left_speed = 0;
  s_right_speed = 0;
  (void)Motion_Submit(MOTION_OWNER_AUTO, MOTION_CMD_STOP, 0, FOLLOW_MOTION_TTL_MS);
}

static void Follow_SubmitTank(int16_t target_left, int16_t target_right)
{
  s_left_speed = Follow_ApplySlew(s_left_speed, target_left);
  s_right_speed = Follow_ApplySlew(s_right_speed, target_right);
  (void)Motion_SubmitTank(MOTION_OWNER_AUTO, s_left_speed, s_right_speed, FOLLOW_MOTION_TTL_MS);
}

static void Follow_DebugPrint(const UwbFollow_Data_t *data, uint32_t age_ms)
{
#if FOLLOW_DEBUG_ENABLE
  uint32_t now = HAL_GetTick();
  const char *state_name = "idle";

  switch (s_state)
  {
    case FOLLOW_STATE_NO_TARGET:
      state_name = "no_target";
      break;

    case FOLLOW_STATE_TIMEOUT:
      state_name = "timeout";
      break;

    case FOLLOW_STATE_LOW_CONFIDENCE:
      state_name = "low_rssi";
      break;

    case FOLLOW_STATE_TOO_CLOSE:
      state_name = "too_close";
      break;

    case FOLLOW_STATE_HOLD:
      state_name = "hold";
      break;

    case FOLLOW_STATE_TRACK:
      state_name = "track";
      break;

    case FOLLOW_STATE_IDLE:
    default:
      state_name = "idle";
      break;
  }

  if ((uint32_t)(now - s_debug_last_tick) >= FOLLOW_DEBUG_PERIOD_MS)
  {
    char text[128];
    int len;
    uint32_t distance = 0U;
    int16_t angle = 0;
    int16_t rssi_dbm = -128;

    s_debug_last_tick = now;

    if (data != NULL)
    {
      distance = data->distance_cm;
      angle = data->angle_deg;
      rssi_dbm = data->rssi_dbm;
    }

    len = snprintf(text, sizeof(text),
                   "FOLLOW state=%s dist=%lu angle=%d rssi=%d age=%lu L=%d R=%d\r\n",
                   state_name,
                   (unsigned long)distance,
                   (int)angle,
                   (int)rssi_dbm,
                   (unsigned long)age_ms,
                   (int)s_left_speed,
                   (int)s_right_speed);
    if (len > 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 10);
    }
  }
#else
  (void)data;
  (void)age_ms;
#endif
}

void Follow_Init(void)
{
  Follow_Reset();
}

void Follow_Reset(void)
{
  s_left_speed = 0;
  s_right_speed = 0;
  s_was_active = 0U;
  s_state = FOLLOW_STATE_IDLE;
}

void Follow_Task(void)
{
  UwbFollow_Data_t data;
  uint32_t now = HAL_GetTick();
  uint32_t age_ms = 0xFFFFFFFFU;
  int16_t base_speed = 0;
  int16_t turn = 0;
  int16_t target_left = 0;
  int16_t target_right = 0;

  if (g_car_mode != MODE_FOLLOW)
  {
    if (s_was_active != 0U)
    {
      Follow_Reset();
    }
    return;
  }

  s_was_active = 1U;

  if (UwbFollow_GetLatest(&data) == 0U)
  {
    Follow_SubmitStop(FOLLOW_STATE_NO_TARGET);
    Follow_DebugPrint(NULL, age_ms);
    return;
  }

  age_ms = (uint32_t)(now - data.update_tick);

  if (age_ms > FOLLOW_LOST_TIMEOUT_MS)
  {
    Follow_SubmitStop(FOLLOW_STATE_TIMEOUT);
    Follow_DebugPrint(&data, age_ms);
    return;
  }

  if (data.rssi_dbm < FOLLOW_MIN_RSSI_DBM)
  {
    Follow_SubmitStop(FOLLOW_STATE_LOW_CONFIDENCE);
    Follow_DebugPrint(&data, age_ms);
    return;
  }

  if (data.distance_cm < FOLLOW_NEAR_STOP_CM)
  {
    Follow_SubmitStop(FOLLOW_STATE_TOO_CLOSE);
    Follow_DebugPrint(&data, age_ms);
    return;
  }

  base_speed = Follow_CalcBaseSpeed(data.distance_cm);
  turn = Follow_CalcTurn(data.angle_deg);

  if ((base_speed == 0) && (turn == 0))
  {
    Follow_SubmitStop(FOLLOW_STATE_HOLD);
    Follow_DebugPrint(&data, age_ms);
    return;
  }

  s_state = FOLLOW_STATE_TRACK;
  target_left = Follow_Clamp((int16_t)(base_speed + turn),
                             (int16_t)-FOLLOW_MAX_SPEED,
                             FOLLOW_MAX_SPEED);
  target_right = Follow_Clamp((int16_t)(base_speed - turn),
                              (int16_t)-FOLLOW_MAX_SPEED,
                              FOLLOW_MAX_SPEED);
  Follow_SubmitTank(target_left, target_right);
  Follow_DebugPrint(&data, age_ms);
}
