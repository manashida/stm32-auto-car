#include "tracking.h"
#include "app_config.h"
#include "auto_run.h"
#include "motion_control.h"
#include "usart.h"
#include <stdio.h>

#define TRACKING_O1_PORT GPIOB
#define TRACKING_O1_PIN  GPIO_PIN_5
#define TRACKING_O2_PORT GPIOB
#define TRACKING_O2_PIN  GPIO_PIN_8
#define TRACKING_O3_PORT GPIOB
#define TRACKING_O3_PIN  GPIO_PIN_9
#define TRACKING_O4_PORT GPIOC
#define TRACKING_O4_PIN  GPIO_PIN_13

#define TRACKING_BLACK_LEVEL           1U
#define TRACKING_MOTOR_ENABLE          1
#define TRACKING_DEBUG_PERIOD_MS       300U

#define TRACKING_Q                     1000
#define TRACKING_FILTER_CONFIRM_COUNT  5U
#define TRACKING_ERROR_IIR_NUM         5
#define TRACKING_ERROR_IIR_DEN         10
#define TRACKING_ERROR_DEADBAND_Q      60

#define TRACKING_MAX_BASE_SPEED        360
#define TRACKING_MIN_BASE_SPEED        180
#define TRACKING_WIDE_LINE_SPEED       160
#define TRACKING_SHARP_BASE_SPEED      120
#define TRACKING_KP                    280
#define TRACKING_KP_SHARP              760
#define TRACKING_SHARP_ERROR_Q         300
#define TRACKING_SHARP_MIN_TURN        520
#define TRACKING_CURVE_CENTER_OFFSET_Q 333
#define TRACKING_CURVE_KEEP_TURN       180
#define TRACKING_SEARCH_SPEED          210
#define TRACKING_SEARCH_INIT_SPEED     80
#define TRACKING_MAX_COMMAND_SPEED     1000
#define TRACKING_TURN_SLEW_PER_MS      16

typedef enum
{
  TRACKING_ACTION_TRACK = 0,
  TRACKING_ACTION_SEARCH,
  TRACKING_ACTION_WIDE_LINE,
  TRACKING_ACTION_SHARP_TURN
} TrackingAction_t;

static uint8_t s_filtered_raw = 0;
static uint8_t s_raw_candidate = 0;
static uint8_t s_raw_candidate_count = 0;
static uint8_t s_filter_ready = 0;

static int16_t s_error_q = 0;
static int16_t s_last_line_error_q = 0;
static uint8_t s_error_filter_ready = 0;

static int16_t s_last_left_speed = 0;
static int16_t s_last_right_speed = 0;
static int16_t s_last_turn = 0;
static int16_t s_last_base_speed = 0;
static uint32_t s_last_turn_tick = 0;
static uint8_t s_turn_slew_ready = 0;
static TrackingAction_t s_last_action = TRACKING_ACTION_TRACK;

#if TRACKING_DEBUG_ENABLE
static uint32_t s_debug_last_tick = 0;
#endif

static void Tracking_ResetControllerState(void)
{
  s_error_q = 0;
  s_last_line_error_q = 0;
  s_error_filter_ready = 0U;
  s_last_left_speed = 0;
  s_last_right_speed = 0;
  s_last_turn = 0;
  s_last_base_speed = 0;
  s_last_turn_tick = 0U;
  s_turn_slew_ready = 0U;
  s_last_action = TRACKING_ACTION_TRACK;
}

static uint8_t Tracking_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t Tracking_IsBlack(uint8_t value)
{
  return (value == TRACKING_BLACK_LEVEL) ? 1U : 0U;
}

static uint8_t Tracking_IsRawBlack(uint8_t raw, uint8_t mask)
{
  return Tracking_IsBlack((raw & mask) ? 1U : 0U);
}

static int16_t Tracking_Abs16(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static int16_t Tracking_ClampCommand(int16_t speed)
{
  if (speed > TRACKING_MAX_COMMAND_SPEED)
  {
    return TRACKING_MAX_COMMAND_SPEED;
  }
  if (speed < -TRACKING_MAX_COMMAND_SPEED)
  {
    return (int16_t)-TRACKING_MAX_COMMAND_SPEED;
  }
  return speed;
}

static uint8_t Tracking_ReadFilteredRaw(void)
{
  uint8_t raw = Tracking_ReadRaw();

  if (s_filter_ready == 0U)
  {
    s_filtered_raw = raw;
    s_raw_candidate = raw;
    s_raw_candidate_count = TRACKING_FILTER_CONFIRM_COUNT;
    s_filter_ready = 1U;
    return s_filtered_raw;
  }

  if (raw == s_raw_candidate)
  {
    if (s_raw_candidate_count < TRACKING_FILTER_CONFIRM_COUNT)
    {
      s_raw_candidate_count++;
    }
  }
  else
  {
    s_raw_candidate = raw;
    s_raw_candidate_count = 1U;
  }

  if (s_raw_candidate_count >= TRACKING_FILTER_CONFIRM_COUNT)
  {
    s_filtered_raw = s_raw_candidate;
  }

  return s_filtered_raw;
}

static uint8_t Tracking_CalcNormalizedError(uint8_t raw, int16_t *error_q)
{
  int16_t weighted_sum = 0;
  uint8_t count = 0;

  if (Tracking_IsRawBlack(raw, TRACKING_O1_MASK) != 0U)
  {
    weighted_sum = (int16_t)(weighted_sum + 3);
    count++;
  }
  if (Tracking_IsRawBlack(raw, TRACKING_O2_MASK) != 0U)
  {
    weighted_sum = (int16_t)(weighted_sum + 1);
    count++;
  }
  if (Tracking_IsRawBlack(raw, TRACKING_O3_MASK) != 0U)
  {
    weighted_sum = (int16_t)(weighted_sum - 1);
    count++;
  }
  if (Tracking_IsRawBlack(raw, TRACKING_O4_MASK) != 0U)
  {
    weighted_sum = (int16_t)(weighted_sum - 3);
    count++;
  }

  if (count == 0U)
  {
    return 0U;
  }

  *error_q = (int16_t)(((int32_t)weighted_sum * TRACKING_Q) / ((int32_t)count * 3));
  return 1U;
}

static uint8_t Tracking_CountActive(uint8_t raw)
{
  uint8_t count = 0;

  for (uint8_t mask = TRACKING_O1_MASK; mask <= TRACKING_O4_MASK; mask <<= 1U)
  {
    if (Tracking_IsRawBlack(raw, mask) != 0U)
    {
      count++;
    }
  }

  return count;
}

static uint8_t Tracking_IsWideOrAmbiguous(uint8_t raw)
{
  uint8_t count = Tracking_CountActive(raw);

  if (count >= 4U)
  {
    return 1U;
  }

  if ((Tracking_IsRawBlack(raw, TRACKING_O1_MASK) != 0U) &&
      (Tracking_IsRawBlack(raw, TRACKING_O4_MASK) != 0U))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t Tracking_IsSharpTurnPattern(uint8_t raw, int16_t *error_q)
{
  uint8_t count = Tracking_CountActive(raw);

  if (count < 3U)
  {
    return 0U;
  }

  if ((Tracking_IsRawBlack(raw, TRACKING_O1_MASK) != 0U) &&
      (Tracking_IsRawBlack(raw, TRACKING_O4_MASK) == 0U))
  {
    *error_q = TRACKING_Q;
    return 1U;
  }

  if ((Tracking_IsRawBlack(raw, TRACKING_O4_MASK) != 0U) &&
      (Tracking_IsRawBlack(raw, TRACKING_O1_MASK) == 0U))
  {
    *error_q = (int16_t)-TRACKING_Q;
    return 1U;
  }

  return 0U;
}

static int16_t Tracking_FilterError(int16_t measured_error_q)
{
  int32_t filtered = 0;

  if (s_error_filter_ready == 0U)
  {
    s_error_filter_ready = 1U;
    s_error_q = measured_error_q;
    return s_error_q;
  }

  filtered = (int32_t)s_error_q +
             (((int32_t)TRACKING_ERROR_IIR_NUM *
               ((int32_t)measured_error_q - (int32_t)s_error_q)) /
              TRACKING_ERROR_IIR_DEN);

  s_error_q = (int16_t)filtered;
  return s_error_q;
}

static int16_t Tracking_ApplyErrorDeadband(int16_t error_q)
{
  if (Tracking_Abs16(error_q) <= TRACKING_ERROR_DEADBAND_Q)
  {
    return 0;
  }

  return error_q;
}

static int16_t Tracking_CalcBaseSpeed(int16_t error_q)
{
  int16_t abs_error = Tracking_Abs16(error_q);
  int16_t range = (int16_t)(TRACKING_MAX_BASE_SPEED - TRACKING_MIN_BASE_SPEED);
  int16_t reduction = (int16_t)((((int32_t)range * abs_error) * abs_error) /
                                ((int32_t)TRACKING_Q * TRACKING_Q));

  return (int16_t)(TRACKING_MAX_BASE_SPEED - reduction);
}

static int16_t Tracking_CalcTurn(int16_t error_q)
{
  int16_t abs_error = Tracking_Abs16(error_q);
  int16_t gain = (abs_error >= TRACKING_SHARP_ERROR_Q) ? TRACKING_KP_SHARP : TRACKING_KP;
  int16_t turn = (int16_t)(((int32_t)gain * error_q) / TRACKING_Q);

  if ((abs_error >= TRACKING_SHARP_ERROR_Q) &&
      (Tracking_Abs16(turn) < TRACKING_SHARP_MIN_TURN))
  {
    turn = (error_q > 0) ? TRACKING_SHARP_MIN_TURN : (int16_t)-TRACKING_SHARP_MIN_TURN;
  }

  return turn;
}

static int16_t Tracking_ApplyCurveCenterOffset(int16_t error_q,
                                               int16_t *feedforward_turn)
{
  int16_t target_error_q = 0;

  *feedforward_turn = 0;

  if ((s_last_turn >= TRACKING_CURVE_KEEP_TURN) &&
      (error_q < (int16_t)-TRACKING_ERROR_DEADBAND_Q))
  {
    target_error_q = (int16_t)-TRACKING_CURVE_CENTER_OFFSET_Q;
    *feedforward_turn = TRACKING_CURVE_KEEP_TURN;
  }
  else if ((s_last_turn <= (int16_t)-TRACKING_CURVE_KEEP_TURN) &&
           (error_q > TRACKING_ERROR_DEADBAND_Q))
  {
    target_error_q = TRACKING_CURVE_CENTER_OFFSET_Q;
    *feedforward_turn = (int16_t)-TRACKING_CURVE_KEEP_TURN;
  }
  else
  {
    return error_q;
  }

  return (int16_t)(error_q - target_error_q);
}

static int16_t Tracking_ApplyTurnSlew(int16_t target_turn)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = 0;
  int16_t delta = 0;
  int16_t max_delta = 0;

  if (s_turn_slew_ready == 0U)
  {
    s_turn_slew_ready = 1U;
    s_last_turn_tick = now;
    s_last_turn = target_turn;
    return s_last_turn;
  }

  elapsed = (uint32_t)(now - s_last_turn_tick);
  if (elapsed == 0U)
  {
    return s_last_turn;
  }

  s_last_turn_tick = now;
  max_delta = (int16_t)((uint32_t)TRACKING_TURN_SLEW_PER_MS * elapsed);
  delta = (int16_t)(target_turn - s_last_turn);

  if (delta > max_delta)
  {
    s_last_turn = (int16_t)(s_last_turn + max_delta);
  }
  else if (delta < -max_delta)
  {
    s_last_turn = (int16_t)(s_last_turn - max_delta);
  }
  else
  {
    s_last_turn = target_turn;
  }

  return s_last_turn;
}

static void Tracking_SetMotor(int16_t left_speed, int16_t right_speed)
{
  s_last_left_speed = left_speed;
  s_last_right_speed = right_speed;

#if TRACKING_MOTOR_ENABLE
  (void)Motion_SubmitTank(MOTION_OWNER_TRACKING, left_speed, right_speed, 0U);
#else
  (void)left_speed;
  (void)right_speed;
#endif
}

void Tracking_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* This vehicle's board is mounted with O1/O2 on the right side and O3/O4 on
   * the left side, so the position weights above intentionally use that order.
   */
  GPIO_InitStruct.Pin = TRACKING_O1_PIN |
                        TRACKING_O2_PIN |
                        TRACKING_O3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TRACKING_O4_PIN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  Tracking_ResetControllerState();
}

void Tracking_ResetController(void)
{
  Tracking_ResetControllerState();
}

uint8_t Tracking_ReadRaw(void)
{
  uint8_t raw = 0;

  raw |= (uint8_t)(Tracking_GetO1() << 0);
  raw |= (uint8_t)(Tracking_GetO2() << 1);
  raw |= (uint8_t)(Tracking_GetO3() << 2);
  raw |= (uint8_t)(Tracking_GetO4() << 3);

  return raw;
}

uint8_t Tracking_CalcTankSpeed(uint8_t raw, int16_t *left, int16_t *right, uint8_t *line_valid)
{
  int16_t measured_error_q = 0;
  int16_t control_error_q = 0;
  int16_t base_speed = 0;
  int16_t target_turn = 0;
  int16_t curve_feedforward_turn = 0;
  int16_t turn = 0;
  int16_t left_speed = 0;
  int16_t right_speed = 0;
  uint8_t line_found = 0U;
  uint8_t sharp_turn = 0U;
  TrackingAction_t action = TRACKING_ACTION_TRACK;

  if ((left == NULL) || (right == NULL) || (line_valid == NULL))
  {
    return 0U;
  }

  line_found = Tracking_CalcNormalizedError(raw, &measured_error_q);
  sharp_turn = Tracking_IsSharpTurnPattern(raw, &measured_error_q);
  *line_valid = line_found;

  if ((line_found != 0U) && (sharp_turn != 0U))
  {
    action = TRACKING_ACTION_SHARP_TURN;
    control_error_q = measured_error_q;
    s_error_q = control_error_q;
    s_error_filter_ready = 0U;
    s_last_line_error_q = control_error_q;

    base_speed = TRACKING_SHARP_BASE_SPEED;
    target_turn = Tracking_CalcTurn(control_error_q);
    turn = Tracking_ApplyTurnSlew(target_turn);

    left_speed = Tracking_ClampCommand((int16_t)(base_speed + turn));
    right_speed = Tracking_ClampCommand((int16_t)(base_speed - turn));
  }
  else if ((line_found != 0U) && (Tracking_IsWideOrAmbiguous(raw) != 0U))
  {
    action = TRACKING_ACTION_WIDE_LINE;
    control_error_q = 0;
    s_error_q = 0;
    base_speed = TRACKING_WIDE_LINE_SPEED;
    turn = Tracking_ApplyTurnSlew(0);
    left_speed = Tracking_ClampCommand((int16_t)(base_speed + turn));
    right_speed = Tracking_ClampCommand((int16_t)(base_speed - turn));
  }
  else if (line_found != 0U)
  {
    action = TRACKING_ACTION_TRACK;
    control_error_q = Tracking_FilterError(measured_error_q);
    control_error_q = Tracking_ApplyCurveCenterOffset(control_error_q,
                                                      &curve_feedforward_turn);
    control_error_q = Tracking_ApplyErrorDeadband(control_error_q);
    s_error_q = control_error_q;

    if (control_error_q != 0)
    {
      s_last_line_error_q = control_error_q;
    }

    base_speed = Tracking_CalcBaseSpeed(control_error_q);
    target_turn = Tracking_CalcTurn(control_error_q);
    target_turn = Tracking_ClampCommand((int16_t)(target_turn +
                                                 curve_feedforward_turn));
    turn = Tracking_ApplyTurnSlew(target_turn);

    left_speed = Tracking_ClampCommand((int16_t)(base_speed + turn));
    right_speed = Tracking_ClampCommand((int16_t)(base_speed - turn));
  }
  else
  {
    action = TRACKING_ACTION_SEARCH;
    s_error_filter_ready = 0U;
    base_speed = 0;

    if (s_last_line_error_q < 0)
    {
      target_turn = (int16_t)-TRACKING_SEARCH_SPEED;
    }
    else if (s_last_line_error_q > 0)
    {
      target_turn = TRACKING_SEARCH_SPEED;
    }
    else
    {
      target_turn = (int16_t)-TRACKING_SEARCH_INIT_SPEED;
    }

    turn = Tracking_ApplyTurnSlew(target_turn);
    left_speed = turn;
    right_speed = (int16_t)-turn;
  }

  s_last_turn = turn;
  s_last_base_speed = base_speed;
  s_last_action = action;
  *left = left_speed;
  *right = right_speed;

  return 1U;
}

void Tracking_Task(void)
{
  uint8_t raw = Tracking_ReadFilteredRaw();
  int16_t left_speed = 0;
  int16_t right_speed = 0;
  uint8_t line_valid = 0U;

  if (g_car_mode != MODE_TRACK)
  {
    Tracking_ResetControllerState();
    return;
  }

  if (Tracking_CalcTankSpeed(raw, &left_speed, &right_speed, &line_valid) == 0U)
  {
    return;
  }

  Tracking_SetMotor(left_speed, right_speed);

#if TRACKING_DEBUG_ENABLE
  {
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_debug_last_tick) >= TRACKING_DEBUG_PERIOD_MS)
    {
      char text[128];
      const char *action_name = "wide";
      int len;

      switch (s_last_action)
      {
        case TRACKING_ACTION_TRACK:
          action_name = "track";
          break;

        case TRACKING_ACTION_SEARCH:
          action_name = "search";
          break;

        case TRACKING_ACTION_SHARP_TURN:
          action_name = "sharp";
          break;

        case TRACKING_ACTION_WIDE_LINE:
        default:
          action_name = "wide";
          break;
      }

      s_debug_last_tick = now;
      len = snprintf(text, sizeof(text),
                     "TRK O1=%u O2=%u O3=%u O4=%u raw=0x%02X err=%d action=%s L=%d R=%d\r\n",
                     (unsigned)((raw & TRACKING_O1_MASK) ? 1U : 0U),
                     (unsigned)((raw & TRACKING_O2_MASK) ? 1U : 0U),
                     (unsigned)((raw & TRACKING_O3_MASK) ? 1U : 0U),
                     (unsigned)((raw & TRACKING_O4_MASK) ? 1U : 0U),
                     (unsigned)raw,
                     (int)s_error_q,
                     action_name,
                     (int)s_last_left_speed,
                     (int)s_last_right_speed);
      if (len > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 10);
      }
    }
  }
#endif
}

uint8_t Tracking_GetO1(void)
{
  return Tracking_ReadPin(TRACKING_O1_PORT, TRACKING_O1_PIN);
}

uint8_t Tracking_GetO2(void)
{
  return Tracking_ReadPin(TRACKING_O2_PORT, TRACKING_O2_PIN);
}

uint8_t Tracking_GetO3(void)
{
  return Tracking_ReadPin(TRACKING_O3_PORT, TRACKING_O3_PIN);
}

uint8_t Tracking_GetO4(void)
{
  return Tracking_ReadPin(TRACKING_O4_PORT, TRACKING_O4_PIN);
}
