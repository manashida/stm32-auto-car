#include "servo.h"
#include "tim.h"

#define SERVO_PWM_FULL_SCALE_ANGLE 180U
#define SERVO_DEFAULT_OPEN_MS      1500U
#define SERVO_DEFAULT_CLOSE_MS     1200U

static uint8_t s_servo_inited = 0U;
static uint8_t s_current_angle = SERVO_CLOSE_ANGLE;
static uint8_t s_start_angle = SERVO_CLOSE_ANGLE;
static uint8_t s_target_angle = SERVO_CLOSE_ANGLE;
static uint32_t s_move_start_tick = 0U;
static uint16_t s_move_duration_ms = 0U;
static uint8_t s_servo_busy = 0U;

static uint8_t Servo_LimitAngle(uint8_t angle)
{
#if SERVO_MIN_ANGLE > 0U
  if (angle < SERVO_MIN_ANGLE)
  {
    angle = SERVO_MIN_ANGLE;
  }
#endif

  if (angle > SERVO_MAX_ANGLE)
  {
    angle = SERVO_MAX_ANGLE;
  }

  return angle;
}

static uint16_t Servo_AngleToPulse(uint8_t angle)
{
  angle = Servo_LimitAngle(angle);
  return (uint16_t)(SERVO_MIN_PULSE_US +
                    (((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_PWM_FULL_SCALE_ANGLE));
}

static void Servo_ApplyAngle(uint8_t angle)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Servo_AngleToPulse(angle));
}

static uint16_t Servo_GetDefaultDuration(uint8_t angle)
{
  uint8_t limited_angle = Servo_LimitAngle(angle);
  uint8_t angle_delta;

  if (limited_angle == SERVO_OPEN_ANGLE)
  {
    return SERVO_DEFAULT_OPEN_MS;
  }

  if (limited_angle == SERVO_CLOSE_ANGLE)
  {
    return SERVO_DEFAULT_CLOSE_MS;
  }

  angle_delta = (limited_angle > s_current_angle) ?
                (limited_angle - s_current_angle) :
                (s_current_angle - limited_angle);

  if (angle_delta == 0U)
  {
    return 0U;
  }

  return (uint16_t)(((uint32_t)SERVO_DEFAULT_OPEN_MS * (uint32_t)angle_delta) / SERVO_OPEN_ANGLE);
}

void Servo_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  s_servo_inited = 1U;
  s_current_angle = Servo_LimitAngle(SERVO_CLOSE_ANGLE);
  s_start_angle = s_current_angle;
  s_target_angle = s_current_angle;
  s_move_start_tick = HAL_GetTick();
  s_move_duration_ms = 0U;
  s_servo_busy = 0U;
  Servo_ApplyAngle(s_current_angle);
}

void Servo_SetAngle(uint8_t angle)
{
  Servo_MoveTo(angle, Servo_GetDefaultDuration(angle));
}

void Servo_MoveTo(uint8_t angle, uint16_t duration_ms)
{
  uint8_t target_angle = Servo_LimitAngle(angle);

  if (s_servo_inited == 0U)
  {
    return;
  }

  if ((s_servo_busy != 0U) && (target_angle == s_target_angle))
  {
    return;
  }

  if (target_angle == s_current_angle)
  {
    s_start_angle = s_current_angle;
    s_target_angle = s_current_angle;
    s_move_duration_ms = 0U;
    s_servo_busy = 0U;
    return;
  }

  s_start_angle = s_current_angle;
  s_target_angle = target_angle;
  s_move_start_tick = HAL_GetTick();
  s_move_duration_ms = duration_ms;

  if (duration_ms == 0U)
  {
    s_current_angle = s_target_angle;
    s_servo_busy = 0U;
    Servo_ApplyAngle(s_current_angle);
    return;
  }

  s_servo_busy = 1U;
}

void Servo_Task(void)
{
  uint32_t elapsed;
  int16_t angle_delta;
  int16_t interpolated_angle;

  if ((s_servo_inited == 0U) || (s_servo_busy == 0U))
  {
    return;
  }

  elapsed = HAL_GetTick() - s_move_start_tick;
  if (elapsed >= s_move_duration_ms)
  {
    s_current_angle = s_target_angle;
    s_servo_busy = 0U;
    Servo_ApplyAngle(s_current_angle);
    return;
  }

  angle_delta = (int16_t)s_target_angle - (int16_t)s_start_angle;
  interpolated_angle = (int16_t)s_start_angle +
                       (int16_t)(((int32_t)angle_delta * (int32_t)elapsed) / (int32_t)s_move_duration_ms);

  if (interpolated_angle < (int16_t)SERVO_MIN_ANGLE)
  {
    interpolated_angle = (int16_t)SERVO_MIN_ANGLE;
  }
  else if (interpolated_angle > (int16_t)SERVO_MAX_ANGLE)
  {
    interpolated_angle = (int16_t)SERVO_MAX_ANGLE;
  }

  if ((uint8_t)interpolated_angle != s_current_angle)
  {
    s_current_angle = (uint8_t)interpolated_angle;
    Servo_ApplyAngle(s_current_angle);
  }
}

uint8_t Servo_IsBusy(void)
{
  return s_servo_busy;
}

uint8_t Servo_GetAngle(void)
{
  return s_current_angle;
}
