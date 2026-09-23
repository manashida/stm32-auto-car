#include "motion_control.h"
#include "car.h"
#include "main.h"
#include "motor.h"

extern int16_t g_car_speed;

typedef struct
{
  MotionOwner_t owner;
  MotionCommand_t command;
  int16_t speed;
  uint32_t expire_tick;
  uint8_t active;
  uint8_t has_timeout;
} MotionControlState_t;

static MotionControlState_t s_motion = {
  MOTION_OWNER_NONE,
  MOTION_CMD_STOP,
  0,
  0U,
  0U,
  0U
};

static uint8_t Motion_GetPriority(MotionOwner_t owner)
{
  switch (owner)
  {
    case MOTION_OWNER_EMERGENCY:
      return 100U;
    case MOTION_OWNER_TRANSPORT:
      return 80U;
    case MOTION_OWNER_MANUAL:
      return 60U;
    case MOTION_OWNER_AUTO:
      return 40U;
    case MOTION_OWNER_TRACKING:
      return 20U;
    case MOTION_OWNER_NONE:
    default:
      return 0U;
  }
}

static int16_t Motion_NormalizeSpeed(int16_t speed)
{
  return (speed < 0) ? (int16_t)-speed : speed;
}

static int16_t Motion_AverageAbsSpeed(int16_t left_speed, int16_t right_speed)
{
  return (int16_t)((Motion_NormalizeSpeed(left_speed) +
                    Motion_NormalizeSpeed(right_speed)) / 2);
}

static void Motion_Apply(MotionCommand_t command, int16_t speed)
{
  int16_t run_speed = Motion_NormalizeSpeed(speed);

  switch (command)
  {
    case MOTION_CMD_FORWARD:
      g_car_speed = run_speed;
      Car_Forward(run_speed);
      break;

    case MOTION_CMD_BACKWARD:
      g_car_speed = run_speed;
      Car_Backward(run_speed);
      break;

    case MOTION_CMD_LEFT:
      g_car_speed = run_speed;
      Car_Left(run_speed);
      break;

    case MOTION_CMD_RIGHT:
      g_car_speed = run_speed;
      Car_Right(run_speed);
      break;

    case MOTION_CMD_STOP:
    default:
      g_car_speed = 0;
      Car_Stop();
      break;
  }
}

static void Motion_ApplyTank(int16_t left_speed, int16_t right_speed)
{
  g_car_speed = Motion_AverageAbsSpeed(left_speed, right_speed);
  Motor_SetLeftSpeed(left_speed);
  Motor_SetRightSpeed(right_speed);
}

static uint8_t Motion_CanAccept(MotionOwner_t owner)
{
  if (owner == MOTION_OWNER_NONE)
  {
    return 0U;
  }

  if (s_motion.active == 0U)
  {
    return 1U;
  }

  if (s_motion.owner == owner)
  {
    return 1U;
  }

  return (Motion_GetPriority(owner) > Motion_GetPriority(s_motion.owner)) ? 1U : 0U;
}

static void Motion_ClearOwner(void)
{
  s_motion.owner = MOTION_OWNER_NONE;
  s_motion.command = MOTION_CMD_STOP;
  s_motion.speed = 0;
  s_motion.expire_tick = 0U;
  s_motion.active = 0U;
  s_motion.has_timeout = 0U;
}

static void Motion_LockEmergencyStop(void)
{
  s_motion.owner = MOTION_OWNER_EMERGENCY;
  s_motion.command = MOTION_CMD_STOP;
  s_motion.speed = 0;
  s_motion.expire_tick = 0U;
  s_motion.active = 1U;
  s_motion.has_timeout = 0U;
}

void Motion_ClearNonEmergency(void)
{
  if ((s_motion.active != 0U) && (s_motion.owner != MOTION_OWNER_EMERGENCY))
  {
    Motion_Apply(MOTION_CMD_STOP, 0);
    Motion_ClearOwner();
  }
}

void Motion_ClearEmergency(void)
{
  if ((s_motion.active != 0U) && (s_motion.owner == MOTION_OWNER_EMERGENCY))
  {
    Motion_Apply(MOTION_CMD_STOP, 0);
    Motion_ClearOwner();
  }
}

uint8_t Motion_IsEmergencyLocked(void)
{
  return ((s_motion.active != 0U) &&
          (s_motion.owner == MOTION_OWNER_EMERGENCY)) ? 1U : 0U;
}

uint8_t Motion_Submit(MotionOwner_t owner,
                      MotionCommand_t command,
                      int16_t speed,
                      uint32_t timeout_ms)
{
  uint32_t now = HAL_GetTick();

  if (Motion_CanAccept(owner) == 0U)
  {
    return 0U;
  }

  s_motion.owner = owner;
  s_motion.command = command;
  s_motion.speed = speed;
  s_motion.active = 1U;

  if (timeout_ms > 0U)
  {
    s_motion.expire_tick = now + timeout_ms;
    s_motion.has_timeout = 1U;
  }
  else
  {
    s_motion.expire_tick = 0U;
    s_motion.has_timeout = 0U;
  }

  Motion_Apply(command, speed);

  if (command == MOTION_CMD_STOP)
  {
    if (owner == MOTION_OWNER_EMERGENCY)
    {
      Motion_LockEmergencyStop();
    }
    else
    {
      Motion_ClearOwner();
    }
  }

  return 1U;
}

uint8_t Motion_SubmitTank(MotionOwner_t owner,
                          int16_t left_speed,
                          int16_t right_speed,
                          uint32_t timeout_ms)
{
  uint32_t now = HAL_GetTick();

  if ((left_speed == 0) && (right_speed == 0))
  {
    return Motion_Submit(owner, MOTION_CMD_STOP, 0, timeout_ms);
  }

  if (Motion_CanAccept(owner) == 0U)
  {
    return 0U;
  }

  s_motion.owner = owner;
  s_motion.command = MOTION_CMD_TANK;
  s_motion.speed = Motion_AverageAbsSpeed(left_speed, right_speed);
  s_motion.active = 1U;

  if (timeout_ms > 0U)
  {
    s_motion.expire_tick = now + timeout_ms;
    s_motion.has_timeout = 1U;
  }
  else
  {
    s_motion.expire_tick = 0U;
    s_motion.has_timeout = 0U;
  }

  Motion_ApplyTank(left_speed, right_speed);
  return 1U;
}

void Motion_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((s_motion.active != 0U) &&
      (s_motion.has_timeout != 0U) &&
      ((uint32_t)(now - s_motion.expire_tick) < 0x80000000UL))
  {
    Motion_Apply(MOTION_CMD_STOP, 0);

    if (s_motion.owner == MOTION_OWNER_EMERGENCY)
    {
      Motion_LockEmergencyStop();
    }
    else
    {
      Motion_ClearOwner();
    }
  }
}
