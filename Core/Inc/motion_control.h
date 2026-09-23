#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  MOTION_OWNER_NONE = 0,
  MOTION_OWNER_TRACKING,
  MOTION_OWNER_AUTO,
  MOTION_OWNER_MANUAL,
  MOTION_OWNER_TRANSPORT,
  MOTION_OWNER_EMERGENCY
} MotionOwner_t;

typedef enum
{
  MOTION_CMD_STOP = 0,
  MOTION_CMD_FORWARD,
  MOTION_CMD_BACKWARD,
  MOTION_CMD_LEFT,
  MOTION_CMD_RIGHT,
  MOTION_CMD_TANK
} MotionCommand_t;

uint8_t Motion_Submit(MotionOwner_t owner,
                      MotionCommand_t command,
                      int16_t speed,
                      uint32_t timeout_ms);
uint8_t Motion_SubmitTank(MotionOwner_t owner,
                          int16_t left_speed,
                          int16_t right_speed,
                          uint32_t timeout_ms);
void Motion_ClearNonEmergency(void);
void Motion_ClearEmergency(void);
uint8_t Motion_IsEmergencyLocked(void);
void Motion_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CONTROL_H */
