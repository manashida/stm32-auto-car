#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define SERVO_MIN_PULSE_US 600U
#define SERVO_MAX_PULSE_US 2400U
#define SERVO_MIN_ANGLE    0U
#define SERVO_MAX_ANGLE    40U

#define SERVO_CLOSE_ANGLE  5U
#define SERVO_OPEN_ANGLE   40U

void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);
void Servo_MoveTo(uint8_t angle, uint16_t duration_ms);
void Servo_Task(void);
uint8_t Servo_IsBusy(void);
uint8_t Servo_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
