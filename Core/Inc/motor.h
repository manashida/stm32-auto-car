#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void Motor_Init(void);
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
