#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ULTRASONIC_TIMEOUT_CM 999U

void Ultrasonic_Init(void);
void Ultrasonic_Start(void);
void Ultrasonic_Task(void);
uint8_t Ultrasonic_IsBusy(void);
uint8_t Ultrasonic_IsDistanceValid(uint16_t distance_cm);
uint8_t Ultrasonic_TakeDistanceCm(uint16_t *distance_cm);
uint16_t Ultrasonic_GetDistanceCm(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H */
