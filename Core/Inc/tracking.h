#ifndef __TRACKING_H
#define __TRACKING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define TRACKING_O1_MASK (1U << 0)
#define TRACKING_O2_MASK (1U << 1)
#define TRACKING_O3_MASK (1U << 2)
#define TRACKING_O4_MASK (1U << 3)

void Tracking_Init(void);
void Tracking_Task(void);
void Tracking_ResetController(void);
uint8_t Tracking_ReadRaw(void);
uint8_t Tracking_CalcTankSpeed(uint8_t raw, int16_t *left, int16_t *right, uint8_t *line_valid);
uint8_t Tracking_GetO1(void);
uint8_t Tracking_GetO2(void);
uint8_t Tracking_GetO3(void);
uint8_t Tracking_GetO4(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRACKING_H */
