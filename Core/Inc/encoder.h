#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Mechanical calibration:
 * - ENCODER_PULSE_PER_REV: encoder pulses per motor-shaft revolution.
 * - GEAR_RATIO: motor revolutions per wheel revolution. Use 1 if the encoder
 *   is already mounted on the wheel/output shaft.
 * - WHEEL_DIAMETER_MM: measured tire diameter under load.
 */
#define ENCODER_PULSE_PER_REV       20L
#define GEAR_RATIO                  30L
#define WHEEL_DIAMETER_MM           65L
#define WHEEL_CIRCUMFERENCE_MM      ((WHEEL_DIAMETER_MM * 314L) / 100L)
#define ENCODER_OUTPUT_PULSE_PER_REV (ENCODER_PULSE_PER_REV * GEAR_RATIO)

extern volatile int32_t g_left_encoder_count;
extern volatile int32_t g_right_encoder_count;
extern volatile int32_t g_left_encoder_speed;
extern volatile int32_t g_right_encoder_speed;

void Encoder_Init(void);
void Encoder_Update(void);
int32_t Encoder_GetLeftPulse(void);
int32_t Encoder_GetRightPulse(void);
int32_t Encoder_GetLeftSpeed(void);
int32_t Encoder_GetRightSpeed(void);
int32_t Encoder_GetVelocity_mmps(void);
void Encoder_Reset(void);

void Encoder_EXTI_IRQHandler(void);
void Encoder_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
