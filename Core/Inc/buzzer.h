#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Task(void);

void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Stop(void);

void Buzzer_Beep(uint16_t duration_ms);
void Buzzer_BeepTimes(uint8_t times, uint32_t on_ms, uint32_t off_ms);

void Buzzer_SetAlarm(uint8_t enable);
void Buzzer_SetPattern(uint16_t on_ms, uint16_t off_ms);
void Buzzer_SetAlarmPattern(uint32_t on_ms, uint32_t off_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H */
