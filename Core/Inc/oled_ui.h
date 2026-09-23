#ifndef __OLED_UI_H
#define __OLED_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern int16_t g_car_speed;
extern uint16_t g_distance_cm;
extern int32_t g_weight_g;

void OLED_UI_Init(void);
void OLED_UI_Refresh(void);
void OLED_UI_Task(void);
void OLED_UI_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_UI_H */
