#ifndef __AUTO_RUN_H
#define __AUTO_RUN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "car_mode.h"
#include <stdint.h>

extern int16_t g_car_speed;
extern uint16_t g_distance_cm;

void Auto_Init(void);
void Auto_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __AUTO_RUN_H */
