#ifndef __APP_HEALTH_H
#define __APP_HEALTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  APP_HEALTH_TASK_APP_MODE = 0,
  APP_HEALTH_TASK_BLUETOOTH,
  APP_HEALTH_TASK_LED,
  APP_HEALTH_TASK_BUZZER,
  APP_HEALTH_TASK_ULTRASONIC,
  APP_HEALTH_TASK_ENCODER,
  APP_HEALTH_TASK_WEIGHT,
  APP_HEALTH_TASK_UWB,
  APP_HEALTH_TASK_FOLLOW,
  APP_HEALTH_TASK_MODE_CONTROL,
  APP_HEALTH_TASK_SERVO,
  APP_HEALTH_TASK_MOTION,
  APP_HEALTH_TASK_OLED,
  APP_HEALTH_TASK_DEBUG,
  APP_HEALTH_TASK_COUNT
} AppHealthTask_t;

typedef struct
{
  uint32_t loop_count;
  uint32_t last_loop_ms;
  uint32_t max_loop_ms;
  uint32_t loop_overrun_count;
  uint32_t task_max_ms[APP_HEALTH_TASK_COUNT];
} AppHealthSnapshot_t;

void AppHealth_Init(void);
void AppHealth_LoopBegin(uint32_t now);
void AppHealth_TaskBegin(AppHealthTask_t task, uint32_t now);
void AppHealth_TaskEnd(AppHealthTask_t task, uint32_t now);
void AppHealth_GetSnapshot(AppHealthSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* __APP_HEALTH_H */
