#ifndef __APP_MODE_TASK_H
#define __APP_MODE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  APP_MODE_AUTO_TRANSPORT = 0,
  APP_MODE_BLUETOOTH_CONTROL,
  APP_MODE_FOLLOW_LIGHT
} AppMode_t;

void AppMode_Init(void);
void AppMode_Task(void);
AppMode_t AppMode_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MODE_TASK_H */
