#ifndef __STATUS_LED_H
#define __STATUS_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  STATUS_LED_IDLE = 0,
  STATUS_LED_MANUAL,
  STATUS_LED_FOLLOW,
  STATUS_LED_AUTO,
  STATUS_LED_TRACK,
  STATUS_LED_WAIT_LOAD,
  STATUS_LED_READY,
  STATUS_LED_TRANSPORT,
  STATUS_LED_TRANSPORT_RUN,
  STATUS_LED_TRANSPORT_OBSTACLE,
  STATUS_LED_TRANSPORT_ARRIVED,
  STATUS_LED_OBSTACLE,
  STATUS_LED_UNLOAD,
  STATUS_LED_UNLOADING,
  STATUS_LED_UNLOAD_DONE,
  STATUS_LED_FINISH,
  STATUS_LED_TRANSPORT_FINISH,
  STATUS_LED_ERROR,
  STATUS_LED_EMERGENCY
} StatusLedState_t;

void StatusLed_Init(void);
void StatusLed_Task(void);
void StatusLed_SetState(StatusLedState_t state);
StatusLedState_t StatusLed_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATUS_LED_H */
