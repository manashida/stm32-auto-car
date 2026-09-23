#ifndef __TRANSPORT_TASK_H
#define __TRANSPORT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  TRANSPORT_IDLE = 0,
  TRANSPORT_WAIT_LOAD,
  TRANSPORT_START,
  TRANSPORT_LOAD_CHECK,
  TRANSPORT_RUN_TRACK,
  TRANSPORT_RUNNING = TRANSPORT_RUN_TRACK,
  TRANSPORT_OBSTACLE,
  TRANSPORT_ARRIVED_LOST_LINE,
  TRANSPORT_ARRIVED = TRANSPORT_ARRIVED_LOST_LINE,
  TRANSPORT_UNLOAD_OPEN,
  TRANSPORT_UNLOAD_WAIT,
  TRANSPORT_UNLOAD_CHECK,
  TRANSPORT_UNLOAD_RETRY_CLOSE,
  TRANSPORT_UNLOAD_CLOSE,
  TRANSPORT_TURN_BEFORE_UNLOAD,
  TRANSPORT_RETURN_TURN,
  TRANSPORT_RETURN_SEARCH_LINE,
  TRANSPORT_RETURN_TRACK,
  TRANSPORT_HOME_TURN,
  TRANSPORT_FINISH,
  TRANSPORT_ERROR,
  TRANSPORT_EMERGENCY_STOP
} TransportState_t;

void Transport_Init(void);
void Transport_Task(void);
void Transport_Start(void);
void Transport_Stop(void);
void Transport_EnterWaitLoad(void);
void Transport_EmergencyStop(void);
void Transport_Reset(void);

TransportState_t Transport_GetState(void);
const char *Transport_GetStateName(void);
uint8_t Transport_IsLoadReady(void);
uint8_t Transport_IsBusy(void);
uint8_t Transport_IsError(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSPORT_TASK_H */
