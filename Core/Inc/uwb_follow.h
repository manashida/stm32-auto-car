#ifndef __UWB_FOLLOW_H
#define __UWB_FOLLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint32_t distance_cm;
  int16_t angle_deg;
  int16_t rssi_dbm;
  uint8_t rssi_raw;
  uint8_t status;
  uint32_t tag_id;
  uint16_t sequence;
  uint32_t update_tick;
} UwbFollow_Data_t;

void UwbFollow_Init(void);
void UwbFollow_Task(void);
void UwbFollow_OnRxCpltCallback(UART_HandleTypeDef *huart);
void UwbFollow_OnErrorCallback(UART_HandleTypeDef *huart);
uint8_t UwbFollow_GetLatest(UwbFollow_Data_t *out);
uint32_t UwbFollow_GetRxCount(void);
uint32_t UwbFollow_GetParseOkCount(void);
uint32_t UwbFollow_GetParseErrCount(void);
uint32_t UwbFollow_GetRxErrorCount(void);
uint32_t UwbFollow_GetLastRxError(void);

#ifdef __cplusplus
}
#endif

#endif /* __UWB_FOLLOW_H */
