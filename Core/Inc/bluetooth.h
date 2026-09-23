#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "car_mode.h"
#include <stdint.h>

void Bluetooth_Init(void);
void Bluetooth_Task(void);
void Bluetooth_StartReceive_IT(void);
void Bluetooth_OnRxCpltCallback(UART_HandleTypeDef *huart);
void Bluetooth_OnErrorCallback(UART_HandleTypeDef *huart);
uint32_t Bluetooth_GetRxErrorCount(void);
uint32_t Bluetooth_GetLastRxError(void);
CarMode_t Bluetooth_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H */
