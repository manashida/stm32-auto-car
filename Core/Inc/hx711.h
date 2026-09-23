#ifndef __HX711_H
#define __HX711_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define HX711_DEFAULT_SCALE       (-373.70f)
#define HX711_DEFAULT_TARE_TIMES  20U

typedef enum
{
  HX711_OK = 0,
  HX711_ERROR_TIMEOUT = 1
} HX711_StatusTypeDef;

void HX711_Init(void);
uint8_t HX711_IsReady(void);
HX711_StatusTypeDef HX711_ReadRaw(int32_t *raw);
HX711_StatusTypeDef HX711_ReadRawIfReady(int32_t *raw);
HX711_StatusTypeDef HX711_Tare(uint8_t times);
void HX711_SetScale(float scale);
void HX711_SetOffset(int32_t offset);
float HX711_GetScale(void);
int32_t HX711_GetOffset(void);
float HX711_RawToWeight(int32_t raw);
float HX711_GetWeight(uint8_t times);
float HX711_Calibrate(float known_weight_g, uint8_t times);
HX711_StatusTypeDef HX711_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __HX711_H */
