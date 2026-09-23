#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void WS2812_Init(void);
uint8_t WS2812_GetLedCount(void);
void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetPixelRGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Clear(void);
void WS2812_Show(void);
void WS2812_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */
