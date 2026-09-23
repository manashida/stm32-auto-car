#ifndef __RGB_LED_H
#define __RGB_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void RGB_Init(void);
void RGB_Off(void);
void RGB_Red(void);
void RGB_Green(void);
void RGB_Blue(void);
void RGB_Yellow(void);
void RGB_Cyan(void);
void RGB_Purple(void);
void RGB_White(void);
void RGB_Set(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* __RGB_LED_H */
