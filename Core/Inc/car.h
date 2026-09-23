#ifndef __CAR_H
#define __CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Car_Init(void);
void Car_Forward(int16_t speed);
void Car_Backward(int16_t speed);
void Car_Left(int16_t speed);
void Car_Right(int16_t speed);
void Car_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */
