#include "car.h"
#include "motor.h"

void Car_Init(void)
{
  Motor_Init();
}

void Car_Forward(int16_t speed)
{
  Motor_SetLeftSpeed(speed);
  Motor_SetRightSpeed(speed);
}

void Car_Backward(int16_t speed)
{
  Motor_SetLeftSpeed((int16_t)-speed);
  Motor_SetRightSpeed((int16_t)-speed);
}

void Car_Left(int16_t speed)
{
  Motor_SetLeftSpeed((int16_t)-speed);
  Motor_SetRightSpeed(speed);
}

void Car_Right(int16_t speed)
{
  Motor_SetLeftSpeed(speed);
  Motor_SetRightSpeed((int16_t)-speed);
}

void Car_Stop(void)
{
  Motor_SetLeftSpeed(0);
  Motor_SetRightSpeed(0);
}
