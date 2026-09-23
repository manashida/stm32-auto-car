#include "motor.h"
#include "tim.h"

#define MOTOR_MAX_SPEED 1000

#define LEFT_IN1_PORT GPIOB
#define LEFT_IN1_PIN  GPIO_PIN_0
#define LEFT_IN2_PORT GPIOB
#define LEFT_IN2_PIN  GPIO_PIN_1

#define RIGHT_IN1_PORT GPIOB
#define RIGHT_IN1_PIN  GPIO_PIN_10
#define RIGHT_IN2_PORT GPIOB
#define RIGHT_IN2_PIN  GPIO_PIN_11

#define LEFT_MOTOR_INVERT  0
#define RIGHT_MOTOR_INVERT 1

static int16_t Motor_ClampSpeed(int16_t speed)
{
  if (speed > MOTOR_MAX_SPEED)
  {
    return MOTOR_MAX_SPEED;
  }
  if (speed < -MOTOR_MAX_SPEED)
  {
    return -MOTOR_MAX_SPEED;
  }
  return speed;
}

static void Motor_SetDirection(GPIO_TypeDef *in1_port,
                               uint16_t in1_pin,
                               GPIO_TypeDef *in2_port,
                               uint16_t in2_pin,
                               int16_t speed)
{
  if (speed > 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
}

static uint32_t Motor_GetPulse(int16_t speed)
{
  int16_t clamped = Motor_ClampSpeed(speed);

  if (clamped < 0)
  {
    clamped = (int16_t)-clamped;
  }

  return (uint32_t)clamped;
}

static int16_t Motor_ApplyInvert(int16_t speed, uint8_t invert)
{
  if (invert != 0U)
  {
    return (int16_t)-speed;
  }

  return speed;
}

void Motor_Init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Motor_SetLeftSpeed(0);
  Motor_SetRightSpeed(0);
}

void Motor_SetLeftSpeed(int16_t speed)
{
  int16_t clamped = Motor_ClampSpeed(speed);
  int16_t direction_speed = Motor_ApplyInvert(clamped, LEFT_MOTOR_INVERT);

  Motor_SetDirection(LEFT_IN1_PORT, LEFT_IN1_PIN,
                     LEFT_IN2_PORT, LEFT_IN2_PIN,
                     direction_speed);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Motor_GetPulse(clamped));
}

void Motor_SetRightSpeed(int16_t speed)
{
  int16_t clamped = Motor_ClampSpeed(speed);
  int16_t direction_speed = Motor_ApplyInvert(clamped, RIGHT_MOTOR_INVERT);

  Motor_SetDirection(RIGHT_IN1_PORT, RIGHT_IN1_PIN,
                     RIGHT_IN2_PORT, RIGHT_IN2_PIN,
                     direction_speed);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Motor_GetPulse(clamped));
}
