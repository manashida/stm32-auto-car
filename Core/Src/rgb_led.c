#include "rgb_led.h"
#include "ws2812.h"

static uint8_t RGB_ToWsValue(uint8_t value)
{
  if (value == 0U)
  {
    return 0U;
  }

  return (value < 8U) ? 255U : value;
}

void RGB_Init(void)
{
  WS2812_Init();
}

void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
  WS2812_SetRGB(RGB_ToWsValue(r), RGB_ToWsValue(g), RGB_ToWsValue(b));
  WS2812_Show();
}

void RGB_Off(void)
{
  WS2812_Off();
}

void RGB_Red(void)
{
  RGB_Set(255U, 0U, 0U);
}

void RGB_Green(void)
{
  RGB_Set(0U, 255U, 0U);
}

void RGB_Blue(void)
{
  RGB_Set(0U, 0U, 255U);
}

void RGB_Yellow(void)
{
  RGB_Set(255U, 255U, 0U);
}

void RGB_Cyan(void)
{
  RGB_Set(0U, 255U, 255U);
}

void RGB_Purple(void)
{
  RGB_Set(255U, 0U, 255U);
}

void RGB_White(void)
{
  RGB_Set(255U, 255U, 255U);
}
