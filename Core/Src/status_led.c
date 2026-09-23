#include "status_led.h"
#include "ws2812.h"

#define STATUS_LED_DIM_LEVEL 80U

typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} StatusLedColor_t;

static StatusLedState_t s_state = STATUS_LED_IDLE;

static StatusLedColor_t StatusLed_MakeColor(uint8_t red, uint8_t green, uint8_t blue)
{
  StatusLedColor_t color;

  color.red = red;
  color.green = green;
  color.blue = blue;
  return color;
}

static StatusLedColor_t StatusLed_GetColor(StatusLedState_t state)
{
  switch (state)
  {
    case STATUS_LED_WAIT_LOAD:
      return StatusLed_MakeColor(0U, 0U, STATUS_LED_DIM_LEVEL);

    case STATUS_LED_READY:
      return StatusLed_MakeColor(0U, STATUS_LED_DIM_LEVEL, STATUS_LED_DIM_LEVEL);

    case STATUS_LED_AUTO:
    case STATUS_LED_TRACK:
    case STATUS_LED_TRANSPORT:
    case STATUS_LED_TRANSPORT_RUN:
    case STATUS_LED_FINISH:
    case STATUS_LED_TRANSPORT_FINISH:
      return StatusLed_MakeColor(0U, STATUS_LED_DIM_LEVEL, 0U);

    case STATUS_LED_TRANSPORT_ARRIVED:
    case STATUS_LED_FOLLOW:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL, 0U, STATUS_LED_DIM_LEVEL);

    case STATUS_LED_UNLOAD:
    case STATUS_LED_UNLOADING:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL, STATUS_LED_DIM_LEVEL, 0U);

    case STATUS_LED_UNLOAD_DONE:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL,
                                 STATUS_LED_DIM_LEVEL,
                                 STATUS_LED_DIM_LEVEL);

    case STATUS_LED_OBSTACLE:
    case STATUS_LED_TRANSPORT_OBSTACLE:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL, STATUS_LED_DIM_LEVEL, 0U);

    case STATUS_LED_ERROR:
    case STATUS_LED_EMERGENCY:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL, 0U, 0U);

    case STATUS_LED_MANUAL:
    default:
      return StatusLed_MakeColor(STATUS_LED_DIM_LEVEL,
                                 STATUS_LED_DIM_LEVEL,
                                 STATUS_LED_DIM_LEVEL);
  }
}

static void StatusLed_ApplyState(StatusLedState_t state)
{
  if (state == STATUS_LED_IDLE)
  {
    WS2812_Clear();
  }
  else
  {
    StatusLedColor_t color = StatusLed_GetColor(state);
    WS2812_SetRGB(color.red, color.green, color.blue);
  }

  WS2812_Show();
}

void StatusLed_Init(void)
{
  WS2812_Init();
  s_state = STATUS_LED_IDLE;
  StatusLed_ApplyState(s_state);
}

void StatusLed_SetState(StatusLedState_t state)
{
  if (s_state == state)
  {
    return;
  }

  s_state = state;
  StatusLed_ApplyState(s_state);
}

StatusLedState_t StatusLed_GetState(void)
{
  return s_state;
}

void StatusLed_Task(void)
{
}
