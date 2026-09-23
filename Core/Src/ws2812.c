#include "ws2812.h"

#define WS2812_PORT          GPIOA
#define WS2812_PIN           GPIO_PIN_12
/* The RGB-LED bar module in use is a 10-position WS2812RGB strip. */
#define WS2812_LED_COUNT     10U
#define WS2812_RESET_US      300U

/* These timings are calibrated for a 72 MHz STM32F103 core.
 * The 0-bit high pulse is intentionally short because an overlong T0H makes
 * WS2812 read zero bits as one bits, which shows up as a constant white strip.
 */
#define WS2812_T0H_CYCLES    18U
#define WS2812_T1H_CYCLES    50U
#define WS2812_PERIOD_CYCLES 90U

typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} WS2812_Color_t;

static WS2812_Color_t s_pixels[WS2812_LED_COUNT];

static void WS2812_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t WS2812_Micros(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static void WS2812_DelayUs(uint32_t us)
{
  uint32_t start = WS2812_Micros();

  while ((uint32_t)(WS2812_Micros() - start) < us)
  {
  }
}

static void WS2812_WriteByte(uint8_t value)
{
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    uint32_t high_cycles = ((value & mask) != 0U) ?
                           WS2812_T1H_CYCLES :
                           WS2812_T0H_CYCLES;
    uint32_t start;

    WS2812_PORT->BSRR = WS2812_PIN;
    start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < high_cycles)
    {
    }

    WS2812_PORT->BRR = WS2812_PIN;

    while ((uint32_t)(DWT->CYCCNT - start) < WS2812_PERIOD_CYCLES)
    {
    }
  }
}

void WS2812_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  WS2812_DwtInit();

  HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = WS2812_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WS2812_PORT, &GPIO_InitStruct);

  WS2812_Off();
}

uint8_t WS2812_GetLedCount(void)
{
  return WS2812_LED_COUNT;
}

void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint8_t i = 0; i < WS2812_LED_COUNT; i++)
  {
    s_pixels[i].red = r;
    s_pixels[i].green = g;
    s_pixels[i].blue = b;
  }
}

void WS2812_SetPixelRGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
  if (index >= WS2812_LED_COUNT)
  {
    return;
  }

  s_pixels[index].red = r;
  s_pixels[index].green = g;
  s_pixels[index].blue = b;
}

void WS2812_Clear(void)
{
  WS2812_SetRGB(0U, 0U, 0U);
}

void WS2812_Show(void)
{
  uint32_t primask = __get_PRIMASK();

  /* WS2812 has sub-microsecond bit timing. Keep interrupts closed only while
   * one LED frame is shifted out, then restore the previous IRQ state.
   */
  __disable_irq();
  for (uint8_t i = 0; i < WS2812_LED_COUNT; i++)
  {
    WS2812_WriteByte(s_pixels[i].green);
    WS2812_WriteByte(s_pixels[i].red);
    WS2812_WriteByte(s_pixels[i].blue);
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
  WS2812_DelayUs(WS2812_RESET_US);
}

void WS2812_Off(void)
{
  WS2812_Clear();
  WS2812_Show();
}
