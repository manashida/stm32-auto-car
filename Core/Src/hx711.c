#include "hx711.h"

#define HX711_DT_PORT       GPIOB
#define HX711_DT_PIN        GPIO_PIN_15
#define HX711_SCK_PORT      GPIOB
#define HX711_SCK_PIN       GPIO_PIN_14

#define HX711_READY_TIMEOUT_US 100000U

static int32_t hx711_offset = 0;
static float hx711_scale = HX711_DEFAULT_SCALE;
static HX711_StatusTypeDef s_last_status = HX711_OK;

static void HX711_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t HX711_Micros(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static void HX711_DelayUs(uint32_t us)
{
  uint32_t start = HX711_Micros();

  while ((uint32_t)(HX711_Micros() - start) < us)
  {
  }
}

static HX711_StatusTypeDef HX711_WaitReady(uint32_t timeout_us)
{
  uint32_t start = HX711_Micros();

  while (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_SET)
  {
    if ((uint32_t)(HX711_Micros() - start) >= timeout_us)
    {
      s_last_status = HX711_ERROR_TIMEOUT;
      return HX711_ERROR_TIMEOUT;
    }
  }

  s_last_status = HX711_OK;
  return HX711_OK;
}

static HX711_StatusTypeDef HX711_ReadReadyRaw(int32_t *raw)
{
  uint32_t value = 0;

  if (raw == NULL)
  {
    s_last_status = HX711_ERROR_TIMEOUT;
    return HX711_ERROR_TIMEOUT;
  }

  for (uint8_t i = 0; i < 24U; i++)
  {
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
    HX711_DelayUs(1);
    value = (value << 1U) | (uint32_t)(HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
    HX711_DelayUs(1);
  }

  /* 25th pulse: channel A, gain 128. */
  HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_SET);
  HX711_DelayUs(1);
  HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);
  HX711_DelayUs(1);

  if ((value & 0x800000U) != 0U)
  {
    value |= 0xFF000000U;
  }

  *raw = (int32_t)value;
  s_last_status = HX711_OK;
  return HX711_OK;
}

void HX711_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HX711_DwtInit();

  HAL_GPIO_WritePin(HX711_SCK_PORT, HX711_SCK_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = HX711_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = HX711_DT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX711_DT_PORT, &GPIO_InitStruct);

  hx711_offset = 0;
  hx711_scale = HX711_DEFAULT_SCALE;
  s_last_status = HX711_OK;
}

uint8_t HX711_IsReady(void)
{
  return (HAL_GPIO_ReadPin(HX711_DT_PORT, HX711_DT_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

HX711_StatusTypeDef HX711_ReadRaw(int32_t *raw)
{
  if (raw == NULL)
  {
    s_last_status = HX711_ERROR_TIMEOUT;
    return HX711_ERROR_TIMEOUT;
  }

  if (HX711_WaitReady(HX711_READY_TIMEOUT_US) != HX711_OK)
  {
    *raw = 0;
    return HX711_ERROR_TIMEOUT;
  }

  return HX711_ReadReadyRaw(raw);
}

HX711_StatusTypeDef HX711_ReadRawIfReady(int32_t *raw)
{
  if (HX711_IsReady() == 0U)
  {
    if (raw != NULL)
    {
      *raw = 0;
    }
    s_last_status = HX711_ERROR_TIMEOUT;
    return HX711_ERROR_TIMEOUT;
  }

  return HX711_ReadReadyRaw(raw);
}

static HX711_StatusTypeDef HX711_ReadAverage(uint8_t times, int32_t *average)
{
  int64_t sum = 0;
  int32_t raw = 0;

  if (average == NULL)
  {
    s_last_status = HX711_ERROR_TIMEOUT;
    return HX711_ERROR_TIMEOUT;
  }

  if (times == 0U)
  {
    times = 1U;
  }

  for (uint8_t i = 0; i < times; i++)
  {
    if (HX711_ReadRaw(&raw) != HX711_OK)
    {
      *average = 0;
      return HX711_ERROR_TIMEOUT;
    }

    sum += raw;
  }

  *average = (int32_t)(sum / times);
  s_last_status = HX711_OK;
  return HX711_OK;
}

HX711_StatusTypeDef HX711_Tare(uint8_t times)
{
  int32_t average = 0;

  if (HX711_ReadAverage(times, &average) != HX711_OK)
  {
    return HX711_ERROR_TIMEOUT;
  }

  hx711_offset = average;
  s_last_status = HX711_OK;
  return HX711_OK;
}

void HX711_SetScale(float scale)
{
  if (scale != 0.0f)
  {
    hx711_scale = scale;
  }
}

void HX711_SetOffset(int32_t offset)
{
  hx711_offset = offset;
}

float HX711_GetScale(void)
{
  return hx711_scale;
}

int32_t HX711_GetOffset(void)
{
  return hx711_offset;
}

float HX711_RawToWeight(int32_t raw)
{
  return ((float)(raw - hx711_offset)) / hx711_scale;
}

float HX711_GetWeight(uint8_t times)
{
  int32_t average = 0;

  if (HX711_ReadAverage(times, &average) != HX711_OK)
  {
    return 0.0f;
  }

  return HX711_RawToWeight(average);
}

float HX711_Calibrate(float known_weight_g, uint8_t times)
{
  int32_t average = 0;

  if (known_weight_g == 0.0f)
  {
    return hx711_scale;
  }

  if (HX711_ReadAverage(times, &average) != HX711_OK)
  {
    return hx711_scale;
  }

  hx711_scale = ((float)(average - hx711_offset)) / known_weight_g;
  return hx711_scale;
}

HX711_StatusTypeDef HX711_GetLastStatus(void)
{
  return s_last_status;
}
