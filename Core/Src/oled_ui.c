#include "oled_ui.h"
#include "app_mode_task.h"
#include "encoder.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>

#define OLED_I2C_ADDRESS    (0x3C << 1)
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGE_COUNT     (OLED_HEIGHT / 8)
#define OLED_BUFFER_SIZE    (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_REFRESH_MS     500U

static uint8_t s_oled_buffer[OLED_BUFFER_SIZE];
static uint32_t s_last_refresh_tick;

__attribute__((weak)) int32_t g_weight_g = 0;

static const uint8_t s_font_6x8[][6] = {
  [' ' - 32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  ['-' - 32] = {0x08, 0x08, 0x08, 0x08, 0x08, 0x00},
  ['.' - 32] = {0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
  ['/' - 32] = {0x20, 0x10, 0x08, 0x04, 0x02, 0x00},
  [':' - 32] = {0x00, 0x00, 0x36, 0x36, 0x00, 0x00},
  ['0' - 32] = {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00},
  ['1' - 32] = {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},
  ['2' - 32] = {0x42, 0x61, 0x51, 0x49, 0x46, 0x00},
  ['3' - 32] = {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00},
  ['4' - 32] = {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},
  ['5' - 32] = {0x27, 0x45, 0x45, 0x45, 0x39, 0x00},
  ['6' - 32] = {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00},
  ['7' - 32] = {0x01, 0x71, 0x09, 0x05, 0x03, 0x00},
  ['8' - 32] = {0x36, 0x49, 0x49, 0x49, 0x36, 0x00},
  ['9' - 32] = {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00},
  ['A' - 32] = {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00},
  ['B' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00},
  ['D' - 32] = {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00},
  ['E' - 32] = {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00},
  ['F' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00},
  ['I' - 32] = {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00},
  ['L' - 32] = {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00},
  ['M' - 32] = {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00},
  ['O' - 32] = {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00},
  ['P' - 32] = {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00},
  ['S' - 32] = {0x26, 0x49, 0x49, 0x49, 0x32, 0x00},
  ['T' - 32] = {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00},
  ['U' - 32] = {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00},
  ['W' - 32] = {0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00},
  ['c' - 32] = {0x38, 0x44, 0x44, 0x44, 0x20, 0x00},
  ['g' - 32] = {0x08, 0x54, 0x54, 0x54, 0x3C, 0x00},
  ['m' - 32] = {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00},
  ['s' - 32] = {0x48, 0x54, 0x54, 0x54, 0x20, 0x00},
};

static void OLED_WriteCommand(uint8_t command)
{
  uint8_t data[2] = {0x00, command};
  HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, data, sizeof(data), 100);
}

static void OLED_WriteData(const uint8_t *data, uint16_t size)
{
  uint8_t packet[17];

  packet[0] = 0x40;
  while (size > 0U)
  {
    uint16_t chunk = (size > 16U) ? 16U : size;

    memcpy(&packet[1], data, chunk);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, packet, (uint16_t)(chunk + 1U), 100);

    data += chunk;
    size = (uint16_t)(size - chunk);
  }
}

static void OLED_SetCursor(uint8_t page, uint8_t column)
{
  OLED_WriteCommand((uint8_t)(0xB0U + page));
  OLED_WriteCommand((uint8_t)(0x00U + (column & 0x0FU)));
  OLED_WriteCommand((uint8_t)(0x10U + ((column >> 4U) & 0x0FU)));
}

static void OLED_UpdateScreen(void)
{
  for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++)
  {
    OLED_SetCursor(page, 0);
    OLED_WriteData(&s_oled_buffer[OLED_WIDTH * page], OLED_WIDTH);
  }
}

static void OLED_ClearBuffer(void)
{
  memset(s_oled_buffer, 0, sizeof(s_oled_buffer));
}

static void OLED_DrawChar2x(uint8_t x, uint8_t page, char ch)
{
  if ((x >= OLED_WIDTH) || (page >= (OLED_PAGE_COUNT - 1U)))
  {
    return;
  }

  if ((ch < ' ') || (ch > 's'))
  {
    ch = ' ';
  }

  for (uint8_t src_col = 0; src_col < 6U; src_col++)
  {
    uint8_t src = s_font_6x8[ch - 32][src_col];
    uint8_t upper = 0;
    uint8_t lower = 0;

    for (uint8_t bit = 0; bit < 8U; bit++)
    {
      if ((src & (uint8_t)(1U << bit)) != 0U)
      {
        uint8_t scaled_bit = (uint8_t)(bit * 2U);

        if (scaled_bit < 8U)
        {
          upper |= (uint8_t)(1U << scaled_bit);
          upper |= (uint8_t)(1U << (scaled_bit + 1U));
        }
        else
        {
          scaled_bit = (uint8_t)(scaled_bit - 8U);
          lower |= (uint8_t)(1U << scaled_bit);
          lower |= (uint8_t)(1U << (scaled_bit + 1U));
        }
      }
    }

    uint8_t dst_col = (uint8_t)(x + (src_col * 2U));

    if (dst_col < OLED_WIDTH)
    {
      s_oled_buffer[(page * OLED_WIDTH) + dst_col] = upper;
      s_oled_buffer[((page + 1U) * OLED_WIDTH) + dst_col] = lower;
    }
    if ((uint8_t)(dst_col + 1U) < OLED_WIDTH)
    {
      s_oled_buffer[(page * OLED_WIDTH) + dst_col + 1U] = upper;
      s_oled_buffer[((page + 1U) * OLED_WIDTH) + dst_col + 1U] = lower;
    }
  }
}

static void OLED_DrawString2x(uint8_t x, uint8_t page, const char *text)
{
  while ((*text != '\0') && (x < OLED_WIDTH))
  {
    OLED_DrawChar2x(x, page, *text);
    x = (uint8_t)(x + 12U);
    text++;
  }
}

static void OLED_DrawString2xCompact(uint8_t x, uint8_t page, const char *text)
{
  while ((*text != '\0') && (x < OLED_WIDTH))
  {
    OLED_DrawChar2x(x, page, *text);
    x = (uint8_t)(x + 11U);
    text++;
  }
}

static uint16_t OLED_LimitDistance(uint16_t distance)
{
  if (distance > 999U)
  {
    return 999U;
  }

  return distance;
}

static int32_t OLED_LimitVelocityMmps(int32_t velocity_mmps)
{
  if (velocity_mmps > 9990)
  {
    return 9990;
  }
  if (velocity_mmps < -9990)
  {
    return -9990;
  }

  return velocity_mmps;
}

static int32_t OLED_LimitWeight(int32_t weight)
{
  if (weight > 9999)
  {
    return 9999;
  }
  if (weight < -999)
  {
    return -999;
  }

  return weight;
}

void OLED_UI_Init(void)
{
  HAL_Delay(100);

  OLED_WriteCommand(0xAE);
  OLED_WriteCommand(0x20);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0xB0);
  OLED_WriteCommand(0xC8);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0x10);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0x81);
  OLED_WriteCommand(0x7F);
  OLED_WriteCommand(0xA1);
  OLED_WriteCommand(0xA6);
  OLED_WriteCommand(0xA8);
  OLED_WriteCommand(0x3F);
  OLED_WriteCommand(0xA4);
  OLED_WriteCommand(0xD3);
  OLED_WriteCommand(0x00);
  OLED_WriteCommand(0xD5);
  OLED_WriteCommand(0x80);
  OLED_WriteCommand(0xD9);
  OLED_WriteCommand(0xF1);
  OLED_WriteCommand(0xDA);
  OLED_WriteCommand(0x12);
  OLED_WriteCommand(0xDB);
  OLED_WriteCommand(0x40);
  OLED_WriteCommand(0x8D);
  OLED_WriteCommand(0x14);
  OLED_WriteCommand(0xAF);

  OLED_ClearBuffer();
  OLED_UpdateScreen();
  s_last_refresh_tick = HAL_GetTick();
}

void OLED_UI_Refresh(void)
{
  char line[24];
  const char *mode_text = "AUTO";
  int32_t velocity_mmps = OLED_LimitVelocityMmps(Encoder_GetVelocity_mmps());
  int32_t velocity_abs = (velocity_mmps < 0) ? -velocity_mmps : velocity_mmps;
  uint16_t distance = OLED_LimitDistance(g_distance_cm);
  int32_t weight = OLED_LimitWeight(g_weight_g);

  OLED_ClearBuffer();

  snprintf(line, sizeof(line), "SPD:%ld.%02ldm/s",
           (long)(velocity_abs / 1000),
           (long)((velocity_abs % 1000) / 10));
  OLED_DrawString2xCompact(0, 0, line);

  snprintf(line, sizeof(line), "DIS: %03ucm", distance);
  OLED_DrawString2x(0, 2, line);

  if (weight < 0)
  {
    snprintf(line, sizeof(line), "WT :-%03ldg", (long)-weight);
  }
  else
  {
    snprintf(line, sizeof(line), "WT : %04ldg", (long)weight);
  }
  OLED_DrawString2x(0, 4, line);

  if (AppMode_GetMode() == APP_MODE_BLUETOOTH_CONTROL)
  {
    mode_text = "BT";
  }
  else if (AppMode_GetMode() == APP_MODE_FOLLOW_LIGHT)
  {
    mode_text = "FOLLOW";
  }

  snprintf(line, sizeof(line), "MODE:%s", mode_text);
  OLED_DrawString2xCompact(0, 6, line);

  OLED_UpdateScreen();
}

void OLED_UI_Task(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - s_last_refresh_tick) >= OLED_REFRESH_MS)
  {
    s_last_refresh_tick = now;
    OLED_UI_Refresh();
  }
}

void OLED_UI_Update(void)
{
  OLED_UI_Task();
}
