#include "uwb_follow.h"
#include "app_config.h"
#include "usart.h"
#include <stdio.h>

#define UWB_FOLLOW_SIMPLE_HEADER      0xF0U
#define UWB_FOLLOW_SIMPLE_MARK        0x06U
#define UWB_FOLLOW_SIMPLE_TAIL        0xAAU
#define UWB_FOLLOW_SIMPLE_FRAME_LEN   10U

#define UWB_FOLLOW_GC_HEADER_0        0x59U
#define UWB_FOLLOW_GC_HEADER_1        0x4DU
#define UWB_FOLLOW_GC_CMD_PDOA        0x36U
#define UWB_FOLLOW_GC_HEADER_LEN      7U
#define UWB_FOLLOW_GC_FRAME_OVERHEAD  8U
#define UWB_FOLLOW_GC_MIN_PAYLOAD_LEN 25U
#define UWB_FOLLOW_GC_MAX_FRAME_LEN   (UWB_FOLLOW_GC_FRAME_OVERHEAD + UWB_FOLLOW_GC_MAX_PAYLOAD_LEN)

typedef enum
{
  UWB_PARSE_WAIT_HEADER = 0,
  UWB_PARSE_SIMPLE_FRAME,
  UWB_PARSE_GC_FRAME
} UwbParseState_t;

static volatile uint8_t s_rx_buffer[UWB_FOLLOW_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_rx_overflow_count = 0;
static volatile uint32_t s_rx_error_count = 0;
static volatile uint32_t s_rx_last_error = HAL_UART_ERROR_NONE;
static uint8_t s_rx_byte = 0;

static UwbParseState_t s_parse_state = UWB_PARSE_WAIT_HEADER;
static uint8_t s_frame[UWB_FOLLOW_GC_MAX_FRAME_LEN];
static uint16_t s_frame_index = 0;
static uint16_t s_frame_expected_len = 0;
static uint16_t s_sequence = 0;

static UwbFollow_Data_t s_latest;
static uint8_t s_has_latest = 0;
static uint32_t s_parse_ok_count = 0;
static uint32_t s_parse_err_count = 0;

#if UWB_FOLLOW_DEBUG_ENABLE
static uint32_t s_debug_last_tick = 0;
#endif

static void UwbFollow_ResetParser(void)
{
  s_parse_state = UWB_PARSE_WAIT_HEADER;
  s_frame_index = 0;
  s_frame_expected_len = 0;
}

static uint16_t UwbFollow_ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t UwbFollow_ReadI16LE(const uint8_t *data)
{
  return (int16_t)UwbFollow_ReadU16LE(data);
}

static uint32_t UwbFollow_ReadU24LE(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16);
}

static uint32_t UwbFollow_ReadU32LE(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static uint8_t UwbFollow_ChecksumIsValid(uint16_t frame_len)
{
  uint16_t i;
  uint8_t sum = 0U;

  if (frame_len == 0U)
  {
    return 0U;
  }

  for (i = 0U; i < (uint16_t)(frame_len - 1U); i++)
  {
    sum = (uint8_t)(sum + s_frame[i]);
  }

  return (sum == s_frame[frame_len - 1U]) ? 1U : 0U;
}

static uint8_t UwbFollow_TargetAccepted(uint32_t tag_id)
{
#if UWB_FOLLOW_FILTER_TARGET_ID
  return (tag_id == UWB_FOLLOW_TARGET_ID) ? 1U : 0U;
#else
  (void)tag_id;
  return 1U;
#endif
}

static void UwbFollow_StartReceive_IT(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

static uint8_t UwbFollow_QueuePop(uint8_t *out)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (s_rx_tail == s_rx_head)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }

  *out = s_rx_buffer[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1U) % UWB_FOLLOW_RX_BUFFER_SIZE);

  if (primask == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

static void UwbFollow_QueuePushFromISR(uint8_t value)
{
  uint16_t next = (uint16_t)((s_rx_head + 1U) % UWB_FOLLOW_RX_BUFFER_SIZE);

  s_rx_count++;

  if (next == s_rx_tail)
  {
    s_rx_overflow_count++;
    return;
  }

  s_rx_buffer[s_rx_head] = value;
  s_rx_head = next;
}

static uint8_t UwbFollow_ParseSimpleFrame(void)
{
  UwbFollow_Data_t data = {0};
  uint16_t module_id = 0;

  if ((s_frame[0] != UWB_FOLLOW_SIMPLE_HEADER) ||
      (s_frame[1] != UWB_FOLLOW_SIMPLE_MARK) ||
      (s_frame[9] != UWB_FOLLOW_SIMPLE_TAIL))
  {
    return 0U;
  }

  module_id = UwbFollow_ReadU16LE(&s_frame[2]);
  data.tag_id = module_id;
  data.distance_cm = UwbFollow_ReadU16LE(&s_frame[4]);
  data.angle_deg = UwbFollow_ReadI16LE(&s_frame[6]);
  data.rssi_raw = s_frame[8];
  data.rssi_dbm = (int16_t)data.rssi_raw - 256;
  data.status = 0U;
  data.sequence = s_sequence++;
  data.update_tick = HAL_GetTick();

  if (UwbFollow_TargetAccepted(module_id) == 0U)
  {
    return 1U;
  }

  s_latest = data;
  s_has_latest = 1U;
  return 1U;
}

static uint8_t UwbFollow_ParseGcP2304Frame(void)
{
  const uint8_t *payload = &s_frame[UWB_FOLLOW_GC_HEADER_LEN];
  UwbFollow_Data_t data = {0};
  uint16_t payload_len = UwbFollow_ReadU16LE(&s_frame[5]);
  uint32_t tag_id = 0;

  if ((s_frame[0] != UWB_FOLLOW_GC_HEADER_0) ||
      (s_frame[1] != UWB_FOLLOW_GC_HEADER_1) ||
      (s_frame[2] != UWB_FOLLOW_GC_CMD_PDOA))
  {
    return 0U;
  }

  if ((payload_len < UWB_FOLLOW_GC_MIN_PAYLOAD_LEN) ||
      (payload_len > UWB_FOLLOW_GC_MAX_PAYLOAD_LEN))
  {
    return 0U;
  }

  if (UwbFollow_ChecksumIsValid((uint16_t)(UWB_FOLLOW_GC_FRAME_OVERHEAD + payload_len)) == 0U)
  {
    return 0U;
  }

  tag_id = UwbFollow_ReadU24LE(&payload[9]);
  data.tag_id = tag_id;
  data.sequence = UwbFollow_ReadU16LE(&payload[12]);
  data.distance_cm = UwbFollow_ReadU32LE(&payload[14]);
  data.angle_deg = UwbFollow_ReadI16LE(&payload[19]);
  data.rssi_raw = payload[23];
  data.rssi_dbm = (int16_t)((int8_t)payload[23]);
  data.status = payload[24];
  data.update_tick = HAL_GetTick();

  if (UwbFollow_TargetAccepted(tag_id) == 0U)
  {
    return 1U;
  }

  s_latest = data;
  s_has_latest = 1U;
  return 1U;
}

static void UwbFollow_HandleFrame(void)
{
  uint8_t parsed = 0U;

  if (s_parse_state == UWB_PARSE_SIMPLE_FRAME)
  {
    parsed = UwbFollow_ParseSimpleFrame();
  }
  else if (s_parse_state == UWB_PARSE_GC_FRAME)
  {
    parsed = UwbFollow_ParseGcP2304Frame();
  }

  if (parsed != 0U)
  {
    s_parse_ok_count++;
  }
  else
  {
    s_parse_err_count++;
  }
}

static void UwbFollow_BeginSimpleFrame(void)
{
  s_frame[0] = UWB_FOLLOW_SIMPLE_HEADER;
  s_frame_index = 1U;
  s_frame_expected_len = UWB_FOLLOW_SIMPLE_FRAME_LEN;
  s_parse_state = UWB_PARSE_SIMPLE_FRAME;
}

static void UwbFollow_BeginGcFrame(void)
{
  s_frame[0] = UWB_FOLLOW_GC_HEADER_0;
  s_frame_index = 1U;
  s_frame_expected_len = 0U;
  s_parse_state = UWB_PARSE_GC_FRAME;
}

static void UwbFollow_ResyncFromByte(uint8_t value)
{
  UwbFollow_ResetParser();

#if UWB_FOLLOW_PROTOCOL_SIMPLE_ENABLE
  if (value == UWB_FOLLOW_SIMPLE_HEADER)
  {
    UwbFollow_BeginSimpleFrame();
    return;
  }
#endif

#if UWB_FOLLOW_PROTOCOL_GC_P2304_ENABLE
  if (value == UWB_FOLLOW_GC_HEADER_0)
  {
    UwbFollow_BeginGcFrame();
    return;
  }
#endif
}

static void UwbFollow_ParseByte(uint8_t value)
{
  switch (s_parse_state)
  {
    case UWB_PARSE_WAIT_HEADER:
#if UWB_FOLLOW_PROTOCOL_SIMPLE_ENABLE
      if (value == UWB_FOLLOW_SIMPLE_HEADER)
      {
        UwbFollow_BeginSimpleFrame();
      }
      else
#endif
#if UWB_FOLLOW_PROTOCOL_GC_P2304_ENABLE
      if (value == UWB_FOLLOW_GC_HEADER_0)
      {
        UwbFollow_BeginGcFrame();
      }
#endif
      break;

    case UWB_PARSE_SIMPLE_FRAME:
      s_frame[s_frame_index] = value;
      s_frame_index++;

      if ((s_frame_index == 2U) && (value != UWB_FOLLOW_SIMPLE_MARK))
      {
        s_parse_err_count++;
        UwbFollow_ResyncFromByte(value);
        break;
      }

      if (s_frame_index >= s_frame_expected_len)
      {
        UwbFollow_HandleFrame();
        UwbFollow_ResetParser();
      }
      break;

    case UWB_PARSE_GC_FRAME:
      if (s_frame_index >= UWB_FOLLOW_GC_MAX_FRAME_LEN)
      {
        s_parse_err_count++;
        UwbFollow_ResyncFromByte(value);
        break;
      }

      s_frame[s_frame_index] = value;
      s_frame_index++;

      if ((s_frame_index == 2U) && (value != UWB_FOLLOW_GC_HEADER_1))
      {
        s_parse_err_count++;
        UwbFollow_ResyncFromByte(value);
        break;
      }

      if (s_frame_index == UWB_FOLLOW_GC_HEADER_LEN)
      {
        uint16_t payload_len = UwbFollow_ReadU16LE(&s_frame[5]);

        if (payload_len > UWB_FOLLOW_GC_MAX_PAYLOAD_LEN)
        {
          s_parse_err_count++;
          UwbFollow_ResetParser();
          break;
        }

        s_frame_expected_len = (uint16_t)(UWB_FOLLOW_GC_FRAME_OVERHEAD + payload_len);
      }

      if ((s_frame_expected_len != 0U) && (s_frame_index >= s_frame_expected_len))
      {
        UwbFollow_HandleFrame();
        UwbFollow_ResetParser();
      }
      break;

    default:
      UwbFollow_ResetParser();
      break;
  }
}

void UwbFollow_Init(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_rx_head = 0;
  s_rx_tail = 0;
  s_rx_count = 0;
  s_rx_overflow_count = 0;
  s_rx_error_count = 0;
  s_rx_last_error = HAL_UART_ERROR_NONE;
  if (primask == 0U)
  {
    __enable_irq();
  }

  s_has_latest = 0U;
  s_parse_ok_count = 0;
  s_parse_err_count = 0;
  s_sequence = 0;
  UwbFollow_ResetParser();
  UwbFollow_StartReceive_IT();
}

void UwbFollow_Task(void)
{
  uint8_t value = 0;

  while (UwbFollow_QueuePop(&value) != 0U)
  {
    UwbFollow_ParseByte(value);
  }

#if UWB_FOLLOW_DEBUG_ENABLE
  {
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_debug_last_tick) >= UWB_FOLLOW_DEBUG_PERIOD_MS)
    {
      char text[144];
      uint32_t age = 0xFFFFFFFFU;
      int len;

      s_debug_last_tick = now;
      if (s_has_latest != 0U)
      {
        age = (uint32_t)(now - s_latest.update_tick);
      }

      len = snprintf(text, sizeof(text),
                     "UWB rx=%lu ok=%lu err=%lu ovf=%lu id=%04lX dist=%lu angle=%d rssi=%d age=%lu\r\n",
                     (unsigned long)s_rx_count,
                     (unsigned long)s_parse_ok_count,
                     (unsigned long)s_parse_err_count,
                     (unsigned long)s_rx_overflow_count,
                     (unsigned long)s_latest.tag_id,
                     (unsigned long)s_latest.distance_cm,
                     (int)s_latest.angle_deg,
                     (int)s_latest.rssi_dbm,
                     (unsigned long)age);
      if (len > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 10);
      }
    }
  }
#endif
}

void UwbFollow_OnRxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  UwbFollow_QueuePushFromISR(s_rx_byte);
  UwbFollow_StartReceive_IT();
}

void UwbFollow_OnErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  s_rx_error_count++;
  s_rx_last_error = HAL_UART_GetError(huart);
  (void)HAL_UART_AbortReceive(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  UwbFollow_StartReceive_IT();
}

uint8_t UwbFollow_GetLatest(UwbFollow_Data_t *out)
{
  if ((out == NULL) || (s_has_latest == 0U))
  {
    return 0U;
  }

  *out = s_latest;
  return 1U;
}

uint32_t UwbFollow_GetRxCount(void)
{
  return s_rx_count;
}

uint32_t UwbFollow_GetParseOkCount(void)
{
  return s_parse_ok_count;
}

uint32_t UwbFollow_GetParseErrCount(void)
{
  return s_parse_err_count;
}

uint32_t UwbFollow_GetRxErrorCount(void)
{
  return s_rx_error_count;
}

uint32_t UwbFollow_GetLastRxError(void)
{
  return s_rx_last_error;
}
