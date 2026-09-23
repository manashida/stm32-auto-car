#include "transport_load.h"
#include "buzzer.h"
#include "status_led.h"
#include "transport_config.h"

extern int32_t g_weight_g;

static float s_load_weight_window[TRANSPORT_WAIT_LOAD_WINDOW_SIZE];
static uint8_t s_load_weight_index = 0;
static uint8_t s_load_weight_count = 0;
static float s_load_last_avg_g = 0.0f;
static uint8_t s_load_stable_count = 0;
static uint8_t s_load_ready = 0;
static uint8_t s_load_ready_notified = 0;
static uint8_t s_load_disturb_count = 0;
static uint32_t s_load_last_sample_tick = 0;
static uint32_t s_load_stable_start_tick = 0;
static uint32_t s_load_ready_tick = 0;

static float TransportLoad_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float TransportLoad_UpdateAverage(float weight_g)
{
  float sum = 0.0f;

  s_load_weight_window[s_load_weight_index] = weight_g;
  s_load_weight_index = (uint8_t)((s_load_weight_index + 1U) % TRANSPORT_WAIT_LOAD_WINDOW_SIZE);

  if (s_load_weight_count < TRANSPORT_WAIT_LOAD_WINDOW_SIZE)
  {
    s_load_weight_count++;
  }

  for (uint8_t i = 0; i < s_load_weight_count; i++)
  {
    sum += s_load_weight_window[i];
  }

  return sum / (float)s_load_weight_count;
}

static void TransportLoad_ResetStableState(float average_g)
{
  s_load_last_avg_g = average_g;
  s_load_stable_count = 0;
  s_load_stable_start_tick = 0;
}

static void TransportLoad_ClearReady(float average_g)
{
  TransportLoad_ResetStableState(average_g);
  s_load_ready = 0;
  s_load_ready_notified = 0;
  s_load_disturb_count = 0;
  s_load_ready_tick = 0;
  StatusLed_SetState(STATUS_LED_WAIT_LOAD);
  Buzzer_Stop();
}

static void TransportLoad_LatchReady(uint32_t now)
{
  if (s_load_ready != 0U)
  {
    return;
  }

  s_load_ready = 1U;
  s_load_ready_tick = now;
  s_load_disturb_count = 0;
  s_load_ready_notified = 0;
}

static uint8_t TransportLoad_ReadyExpired(uint32_t now)
{
  return ((s_load_ready != 0U) &&
          ((uint32_t)(now - s_load_ready_tick) >= TRANSPORT_WAIT_LOAD_READY_TIMEOUT_MS)) ? 1U : 0U;
}

static void TransportLoad_CheckLatchedReady(uint32_t now, float average_g, float diff_g)
{
  uint8_t unsafe = 0U;

  if (TransportLoad_ReadyExpired(now) != 0U)
  {
    TransportLoad_ClearReady(average_g);
    return;
  }

  if (average_g < TRANSPORT_WAIT_LOAD_READY_MIN_G)
  {
    unsafe = 1U;
  }
  else if (diff_g >= TRANSPORT_WAIT_LOAD_DISTURB_DIFF_G)
  {
    unsafe = 1U;
  }

  if (unsafe != 0U)
  {
    if (s_load_disturb_count < TRANSPORT_WAIT_LOAD_DISTURB_COUNT)
    {
      s_load_disturb_count++;
    }

    if (s_load_disturb_count >= TRANSPORT_WAIT_LOAD_DISTURB_COUNT)
    {
      TransportLoad_ClearReady(average_g);
    }

    return;
  }

  s_load_disturb_count = 0;
}

void TransportLoad_Reset(void)
{
  for (uint8_t i = 0; i < TRANSPORT_WAIT_LOAD_WINDOW_SIZE; i++)
  {
    s_load_weight_window[i] = 0.0f;
  }

  s_load_weight_index = 0;
  s_load_weight_count = 0;
  s_load_last_avg_g = 0.0f;
  s_load_stable_count = 0;
  s_load_ready = 0;
  s_load_ready_notified = 0;
  s_load_disturb_count = 0;
  s_load_last_sample_tick = 0;
  s_load_stable_start_tick = 0;
  s_load_ready_tick = 0;
}

void TransportLoad_Update(uint32_t now)
{
  float weight = 0.0f;
  float average = 0.0f;
  float diff = 0.0f;
  uint8_t load_over_threshold = 0U;

  if ((uint32_t)(now - s_load_last_sample_tick) < TRANSPORT_WAIT_LOAD_SAMPLE_MS)
  {
    return;
  }

  s_load_last_sample_tick = now;
  weight = (float)g_weight_g;
  average = TransportLoad_UpdateAverage(weight);

  diff = TransportLoad_AbsFloat(average - s_load_last_avg_g);

  if (s_load_ready != 0U)
  {
    TransportLoad_CheckLatchedReady(now, average, diff);
    s_load_last_avg_g = average;
    return;
  }

  load_over_threshold = (weight > TRANSPORT_WAIT_LOAD_THRESHOLD_G) ? 1U : 0U;

  if (load_over_threshold != 0U)
  {
    if (s_load_stable_start_tick == 0U)
    {
      s_load_stable_start_tick = now;
    }

    if (s_load_stable_count == 0U)
    {
      s_load_stable_count = 1U;
    }

    if ((uint32_t)(now - s_load_stable_start_tick) >= TRANSPORT_WAIT_LOAD_HOLD_MS)
    {
      TransportLoad_LatchReady(now);
    }
  }
  else
  {
    TransportLoad_ClearReady(average);
  }

  s_load_last_avg_g = average;

  if ((s_load_ready != 0U) && (s_load_ready_notified == 0U))
  {
    s_load_ready_notified = 1U;
    StatusLed_SetState(STATUS_LED_READY);
  }
}

uint8_t TransportLoad_IsReady(uint32_t now)
{
  if (TransportLoad_ReadyExpired(now) != 0U)
  {
    TransportLoad_ClearReady(s_load_last_avg_g);
    return 0U;
  }

  return (s_load_ready == 1U) ? 1U : 0U;
}

uint8_t TransportLoad_GetReadyLatched(void)
{
  return (s_load_ready != 0U) ? 1U : 0U;
}
