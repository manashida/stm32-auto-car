#include "transport_nav.h"
#include "tracking.h"
#include "transport_config.h"

static uint32_t s_return_search_start_tick = 0;
static uint32_t s_return_search_ms = 0;
static uint32_t s_return_search_step_tick = 0;
static uint8_t s_return_search_rotating = 1U;
static int8_t s_return_search_direction = 1;
static uint8_t s_return_search_span_steps = 1U;
static uint8_t s_return_search_steps_in_span = 0U;

uint8_t TransportNav_ReadTrackRaw(void)
{
  return Tracking_ReadRaw();
}

uint8_t TransportNav_CalcTankSpeed(uint8_t raw,
                                   int16_t *left_speed,
                                   int16_t *right_speed,
                                   uint8_t *line_valid)
{
  return Tracking_CalcTankSpeed(raw, left_speed, right_speed, line_valid);
}

uint8_t TransportNav_ReadTrack(TransportNavTrack_t *track)
{
  if (track == 0)
  {
    return 0U;
  }

  track->raw = TransportNav_ReadTrackRaw();
  track->line_valid = 0U;
  track->left_speed = 0;
  track->right_speed = 0;

  return TransportNav_CalcTankSpeed(track->raw,
                                    &track->left_speed,
                                    &track->right_speed,
                                    &track->line_valid);
}

uint8_t TransportNav_IsLineDetected(uint8_t raw)
{
  return ((raw & (TRACKING_O1_MASK |
                  TRACKING_O2_MASK |
                  TRACKING_O3_MASK |
                  TRACKING_O4_MASK)) != 0U) ? 1U : 0U;
}

uint8_t TransportNav_IsLineCentered(uint8_t raw)
{
  uint8_t center = (uint8_t)(raw & (TRACKING_O2_MASK | TRACKING_O3_MASK));
  uint8_t edge = (uint8_t)(raw & (TRACKING_O1_MASK | TRACKING_O4_MASK));

  return ((center != 0U) && (edge == 0U)) ? 1U : 0U;
}

void TransportNav_ResetTrackController(void)
{
  Tracking_ResetController();
}

static void TransportNav_AdvanceReturnSearchStep(void)
{
  if (s_return_search_steps_in_span < s_return_search_span_steps)
  {
    s_return_search_steps_in_span++;
  }

  if (s_return_search_steps_in_span < s_return_search_span_steps)
  {
    return;
  }

  s_return_search_steps_in_span = 0U;
  s_return_search_direction = (int8_t)-s_return_search_direction;

  if (s_return_search_span_steps < TRANSPORT_RETURN_SEARCH_MAX_SPAN_STEPS)
  {
    s_return_search_span_steps++;
  }
}

void TransportNav_ReturnSearchClear(void)
{
  s_return_search_start_tick = 0;
  s_return_search_ms = 0;
  s_return_search_step_tick = 0;
  s_return_search_rotating = 1U;
  s_return_search_direction = 1;
  s_return_search_span_steps = 1U;
  s_return_search_steps_in_span = 0U;
}

void TransportNav_ReturnSearchReset(uint32_t now)
{
  s_return_search_start_tick = now;
  s_return_search_ms = 0;
  s_return_search_step_tick = now;
  s_return_search_rotating = 1U;
  s_return_search_direction = 1;
  s_return_search_span_steps = 1U;
  s_return_search_steps_in_span = 0U;
}

void TransportNav_ReturnSearchUpdate(uint32_t now)
{
  s_return_search_ms = (s_return_search_start_tick == 0U) ? 0U :
                       (uint32_t)(now - s_return_search_start_tick);
}

uint32_t TransportNav_ReturnSearchGetElapsedMs(void)
{
  return s_return_search_ms;
}

uint8_t TransportNav_ReturnSearchTimedOut(void)
{
  return (s_return_search_ms >= TRANSPORT_RETURN_SEARCH_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t TransportNav_ReturnSearchLineFound(uint8_t line_valid)
{
  return ((line_valid != 0U) &&
          (s_return_search_ms >= TRANSPORT_RETURN_SEARCH_MIN_SCAN_MS)) ? 1U : 0U;
}

uint8_t TransportNav_ReturnSearchStep(uint32_t now, int16_t *left_speed, int16_t *right_speed)
{
  if ((left_speed == 0) || (right_speed == 0))
  {
    return 0U;
  }

  if (s_return_search_step_tick == 0U)
  {
    s_return_search_step_tick = now;
    s_return_search_rotating = 1U;
  }

  if (s_return_search_rotating != 0U)
  {
    if ((uint32_t)(now - s_return_search_step_tick) >= TRANSPORT_RETURN_SEARCH_STEP_MS)
    {
      s_return_search_rotating = 0U;
      s_return_search_step_tick = now;
      *left_speed = 0;
      *right_speed = 0;
      TransportNav_AdvanceReturnSearchStep();
      return 0U;
    }

    if (s_return_search_direction < 0)
    {
      *left_speed = (int16_t)-TRANSPORT_RETURN_SEARCH_SPEED;
      *right_speed = TRANSPORT_RETURN_SEARCH_SPEED;
    }
    else
    {
      *left_speed = TRANSPORT_RETURN_SEARCH_SPEED;
      *right_speed = (int16_t)-TRANSPORT_RETURN_SEARCH_SPEED;
    }

    return 1U;
  }

  *left_speed = 0;
  *right_speed = 0;

  if ((uint32_t)(now - s_return_search_step_tick) >= TRANSPORT_RETURN_SEARCH_PAUSE_MS)
  {
    s_return_search_rotating = 1U;
    s_return_search_step_tick = now;
  }

  return 0U;
}
