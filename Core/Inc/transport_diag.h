#ifndef __TRANSPORT_DIAG_H
#define __TRANSPORT_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "transport_task.h"
#include <stdint.h>

typedef enum
{
  TRANSPORT_STOP_REASON_NONE = 0,
  TRANSPORT_STOP_REASON_LOAD_CHECK,
  TRANSPORT_STOP_REASON_NO_LINE,
  TRANSPORT_STOP_REASON_END_MARK,
  TRANSPORT_STOP_REASON_OBSTACLE,
  TRANSPORT_STOP_REASON_ARRIVED,
  TRANSPORT_STOP_REASON_ERROR,
  TRANSPORT_STOP_REASON_EMERGENCY,
  TRANSPORT_STOP_REASON_RETURN_LOST,
  TRANSPORT_STOP_REASON_RETURN_TIMEOUT
} TransportStopReason_t;

typedef struct
{
  const char *state_name;
  TransportStopReason_t reason;
  uint8_t is_return_state;
  uint8_t track_raw;
  uint8_t track_line_valid;
  uint32_t line_lost_ms;
  uint32_t end_mark_ms;
  uint32_t run_elapsed_ms;
  uint32_t return_elapsed_ms;
  uint32_t return_lost_ms;
  uint32_t return_search_ms;
  int32_t outbound_target_pulse;
  int32_t return_pulse_abs;
  int32_t return_turn_pulse_abs;
  int32_t return_start_mark_min_pulse;
  uint8_t return_left_end_mark;
  uint8_t end_mark_active;
  uint8_t return_start_mark_active;
  uint16_t distance_cm;
  int32_t weight_g;
  uint8_t load_ready;
  int16_t left_speed;
  int16_t right_speed;
  uint8_t obstacle_active;
} TransportDiagSnapshot_t;

const char *TransportDiag_GetStopReasonName(TransportStopReason_t reason);
void TransportDiag_Print(uint32_t now, const TransportDiagSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSPORT_DIAG_H */
