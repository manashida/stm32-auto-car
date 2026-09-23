#include "transport_diag.h"
#include "app_config.h"
#include <stdio.h>

#if TRANSPORT_DEBUG_ENABLE
static uint32_t s_debug_last_tick = 0;
#endif

const char *TransportDiag_GetStopReasonName(TransportStopReason_t reason)
{
  switch (reason)
  {
    case TRANSPORT_STOP_REASON_NONE:
      return "NONE";
    case TRANSPORT_STOP_REASON_LOAD_CHECK:
      return "LOAD_CHECK";
    case TRANSPORT_STOP_REASON_NO_LINE:
      return "NO_LINE";
    case TRANSPORT_STOP_REASON_END_MARK:
      return "END_MARK";
    case TRANSPORT_STOP_REASON_OBSTACLE:
      return "OBSTACLE";
    case TRANSPORT_STOP_REASON_ARRIVED:
      return "ARRIVED";
    case TRANSPORT_STOP_REASON_ERROR:
      return "ERROR";
    case TRANSPORT_STOP_REASON_EMERGENCY:
      return "EMERGENCY";
    case TRANSPORT_STOP_REASON_RETURN_LOST:
      return "RETURN_LOST";
    case TRANSPORT_STOP_REASON_RETURN_TIMEOUT:
      return "RETURN_TIMEOUT";
    default:
      return "UNKNOWN";
  }
}

void TransportDiag_Print(uint32_t now, const TransportDiagSnapshot_t *snapshot)
{
#if TRANSPORT_DEBUG_ENABLE
  if (snapshot == 0)
  {
    return;
  }

  if ((uint32_t)(now - s_debug_last_tick) < TRANSPORT_DEBUG_PERIOD_MS)
  {
    return;
  }

  s_debug_last_tick = now;
  printf("TRP state=%s reason=%s ret=%u raw=0x%02X line=%u lost=%lu end_ms=%lu end_mark=%u run=%lu ret_run=%lu ret_lost=%lu ret_search=%lu out_pulse=%ld ret_pulse=%ld turn_pulse=%ld start_need=%ld left_end=%u start_mark=%u dis=%u wt=%ld ready=%u L=%d R=%d obs=%u\r\n",
         snapshot->state_name,
         TransportDiag_GetStopReasonName(snapshot->reason),
         (unsigned)snapshot->is_return_state,
         (unsigned)snapshot->track_raw,
         (unsigned)snapshot->track_line_valid,
         (unsigned long)snapshot->line_lost_ms,
         (unsigned long)snapshot->end_mark_ms,
         (unsigned)snapshot->end_mark_active,
         (unsigned long)snapshot->run_elapsed_ms,
         (unsigned long)snapshot->return_elapsed_ms,
         (unsigned long)snapshot->return_lost_ms,
         (unsigned long)snapshot->return_search_ms,
         (long)snapshot->outbound_target_pulse,
         (long)snapshot->return_pulse_abs,
         (long)snapshot->return_turn_pulse_abs,
         (long)snapshot->return_start_mark_min_pulse,
         (unsigned)snapshot->return_left_end_mark,
         (unsigned)snapshot->return_start_mark_active,
         snapshot->distance_cm,
         (long)snapshot->weight_g,
         (unsigned)snapshot->load_ready,
         (int)snapshot->left_speed,
         (int)snapshot->right_speed,
         (unsigned)snapshot->obstacle_active);
#else
  (void)now;
  (void)snapshot;
#endif
}
