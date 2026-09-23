#include "transport_task.h"
#include "car.h"
#include "car_mode.h"
#include "encoder.h"
#include "motion_control.h"
#include "servo.h"
#include "transport_config.h"
#include "transport_diag.h"
#include "transport_load.h"
#include "transport_nav.h"
#include "transport_prompt.h"
#include "ultrasonic.h"

#define TRANSPORT_SERVO_CLOSE_ANGLE         SERVO_CLOSE_ANGLE
#define TRANSPORT_SERVO_OPEN_ANGLE          SERVO_OPEN_ANGLE

extern volatile CarMode_t g_car_mode;
extern int16_t g_car_speed;
extern uint16_t g_distance_cm;
extern int32_t g_weight_g;

static TransportState_t s_transport_state = TRANSPORT_IDLE;
static uint32_t s_state_start_tick = 0;
static uint8_t s_unload_retry_count = 0;
static uint8_t s_error_active = 0;
static uint32_t s_run_start_tick = 0;
static uint32_t s_line_lost_start_tick = 0;
static uint32_t s_line_lost_ms = 0;
static uint8_t s_line_lost_active = 0;
static uint32_t s_end_mark_start_tick = 0;
static uint32_t s_end_mark_ms = 0;
static uint8_t s_end_mark_active = 0;
static uint8_t s_track_raw = 0;
static uint8_t s_track_line_valid = 0;
static int16_t s_track_left_speed = 0;
static int16_t s_track_right_speed = 0;
static int32_t s_turn_before_unload_pulse_abs = 0;
static uint8_t s_obstacle_active = 0;
static TransportState_t s_obstacle_resume_state = TRANSPORT_RUN_TRACK;
static uint32_t s_return_start_tick = 0;
static uint32_t s_return_lost_start_tick = 0;
static uint32_t s_return_lost_ms = 0;
static uint8_t s_return_lost_active = 0;
static uint32_t s_return_line_center_tick = 0;
static uint8_t s_return_line_center_active = 0U;
static uint8_t s_return_line_ready_after_turn = 0U;
static uint8_t s_return_left_end_mark = 0U;
static int32_t s_return_progress_pulse_abs = 0;
static int32_t s_return_pulse_abs = 0;
static int32_t s_return_turn_pulse_abs = 0;
static int32_t s_outbound_target_pulse = 0;
static uint32_t s_return_start_mark_tick = 0;
static uint8_t s_return_start_mark_active = 0;
static TransportStopReason_t s_stop_reason = TRANSPORT_STOP_REASON_NONE;

static void Transport_ResetEndMark(void);

static uint32_t Transport_Now(void)
{
  return HAL_GetTick();
}

static uint8_t Transport_Elapsed(uint32_t duration_ms)
{
  return ((uint32_t)(Transport_Now() - s_state_start_tick) >= duration_ms) ? 1U : 0U;
}

static void Transport_SetState(TransportState_t state)
{
  uint8_t changed = (s_transport_state != state) ? 1U : 0U;

  s_transport_state = state;
  s_state_start_tick = Transport_Now();

  if (changed != 0U)
  {
    TransportPrompt_ApplyState(state);
  }
}

static void Transport_StopMotion(void)
{
  g_car_speed = 0;
  Motion_Submit(MOTION_OWNER_TRANSPORT, MOTION_CMD_STOP, 0, 0U);
}

static void Transport_SubmitTankMotion(int16_t left_speed, int16_t right_speed)
{
  Motion_SubmitTank(MOTION_OWNER_TRANSPORT,
                    left_speed,
                    right_speed,
                    TRANSPORT_MOTION_COMMAND_TIMEOUT_MS);
}

static void Transport_ResetRunTrackState(void)
{
  s_run_start_tick = 0;
  s_line_lost_start_tick = 0;
  s_line_lost_ms = 0;
  s_line_lost_active = 0;
  Transport_ResetEndMark();
  s_track_raw = 0;
  s_track_line_valid = 0;
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_turn_before_unload_pulse_abs = 0;
  s_obstacle_active = 0;
  s_obstacle_resume_state = TRANSPORT_RUN_TRACK;
  s_return_start_tick = 0;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  TransportNav_ReturnSearchClear();
  s_return_lost_active = 0;
  s_return_line_center_tick = 0;
  s_return_line_center_active = 0U;
  s_return_line_ready_after_turn = 0U;
  s_return_left_end_mark = 0U;
  s_return_progress_pulse_abs = 0;
  s_return_pulse_abs = 0;
  s_return_turn_pulse_abs = 0;
  s_outbound_target_pulse = 0;
  s_return_start_mark_tick = 0;
  s_return_start_mark_active = 0U;
}

static float Transport_ReadWeightLimited(void)
{
  return (float)g_weight_g;
}

static uint8_t Transport_UnloadIsComplete(float weight_g)
{
  return (weight_g < TRANSPORT_UNLOAD_EMPTY_G) ? 1U : 0U;
}

static int32_t Transport_Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t Transport_GetAveragePulseAbs(void)
{
  return (Transport_Abs32(Encoder_GetLeftPulse()) +
          Transport_Abs32(Encoder_GetRightPulse())) / 2;
}

static void Transport_SaveReturnSegmentProgress(void)
{
  int32_t segment_pulse = Transport_GetAveragePulseAbs();

  if (segment_pulse > 0)
  {
    s_return_progress_pulse_abs += segment_pulse;
  }

  Encoder_Reset();
  s_return_pulse_abs = s_return_progress_pulse_abs;
}

static int32_t Transport_GetReturnStartMinPulse(void)
{
  int32_t min_pulse = TRANSPORT_RETURN_START_MIN_PULSE;

  if (s_outbound_target_pulse > 0)
  {
    int32_t ratio_pulse =
      (int32_t)(((int64_t)s_outbound_target_pulse * TRANSPORT_RETURN_START_RATIO_PERCENT) / 100);

    if (ratio_pulse > min_pulse)
    {
      min_pulse = ratio_pulse;
    }
  }

  return min_pulse;
}

static void Transport_ResetReturnStartMark(void)
{
  s_return_start_mark_tick = 0;
  s_return_start_mark_active = 0U;
}

static uint8_t Transport_DistanceIsObstacle(uint16_t distance_cm)
{
  return ((Ultrasonic_IsDistanceValid(distance_cm) != 0U) &&
          (distance_cm <= TRANSPORT_OBSTACLE_STOP_CM)) ? 1U : 0U;
}

static uint8_t Transport_DistanceCanResume(uint16_t distance_cm)
{
  return ((Ultrasonic_IsDistanceValid(distance_cm) != 0U) &&
          (distance_cm > TRANSPORT_OBSTACLE_RESUME_CM)) ? 1U : 0U;
}

static uint8_t Transport_EndMarkGateOpen(uint32_t run_elapsed_ms)
{
  return ((run_elapsed_ms >= TRANSPORT_MIN_RUN_BEFORE_END_MS) &&
          (Transport_GetAveragePulseAbs() >= TRANSPORT_MIN_PULSE_BEFORE_END)) ? 1U : 0U;
}

static void Transport_ResetEndMark(void)
{
  s_end_mark_active = 0U;
  s_end_mark_start_tick = 0;
  s_end_mark_ms = 0;
}

static uint8_t Transport_IsEndMarkRaw(uint8_t raw)
{
  return ((raw & 0x0FU) == TRANSPORT_END_MARK_RAW) ? 1U : 0U;
}

static uint8_t Transport_ReadTrackForMotion(void)
{
  TransportNavTrack_t track;

  if (TransportNav_ReadTrack(&track) == 0U)
  {
    Transport_StopMotion();
    s_track_raw = track.raw;
    s_track_line_valid = 0U;
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    return 0U;
  }

  s_track_raw = track.raw;
  s_track_line_valid = track.line_valid;
  s_track_left_speed = track.left_speed;
  s_track_right_speed = track.right_speed;
  return 1U;
}

static uint8_t Transport_CalcTrackForMotion(uint8_t raw)
{
  if (TransportNav_CalcTankSpeed(raw,
                                 &s_track_left_speed,
                                 &s_track_right_speed,
                                 &s_track_line_valid) == 0U)
  {
    Transport_StopMotion();
    s_track_line_valid = 0U;
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    return 0U;
  }

  return 1U;
}

static void Transport_EnterErrorWithReason(TransportStopReason_t reason)
{
  Transport_StopMotion();
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  s_error_active = 1U;
  s_stop_reason = reason;
  Transport_ResetRunTrackState();
  Transport_SetState(TRANSPORT_ERROR);
}

static void Transport_EnterError(void)
{
  Transport_EnterErrorWithReason(TRANSPORT_STOP_REASON_ERROR);
}

static void Transport_EnterFinish(void)
{
  Transport_StopMotion();
  Car_Stop();
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  TransportLoad_Reset();
  s_unload_retry_count = 0;
  s_error_active = 0;
  Transport_ResetRunTrackState();
  Transport_SetState(TRANSPORT_FINISH);
}

static void Transport_EnterRunTrack(void)
{
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  Transport_ResetRunTrackState();
  Encoder_Reset();
  TransportNav_ResetTrackController();
  s_run_start_tick = Transport_Now();
  Transport_SetState(TRANSPORT_RUN_TRACK);
}

static void Transport_EnterArrivedLostLine(void)
{
  Transport_StopMotion();
  s_outbound_target_pulse = Transport_GetAveragePulseAbs();
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_obstacle_active = 0;
  Transport_ResetEndMark();
  if (s_stop_reason != TRANSPORT_STOP_REASON_END_MARK)
  {
    s_stop_reason = TRANSPORT_STOP_REASON_ARRIVED;
  }
  Transport_SetState(TRANSPORT_ARRIVED_LOST_LINE);
}

static void Transport_EnterObstacle(TransportState_t resume_state)
{
  Transport_StopMotion();
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_obstacle_active = 1U;
  s_line_lost_active = 0U;
  s_line_lost_start_tick = 0;
  s_line_lost_ms = 0;
  Transport_ResetEndMark();
  s_return_lost_active = 0U;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  s_return_line_center_tick = 0;
  s_return_line_center_active = 0U;
  Transport_ResetReturnStartMark();
  s_obstacle_resume_state = resume_state;
  s_stop_reason = TRANSPORT_STOP_REASON_OBSTACLE;
  Transport_SetState(TRANSPORT_OBSTACLE);
}

static void Transport_EnterReturnSearchLine(void)
{
  uint32_t now = Transport_Now();

  Transport_StopMotion();
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_track_line_valid = 0U;
  s_return_line_ready_after_turn = 0U;
  TransportNav_ReturnSearchReset(now);
  s_return_lost_active = 0U;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  Transport_ResetReturnStartMark();
  TransportNav_ResetTrackController();
  Transport_SetState(TRANSPORT_RETURN_SEARCH_LINE);
}

static void Transport_EnterReturnTurn(void)
{
  Transport_StopMotion();
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_return_turn_pulse_abs = 0;
  s_return_line_ready_after_turn = 0U;
  Encoder_Reset();
  Transport_SetState(TRANSPORT_RETURN_TURN);
}

static void Transport_EnterHomeTurn(void)
{
  Transport_StopMotion();
  s_track_left_speed = 0;
  s_track_right_speed = 0;
  s_return_turn_pulse_abs = 0;
  Encoder_Reset();
  Transport_SetState(TRANSPORT_HOME_TURN);
}

static void Transport_EnterReturnTrack(void)
{
  Transport_StopMotion();
  Encoder_Reset();
  s_return_start_tick = Transport_Now();
  s_return_lost_active = 0U;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  s_return_line_center_tick = 0;
  s_return_line_center_active = 0U;
  s_return_line_ready_after_turn = 0U;
  s_return_left_end_mark = 0U;
  s_return_progress_pulse_abs = 0;
  s_return_pulse_abs = 0;
  Transport_ResetReturnStartMark();
  TransportNav_ResetTrackController();
  Transport_SetState(TRANSPORT_RETURN_TRACK);
}

static void Transport_ResumeReturnTrack(void)
{
  Transport_StopMotion();
  Encoder_Reset();
  s_return_lost_active = 0U;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  s_return_line_center_tick = 0;
  s_return_line_center_active = 0U;
  s_return_line_ready_after_turn = 0U;
  s_return_pulse_abs = s_return_progress_pulse_abs;
  Transport_ResetReturnStartMark();
  TransportNav_ResetTrackController();
  Transport_SetState(TRANSPORT_RETURN_TRACK);
}

static uint8_t Transport_UpdateEndMark(uint32_t now, uint32_t run_elapsed_ms)
{
  if (Transport_EndMarkGateOpen(run_elapsed_ms) == 0U)
  {
    Transport_ResetEndMark();
    return 0U;
  }

  if (Transport_IsEndMarkRaw(s_track_raw) == 0U)
  {
    Transport_ResetEndMark();
    return 0U;
  }

  s_track_line_valid = 1U;
  s_stop_reason = TRANSPORT_STOP_REASON_END_MARK;

  if (s_end_mark_active == 0U)
  {
    s_end_mark_active = 1U;
    s_end_mark_start_tick = now;
  }

  s_end_mark_ms = (uint32_t)(now - s_end_mark_start_tick);
  if (s_end_mark_ms >= TRANSPORT_END_MARK_CONFIRM_MS)
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    Transport_EnterArrivedLostLine();
    return 1U;
  }

  s_track_left_speed = TRANSPORT_END_MARK_CONFIRM_SPEED;
  s_track_right_speed = TRANSPORT_END_MARK_CONFIRM_SPEED;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
  return 1U;
}

static uint8_t Transport_UpdateReturnStartMark(uint32_t now)
{
  uint32_t return_elapsed_ms = (s_return_start_tick == 0U) ? 0U :
                               (uint32_t)(now - s_return_start_tick);

  if (s_return_left_end_mark == 0U)
  {
    if (Transport_IsEndMarkRaw(s_track_raw) == 0U)
    {
      s_return_left_end_mark = 1U;
    }

    Transport_ResetReturnStartMark();
    return 0U;
  }

  if ((return_elapsed_ms < TRANSPORT_RETURN_MIN_MS) ||
      (s_return_pulse_abs < Transport_GetReturnStartMinPulse()) ||
      ((s_track_raw & 0x0FU) != TRANSPORT_END_MARK_RAW))
  {
    Transport_ResetReturnStartMark();
    return 0U;
  }

  s_track_line_valid = 1U;

  if (s_return_start_mark_active == 0U)
  {
    s_return_start_mark_active = 1U;
    s_return_start_mark_tick = now;
  }

  if ((uint32_t)(now - s_return_start_mark_tick) >= TRANSPORT_START_MARK_CONFIRM_MS)
  {
    Transport_EnterHomeTurn();
    return 1U;
  }

  s_track_left_speed = TRANSPORT_RETURN_START_CONFIRM_SPEED;
  s_track_right_speed = TRANSPORT_RETURN_START_CONFIRM_SPEED;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
  return 1U;
}

static void Transport_UpdateRunTrack(uint32_t now, uint16_t distance)
{
  uint32_t run_elapsed_ms = (s_run_start_tick == 0U) ? 0U : (uint32_t)(now - s_run_start_tick);

  if (Transport_DistanceIsObstacle(distance) != 0U)
  {
    Transport_EnterObstacle(TRANSPORT_RUN_TRACK);
    return;
  }

  s_obstacle_active = 0U;
  s_track_raw = TransportNav_ReadTrackRaw();

  if (Transport_UpdateEndMark(now, run_elapsed_ms) != 0U)
  {
    return;
  }

  if (Transport_CalcTrackForMotion(s_track_raw) == 0U)
  {
    return;
  }

  if (s_track_line_valid == 0U)
  {
    s_stop_reason = TRANSPORT_STOP_REASON_NO_LINE;

    if (s_line_lost_active == 0U)
    {
      s_line_lost_active = 1U;
      s_line_lost_start_tick = now;
      s_line_lost_ms = 0;
    }
    else
    {
      s_line_lost_ms = (uint32_t)(now - s_line_lost_start_tick);
    }

    if (s_line_lost_ms >= TRANSPORT_LINE_LOST_SEARCH_TIMEOUT_MS)
    {
      Transport_EnterErrorWithReason(TRANSPORT_STOP_REASON_NO_LINE);
      return;
    }

    Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
    return;
  }

  s_line_lost_active = 0U;
  s_line_lost_start_tick = 0;
  s_line_lost_ms = 0;
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
}

static void Transport_UpdateReturnTurn(void)
{
  s_return_turn_pulse_abs = Transport_GetAveragePulseAbs();

  if (s_return_turn_pulse_abs >= TRANSPORT_RETURN_TURN_180_PULSE)
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    s_track_raw = TransportNav_ReadTrackRaw();
    s_track_line_valid = TransportNav_IsLineDetected(s_track_raw);
    s_return_line_ready_after_turn = s_track_line_valid;
    Encoder_Reset();
    Transport_SetState(TRANSPORT_UNLOAD_OPEN);
    return;
  }

  if (Transport_Elapsed(TRANSPORT_RETURN_TURN_TIMEOUT_MS) != 0U)
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    s_track_raw = TransportNav_ReadTrackRaw();
    s_track_line_valid = TransportNav_IsLineDetected(s_track_raw);
    s_return_line_ready_after_turn = s_track_line_valid;
    Encoder_Reset();
    Transport_SetState(TRANSPORT_UNLOAD_OPEN);
    return;
  }

  s_track_left_speed = (int16_t)-TRANSPORT_RETURN_TURN_SPEED;
  s_track_right_speed = TRANSPORT_RETURN_TURN_SPEED;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
}

static void Transport_UpdateHomeTurn(void)
{
  s_return_turn_pulse_abs = Transport_GetAveragePulseAbs();

  if (s_return_turn_pulse_abs >= TRANSPORT_HOME_TURN_180_PULSE)
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    Encoder_Reset();
    Transport_EnterFinish();
    return;
  }

  if (Transport_Elapsed(TRANSPORT_HOME_TURN_TIMEOUT_MS) != 0U)
  {
    Transport_EnterErrorWithReason(TRANSPORT_STOP_REASON_RETURN_TIMEOUT);
    return;
  }

  s_track_left_speed = (int16_t)-TRANSPORT_HOME_TURN_SPEED;
  s_track_right_speed = TRANSPORT_HOME_TURN_SPEED;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
}

static void Transport_UpdateTurnBeforeUnload(void)
{
  uint32_t turn_elapsed_ms = (uint32_t)(Transport_Now() - s_state_start_tick);

  s_turn_before_unload_pulse_abs = Transport_GetAveragePulseAbs();

  if (s_turn_before_unload_pulse_abs >= TRANSPORT_TURN_BEFORE_UNLOAD_PULSE)
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    Encoder_Reset();
    Transport_SetState(TRANSPORT_UNLOAD_OPEN);
    return;
  }

  if (((turn_elapsed_ms >= TRANSPORT_TURN_BEFORE_UNLOAD_STALL_CHECK_MS) &&
       (s_turn_before_unload_pulse_abs < TRANSPORT_TURN_BEFORE_UNLOAD_MIN_PULSE)) ||
      (turn_elapsed_ms >= TRANSPORT_TURN_BEFORE_UNLOAD_TIMEOUT_MS))
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;
    Encoder_Reset();
    Transport_SetState(TRANSPORT_UNLOAD_OPEN);
    return;
  }

  s_track_left_speed = (int16_t)-TRANSPORT_TURN_BEFORE_UNLOAD_SPEED;
  s_track_right_speed = TRANSPORT_TURN_BEFORE_UNLOAD_SPEED;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
}

static void Transport_UpdateReturnSearchLine(uint32_t now, uint16_t distance)
{
  uint8_t line_centered = 0U;

  if (Transport_DistanceIsObstacle(distance) != 0U)
  {
    Transport_EnterObstacle(TRANSPORT_RETURN_SEARCH_LINE);
    return;
  }

  TransportNav_ReturnSearchUpdate(now);

  if (TransportNav_ReturnSearchTimedOut() != 0U)
  {
    Transport_EnterErrorWithReason(TRANSPORT_STOP_REASON_RETURN_TIMEOUT);
    return;
  }

  s_track_raw = TransportNav_ReadTrackRaw();
  s_track_line_valid = TransportNav_IsLineDetected(s_track_raw);
  line_centered = TransportNav_IsLineCentered(s_track_raw);

  if ((TransportNav_ReturnSearchLineFound(s_track_line_valid) != 0U) &&
      (line_centered != 0U))
  {
    Transport_StopMotion();
    s_track_left_speed = 0;
    s_track_right_speed = 0;

    if (s_return_line_center_active == 0U)
    {
      s_return_line_center_active = 1U;
      s_return_line_center_tick = now;
      return;
    }

    if ((uint32_t)(now - s_return_line_center_tick) >= TRANSPORT_RETURN_SEARCH_CENTER_CONFIRM_MS)
    {
      if (s_return_start_tick == 0U)
      {
        Transport_EnterReturnTrack();
      }
      else
      {
        Transport_ResumeReturnTrack();
      }
    }
    return;
  }

  s_return_line_center_active = 0U;
  s_return_line_center_tick = 0;

  if (TransportNav_ReturnSearchStep(now, &s_track_left_speed, &s_track_right_speed) != 0U)
  {
    Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
    return;
  }

  Transport_StopMotion();
}

static void Transport_UpdateReturnTrack(uint32_t now, uint16_t distance)
{
  uint32_t return_elapsed_ms = (s_return_start_tick == 0U) ? 0U :
                               (uint32_t)(now - s_return_start_tick);

  if (Transport_DistanceIsObstacle(distance) != 0U)
  {
    Transport_EnterObstacle(TRANSPORT_RETURN_TRACK);
    return;
  }

  if (return_elapsed_ms >= TRANSPORT_RETURN_TIMEOUT_MS)
  {
    Transport_EnterErrorWithReason(TRANSPORT_STOP_REASON_RETURN_TIMEOUT);
    return;
  }

  if (Transport_ReadTrackForMotion() == 0U)
  {
    return;
  }

  if (s_track_line_valid == 0U)
  {
    s_stop_reason = TRANSPORT_STOP_REASON_RETURN_LOST;

    if (s_return_lost_active == 0U)
    {
      s_return_lost_active = 1U;
      s_return_lost_start_tick = now;
      s_return_lost_ms = 0;
      Transport_SaveReturnSegmentProgress();
    }
    else
    {
      s_return_lost_ms = (uint32_t)(now - s_return_lost_start_tick);
    }

    if (s_return_lost_ms >= TRANSPORT_RETURN_LINE_LOST_TIMEOUT_MS)
    {
      Transport_EnterReturnSearchLine();
      return;
    }

    s_track_left_speed = (int16_t)-TRANSPORT_RETURN_SEARCH_SPEED;
    s_track_right_speed = TRANSPORT_RETURN_SEARCH_SPEED;
    Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
    return;
  }

  if (s_return_lost_active != 0U)
  {
    Encoder_Reset();
    s_return_lost_active = 0U;
    s_return_lost_start_tick = 0;
    s_return_lost_ms = 0;
  }

  s_return_pulse_abs = s_return_progress_pulse_abs + Transport_GetAveragePulseAbs();

  if (Transport_UpdateReturnStartMark(now) != 0U)
  {
    return;
  }

  s_return_lost_active = 0U;
  s_return_lost_start_tick = 0;
  s_return_lost_ms = 0;
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  Transport_SubmitTankMotion(s_track_left_speed, s_track_right_speed);
}

static uint8_t Transport_IsReturnState(TransportState_t state)
{
  return ((state == TRANSPORT_RETURN_TURN) ||
          (state == TRANSPORT_RETURN_SEARCH_LINE) ||
          (state == TRANSPORT_RETURN_TRACK) ||
          (state == TRANSPORT_HOME_TURN)) ? 1U : 0U;
}

static void Transport_DebugPrint(uint32_t now, uint16_t distance)
{
  uint32_t run_elapsed_ms = (s_run_start_tick == 0U) ? 0U : (uint32_t)(now - s_run_start_tick);
  uint32_t return_elapsed_ms = (s_return_start_tick == 0U) ? 0U :
                               (uint32_t)(now - s_return_start_tick);
  TransportDiagSnapshot_t snapshot = {
    .state_name = Transport_GetStateName(),
    .reason = s_stop_reason,
    .is_return_state = Transport_IsReturnState(s_transport_state),
    .track_raw = s_track_raw,
    .track_line_valid = s_track_line_valid,
    .line_lost_ms = s_line_lost_ms,
    .end_mark_ms = s_end_mark_ms,
    .run_elapsed_ms = run_elapsed_ms,
    .return_elapsed_ms = return_elapsed_ms,
    .return_lost_ms = s_return_lost_ms,
    .return_search_ms = TransportNav_ReturnSearchGetElapsedMs(),
    .outbound_target_pulse = s_outbound_target_pulse,
    .return_pulse_abs = s_return_pulse_abs,
    .return_turn_pulse_abs = s_return_turn_pulse_abs,
    .return_start_mark_min_pulse = Transport_GetReturnStartMinPulse(),
    .return_left_end_mark = s_return_left_end_mark,
    .end_mark_active = s_end_mark_active,
    .return_start_mark_active = s_return_start_mark_active,
    .distance_cm = distance,
    .weight_g = g_weight_g,
    .load_ready = TransportLoad_GetReadyLatched(),
    .left_speed = s_track_left_speed,
    .right_speed = s_track_right_speed,
    .obstacle_active = s_obstacle_active,
  };

  TransportDiag_Print(now, &snapshot);
}

void Transport_Init(void)
{
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  s_unload_retry_count = 0;
  s_error_active = 0;
  TransportLoad_Reset();
  Transport_ResetRunTrackState();
  Transport_SetState(TRANSPORT_IDLE);
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
}

void Transport_Reset(void)
{
  Transport_Stop();
}

void Transport_Start(void)
{
  s_unload_retry_count = 0;
  s_error_active = 0;
  TransportLoad_Reset();
  Transport_ResetRunTrackState();
  Encoder_Reset();
  g_car_mode = MODE_TRANSPORT;
  Transport_StopMotion();
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  Transport_SetState(TRANSPORT_START);
}

void Transport_EnterWaitLoad(void)
{
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  s_unload_retry_count = 0;
  s_error_active = 0;
  TransportLoad_Reset();
  Transport_ResetRunTrackState();
  Encoder_Reset();
  g_car_mode = MODE_TRANSPORT;
  Transport_StopMotion();
  Car_Stop();
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  Transport_SetState(TRANSPORT_WAIT_LOAD);
}

void Transport_Stop(void)
{
  s_stop_reason = TRANSPORT_STOP_REASON_NONE;
  Transport_StopMotion();
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  s_error_active = 0;
  Transport_ResetRunTrackState();
  Transport_SetState(TRANSPORT_IDLE);
}

void Transport_EmergencyStop(void)
{
  g_car_speed = 0;
  Motion_Submit(MOTION_OWNER_EMERGENCY, MOTION_CMD_STOP, 0, 0U);
  Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
  Transport_ResetRunTrackState();

  g_car_mode = MODE_EMERGENCY_STOP;
  s_stop_reason = TRANSPORT_STOP_REASON_EMERGENCY;
  Transport_SetState(TRANSPORT_EMERGENCY_STOP);
}

void Transport_Task(void)
{
  float weight = 0.0f;
  uint16_t distance = g_distance_cm;
  uint32_t now = Transport_Now();

  if (g_car_mode == MODE_EMERGENCY_STOP)
  {
    Transport_EmergencyStop();
    return;
  }

  if (g_car_mode != MODE_TRANSPORT)
  {
    return;
  }

  switch (s_transport_state)
  {
    case TRANSPORT_IDLE:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      break;

    case TRANSPORT_WAIT_LOAD:
      Transport_StopMotion();
      Car_Stop();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      TransportLoad_Update(now);
      if (Transport_IsLoadReady() != 0U)
      {
        Transport_Start();
      }
      break;

    case TRANSPORT_START:
      Transport_StopMotion();
      s_stop_reason = TRANSPORT_STOP_REASON_LOAD_CHECK;
      Transport_SetState(TRANSPORT_LOAD_CHECK);
      break;

    case TRANSPORT_LOAD_CHECK:
      weight = Transport_ReadWeightLimited();
      if (weight > TRANSPORT_LOAD_THRESHOLD_G)
      {
        Transport_EnterRunTrack();
      }
      else if (Transport_Elapsed(TRANSPORT_LOAD_CHECK_TIMEOUT_MS) != 0U)
      {
        Transport_EnterError();
      }
      else
      {
        s_stop_reason = TRANSPORT_STOP_REASON_LOAD_CHECK;
      }
      break;

    case TRANSPORT_RUN_TRACK:
      if (Transport_Elapsed(TRANSPORT_RUN_TIMEOUT_MS) != 0U)
      {
        Transport_EnterError();
      }
      else
      {
        Transport_UpdateRunTrack(now, distance);
      }
      break;

    case TRANSPORT_OBSTACLE:
      Transport_StopMotion();
      s_track_left_speed = 0;
      s_track_right_speed = 0;
      s_obstacle_active = 1U;
      s_line_lost_active = 0U;
      s_line_lost_start_tick = 0;
      s_line_lost_ms = 0;

      if (Transport_DistanceCanResume(distance) != 0U)
      {
        s_obstacle_active = 0U;
        if (s_obstacle_resume_state == TRANSPORT_RETURN_SEARCH_LINE)
        {
          TransportNav_ReturnSearchReset(now);
        }
        else if (s_obstacle_resume_state == TRANSPORT_RETURN_TRACK)
        {
          s_return_lost_active = 0U;
          s_return_lost_start_tick = 0;
          s_return_lost_ms = 0;
        }
        else
        {
          s_line_lost_active = 0U;
          s_line_lost_start_tick = 0;
          s_line_lost_ms = 0;
        }
        s_stop_reason = TRANSPORT_STOP_REASON_NONE;
        Transport_SetState(s_obstacle_resume_state);
      }
      else if (Transport_Elapsed(TRANSPORT_OBSTACLE_TIMEOUT_MS) != 0U)
      {
        Transport_EnterError();
      }
      break;

    case TRANSPORT_ARRIVED_LOST_LINE:
      Transport_StopMotion();
      if (Transport_Elapsed(TRANSPORT_ARRIVED_PROMPT_MS) == 0U)
      {
        break;
      }
      Transport_EnterReturnTurn();
      break;

    case TRANSPORT_TURN_BEFORE_UNLOAD:
      Transport_UpdateTurnBeforeUnload();
      break;

    case TRANSPORT_UNLOAD_OPEN:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_OPEN_ANGLE);
      Transport_SetState(TRANSPORT_UNLOAD_WAIT);
      break;

    case TRANSPORT_UNLOAD_WAIT:
      Transport_StopMotion();
      if (Transport_Elapsed(TRANSPORT_UNLOAD_WAIT_MS) != 0U)
      {
        Transport_SetState(TRANSPORT_UNLOAD_CHECK);
      }
      break;

    case TRANSPORT_UNLOAD_CHECK:
      Transport_StopMotion();
      weight = Transport_ReadWeightLimited();
      if (Transport_UnloadIsComplete(weight) != 0U)
      {
        Transport_SetState(TRANSPORT_UNLOAD_CLOSE);
      }
      else if (s_unload_retry_count < TRANSPORT_MAX_UNLOAD_RETRY)
      {
        s_unload_retry_count++;
        Transport_SetState(TRANSPORT_UNLOAD_RETRY_CLOSE);
      }
      else
      {
        Transport_EnterError();
      }
      break;

    case TRANSPORT_UNLOAD_RETRY_CLOSE:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      if (Transport_Elapsed(TRANSPORT_SERVO_CLOSE_WAIT_MS) != 0U)
      {
        Transport_SetState(TRANSPORT_UNLOAD_OPEN);
      }
      break;

    case TRANSPORT_UNLOAD_CLOSE:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      if (Transport_Elapsed(TRANSPORT_SERVO_CLOSE_WAIT_MS) != 0U)
      {
        if (s_return_line_ready_after_turn != 0U)
        {
          s_track_raw = TransportNav_ReadTrackRaw();
          s_track_line_valid = TransportNav_IsLineDetected(s_track_raw);
          if (s_track_line_valid != 0U)
          {
            Transport_EnterReturnTrack();
            break;
          }
        }

        Transport_EnterReturnSearchLine();
      }
      break;

    case TRANSPORT_RETURN_TURN:
      Transport_UpdateReturnTurn();
      break;

    case TRANSPORT_RETURN_SEARCH_LINE:
      Transport_UpdateReturnSearchLine(now, distance);
      break;

    case TRANSPORT_RETURN_TRACK:
      Transport_UpdateReturnTrack(now, distance);
      break;

    case TRANSPORT_HOME_TURN:
      Transport_UpdateHomeTurn();
      break;

    case TRANSPORT_FINISH:
      Transport_StopMotion();
      Car_Stop();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      if (Transport_Elapsed(TRANSPORT_FINISH_HOLD_MS) != 0U)
      {
        Transport_EnterWaitLoad();
      }
      break;

    case TRANSPORT_ERROR:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      s_error_active = 1U;
      break;

    case TRANSPORT_EMERGENCY_STOP:
    default:
      Transport_StopMotion();
      Servo_SetAngle(TRANSPORT_SERVO_CLOSE_ANGLE);
      break;
  }

  Transport_DebugPrint(now, distance);
}

TransportState_t Transport_GetState(void)
{
  return s_transport_state;
}

const char *Transport_GetStateName(void)
{
  switch (s_transport_state)
  {
    case TRANSPORT_IDLE:
      return "IDLE";
    case TRANSPORT_WAIT_LOAD:
      return "WAIT_LOAD";
    case TRANSPORT_START:
      return "START";
    case TRANSPORT_LOAD_CHECK:
      return "LOAD_CHECK";
    case TRANSPORT_RUN_TRACK:
      return "RUN_TRACK";
    case TRANSPORT_OBSTACLE:
      return "OBSTACLE";
    case TRANSPORT_ARRIVED_LOST_LINE:
      return "ARRIVED_LOST_LINE";
    case TRANSPORT_UNLOAD_OPEN:
      return "UNLOAD_OPEN";
    case TRANSPORT_UNLOAD_WAIT:
      return "UNLOAD_WAIT";
    case TRANSPORT_UNLOAD_CHECK:
      return "UNLOAD_CHECK";
    case TRANSPORT_UNLOAD_RETRY_CLOSE:
      return "UNLOAD_RETRY_CLOSE";
    case TRANSPORT_UNLOAD_CLOSE:
      return "UNLOAD_CLOSE";
    case TRANSPORT_TURN_BEFORE_UNLOAD:
      return "TURN_BEFORE_UNLOAD";
    case TRANSPORT_RETURN_TURN:
      return "RETURN_TURN";
    case TRANSPORT_RETURN_SEARCH_LINE:
      return "RETURN_SEARCH_LINE";
    case TRANSPORT_RETURN_TRACK:
      return "RETURN_TRACK";
    case TRANSPORT_HOME_TURN:
      return "HOME_TURN";
    case TRANSPORT_FINISH:
      return "FINISH";
    case TRANSPORT_ERROR:
      return "ERROR";
    case TRANSPORT_EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    default:
      return "UNKNOWN";
  }
}

uint8_t Transport_IsLoadReady(void)
{
  uint32_t now = Transport_Now();

  if (s_transport_state != TRANSPORT_WAIT_LOAD)
  {
    return 0U;
  }

  return TransportLoad_IsReady(now);
}

uint8_t Transport_IsBusy(void)
{
  return ((s_transport_state != TRANSPORT_IDLE) &&
          (s_transport_state != TRANSPORT_WAIT_LOAD) &&
          (s_transport_state != TRANSPORT_FINISH) &&
          (s_transport_state != TRANSPORT_ERROR) &&
          (s_transport_state != TRANSPORT_EMERGENCY_STOP)) ? 1U : 0U;
}

uint8_t Transport_IsError(void)
{
  return (s_error_active != 0U) ? 1U : 0U;
}
