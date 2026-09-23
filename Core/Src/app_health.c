#include "app_health.h"
#include "app_config.h"

static AppHealthSnapshot_t s_snapshot;
static uint32_t s_last_loop_tick = 0;
static uint32_t s_task_start_tick[APP_HEALTH_TASK_COUNT];

void AppHealth_Init(void)
{
  s_snapshot.loop_count = 0;
  s_snapshot.last_loop_ms = 0;
  s_snapshot.max_loop_ms = 0;
  s_snapshot.loop_overrun_count = 0;
  s_last_loop_tick = 0;

  for (uint8_t i = 0; i < APP_HEALTH_TASK_COUNT; i++)
  {
    s_snapshot.task_max_ms[i] = 0;
    s_task_start_tick[i] = 0;
  }
}

void AppHealth_LoopBegin(uint32_t now)
{
  if (s_last_loop_tick != 0U)
  {
    s_snapshot.last_loop_ms = (uint32_t)(now - s_last_loop_tick);

    if (s_snapshot.last_loop_ms > s_snapshot.max_loop_ms)
    {
      s_snapshot.max_loop_ms = s_snapshot.last_loop_ms;
    }

    if (s_snapshot.last_loop_ms > APP_HEALTH_LOOP_WARN_MS)
    {
      s_snapshot.loop_overrun_count++;
    }
  }

  s_last_loop_tick = now;
  s_snapshot.loop_count++;
}

void AppHealth_TaskBegin(AppHealthTask_t task, uint32_t now)
{
  if (task >= APP_HEALTH_TASK_COUNT)
  {
    return;
  }

  s_task_start_tick[task] = now;
}

void AppHealth_TaskEnd(AppHealthTask_t task, uint32_t now)
{
  uint32_t elapsed_ms;

  if ((task >= APP_HEALTH_TASK_COUNT) || (s_task_start_tick[task] == 0U))
  {
    return;
  }

  elapsed_ms = (uint32_t)(now - s_task_start_tick[task]);
  if (elapsed_ms > s_snapshot.task_max_ms[task])
  {
    s_snapshot.task_max_ms[task] = elapsed_ms;
  }
}

void AppHealth_GetSnapshot(AppHealthSnapshot_t *snapshot)
{
  if (snapshot == 0)
  {
    return;
  }

  *snapshot = s_snapshot;
}
