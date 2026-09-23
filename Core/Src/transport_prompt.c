#include "transport_prompt.h"
#include "buzzer.h"
#include "status_led.h"

void TransportPrompt_ApplyState(TransportState_t state)
{
  switch (state)
  {
    case TRANSPORT_IDLE:
      StatusLed_SetState(STATUS_LED_IDLE);
      Buzzer_Stop();
      break;

    case TRANSPORT_WAIT_LOAD:
      StatusLed_SetState(STATUS_LED_WAIT_LOAD);
      Buzzer_Stop();
      break;

    case TRANSPORT_RUN_TRACK:
      StatusLed_SetState(STATUS_LED_TRANSPORT_RUN);
      Buzzer_Beep(180U);
      break;

    case TRANSPORT_OBSTACLE:
      StatusLed_SetState(STATUS_LED_TRANSPORT_OBSTACLE);
      Buzzer_SetPattern(80U, 250U);
      break;

    case TRANSPORT_ARRIVED_LOST_LINE:
      StatusLed_SetState(STATUS_LED_TRANSPORT_ARRIVED);
      Buzzer_BeepTimes(2U, 100U, 100U);
      break;

    case TRANSPORT_UNLOAD_OPEN:
    case TRANSPORT_UNLOAD_WAIT:
    case TRANSPORT_UNLOAD_CHECK:
    case TRANSPORT_UNLOAD_RETRY_CLOSE:
      StatusLed_SetState(STATUS_LED_UNLOADING);
      Buzzer_Stop();
      break;

    case TRANSPORT_UNLOAD_CLOSE:
      StatusLed_SetState(STATUS_LED_UNLOAD_DONE);
      Buzzer_Beep(150U);
      break;

    case TRANSPORT_TURN_BEFORE_UNLOAD:
      StatusLed_SetState(STATUS_LED_TRANSPORT_ARRIVED);
      Buzzer_Stop();
      break;

    case TRANSPORT_RETURN_TURN:
    case TRANSPORT_RETURN_SEARCH_LINE:
    case TRANSPORT_RETURN_TRACK:
    case TRANSPORT_HOME_TURN:
      StatusLed_SetState(STATUS_LED_TRANSPORT_RUN);
      Buzzer_Stop();
      break;

    case TRANSPORT_FINISH:
      StatusLed_SetState(STATUS_LED_TRANSPORT_FINISH);
      Buzzer_BeepTimes(3U, 150U, 120U);
      break;

    case TRANSPORT_ERROR:
      StatusLed_SetState(STATUS_LED_ERROR);
      Buzzer_SetPattern(250U, 250U);
      break;

    case TRANSPORT_EMERGENCY_STOP:
      StatusLed_SetState(STATUS_LED_EMERGENCY);
      Buzzer_Beep(600U);
      break;

    case TRANSPORT_START:
    case TRANSPORT_LOAD_CHECK:
      StatusLed_SetState(STATUS_LED_TRANSPORT_RUN);
      Buzzer_Stop();
      break;

    default:
      break;
  }
}
