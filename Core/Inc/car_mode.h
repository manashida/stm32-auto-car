#ifndef __CAR_MODE_H
#define __CAR_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MODE_MANUAL = 0,
  MODE_AUTO,
  MODE_TRACK,
  MODE_FOLLOW,
  MODE_TRANSPORT,
  MODE_EMERGENCY_STOP
} CarMode_t;

extern volatile CarMode_t g_car_mode;

#ifdef __cplusplus
}
#endif

#endif /* __CAR_MODE_H */
