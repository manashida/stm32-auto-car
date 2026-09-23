#ifndef __TRANSPORT_NAV_H
#define __TRANSPORT_NAV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint8_t raw;
  uint8_t line_valid;
  int16_t left_speed;
  int16_t right_speed;
} TransportNavTrack_t;

uint8_t TransportNav_ReadTrackRaw(void);
uint8_t TransportNav_CalcTankSpeed(uint8_t raw,
                                   int16_t *left_speed,
                                   int16_t *right_speed,
                                   uint8_t *line_valid);
uint8_t TransportNav_ReadTrack(TransportNavTrack_t *track);
uint8_t TransportNav_IsLineDetected(uint8_t raw);
uint8_t TransportNav_IsLineCentered(uint8_t raw);
void TransportNav_ResetTrackController(void);

void TransportNav_ReturnSearchClear(void);
void TransportNav_ReturnSearchReset(uint32_t now);
void TransportNav_ReturnSearchUpdate(uint32_t now);
uint32_t TransportNav_ReturnSearchGetElapsedMs(void);
uint8_t TransportNav_ReturnSearchTimedOut(void);
uint8_t TransportNav_ReturnSearchLineFound(uint8_t line_valid);
uint8_t TransportNav_ReturnSearchStep(uint32_t now, int16_t *left_speed, int16_t *right_speed);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSPORT_NAV_H */
