#ifndef __TRANSPORT_LOAD_H
#define __TRANSPORT_LOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void TransportLoad_Reset(void);
void TransportLoad_Update(uint32_t now);
uint8_t TransportLoad_IsReady(uint32_t now);
uint8_t TransportLoad_GetReadyLatched(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSPORT_LOAD_H */
