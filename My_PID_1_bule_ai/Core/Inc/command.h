#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"

/*
 * VOFA+ FireWater interface.
 *
 * Commands (terminated by CR/LF, or by one UART idle frame):
 *   kp=0.30
 *   ki=0.30
 *   kd=0.01
 *   target=100
 *   pid=0.30,0.30,0.01,100
 *   reset
 */
void VOFA_RxWrite(const uint8_t *data, uint16_t length);
void VOFA_Process(void);
void VOFA_SendTelemetry(void);

#endif /* COMMAND_H */
