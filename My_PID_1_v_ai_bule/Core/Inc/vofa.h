#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void VOFA_Init(void);
void VOFA_Task(void);
void VOFA_USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
