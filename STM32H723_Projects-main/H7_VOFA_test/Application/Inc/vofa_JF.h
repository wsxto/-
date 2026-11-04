#ifndef __VOFA_JF_H_
#define __VOFA_JF_H_
/*数据帧 + 帧尾 0x00, 0x00, 0x80, 0x7F


*/

#include "main.h"
#include <stdint.h>

#include "usart.h"
#include "dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 最大支持的 float 通道数量。
 */
#define VOFA_PROTOCOL_MAX_CHANNELS      4U

/*
 * 一个 float 的字节数。
 */
#define VOFA_PROTOCOL_FLOAT_BYTES       4U

/*
 * JustFloat 帧尾长度。
 */
#define VOFA_PROTOCOL_TAIL_BYTES        4U

/*
 * 最大帧长度。
 */
#define VOFA_PROTOCOL_MAX_FRAME_BYTES   \
    (VOFA_PROTOCOL_MAX_CHANNELS * VOFA_PROTOCOL_FLOAT_BYTES + VOFA_PROTOCOL_TAIL_BYTES)


typedef struct
{
    float data[VOFA_PROTOCOL_MAX_CHANNELS];
    uint8_t tail[VOFA_PROTOCOL_TAIL_BYTES];
}Vofa_JF_DataFrame_t;

void VOFA_Init(void);//初始化
void VOFA_SetChannel(uint8_t ch, float value);//设置数据
void VOFA_SendDMA(UART_HandleTypeDef *huart);//发送接口







#ifdef __cplusplus
}
#endif

#endif
