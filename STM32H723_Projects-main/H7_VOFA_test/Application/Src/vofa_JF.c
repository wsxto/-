#include "vofa_JF.h"
#include "string.h"
#include "math.h"
#include <stdint.h>


//协议约定+安全双缓冲设计
static Vofa_JF_DataFrame_t Data_Fill_Frame =
{
    .data = {0.0f},                             //CPU写入
    .tail = {0x00, 0x00, 0x80, 0x7F}//帧尾固定
};
static Vofa_JF_DataFrame_t *fill_buffer = &Data_Fill_Frame;

static Vofa_JF_DataFrame_t Data_Send_Frame=
{
    .data = {0.0f},                             //DMA发送
    .tail = {0x00, 0x00, 0x80, 0x7F}//帧尾固定
};
static Vofa_JF_DataFrame_t *send_buffer = &Data_Send_Frame;

/* ========= 状态 ========= */
static volatile uint8_t tx_ready = 0; //发送准备就绪标志

/* ========= 用户数据缓存 ========= */
static float shadow[VOFA_PROTOCOL_MAX_CHANNELS] = {0};

//安全约束
static float VOFA_JF_MakeSafeFloat(float value)
{
    /*
     * JustFloat 的帧尾是：
     * 00 00 80 7F
     *
     * 这个字节序列等价于 IEEE754 的 +Inf。
     * 所以真实数据里最好不要出现 NaN / Inf。
     */
    if (isfinite(value))// isfinite()：判断浮点数是否为【有限的合法数值】
    {
        return value;
    }
    else
    {
        return 0.0f;
    }
}

//float转小端
static void VOFA_JF_FloatToBytes(float value, uint8_t bytes[4])
{
    float safe_value;

    if (bytes == NULL)
    {
        return;
    }

    safe_value = VOFA_JF_MakeSafeFloat(value);

    /*
     * STM32H723 是小端架构(低->高)
     * 所以直接 memcpy 出来的 4 字节就是 VOFA+ JustFloat 需要的顺序。
     */
    memcpy(bytes, &safe_value, VOFA_PROTOCOL_FLOAT_BYTES);
}


void VOFA_Init(void)
{
    tx_ready = 1;
}
void VOFA_SetChannel(uint8_t ch, float value)
{
    if (ch < VOFA_PROTOCOL_MAX_CHANNELS)
    {
        shadow[ch] = value;
    }
}
/* ========= 填充 ========= */
static void FillBuffer(void)
{
    for (uint8_t i = 0; i < VOFA_PROTOCOL_MAX_CHANNELS; i++)
    {
        fill_buffer->data[i] = VOFA_JF_MakeSafeFloat(shadow[i]);
    }
}

//串口发送部分
void VOFA_SendDMA(UART_HandleTypeDef *huart)
{
    if (!tx_ready)
        return;

    tx_ready = 0;

    /* 1. 填数据 */
    FillBuffer();

    /* 2. 交换缓冲 */
    Vofa_JF_DataFrame_t *tmp = send_buffer;
    send_buffer = fill_buffer;
    fill_buffer = tmp;

    /* 3. DMA发送 */
    HAL_UART_Transmit_DMA(huart,
                          (uint8_t *)send_buffer,
                          VOFA_PROTOCOL_MAX_FRAME_BYTES);
}
//发送状态管理
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_ready = 1;
    }
}





