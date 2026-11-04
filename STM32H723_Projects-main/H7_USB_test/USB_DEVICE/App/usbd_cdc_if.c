/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          : USB CDC Interface file
  *
  * 说明：
  * 1. 本文件基于 STM32CubeMX 生成的 CDC 模板改写。
  * 2. 当前版本不做协议解析，只提供“原始字节流”收发。
  * 3. 接收方向：
  *      USB回调 -> RX环形缓冲
  * 4. 发送方向：
  *      应用写入TX环形缓冲 -> CDC_App_TxTask() 推动发送
  * 5. 对于大于 64 字节的数据（FS bulk端点最大包长=64），
  *    USB底层会自动拆成多个包发送，你不需要手动分成 64 字节。
  *
  * 使用方法（最关键）：
  * - 在主循环里反复调用 CDC_App_TxTask()
  * - 用 CDC_App_Read() 读收到的数据
  * - 用 CDC_App_Write() 或 CDC_Transmit_HS() 发送数据
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usbd_cdc.h"
#include "usb_device.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* 你自己的应用层环形缓冲大小（和 CubeMX 里的 2048 缓冲不是一回事） */
#define CDC_APP_RX_RING_SIZE      4096U
#define CDC_APP_TX_RING_SIZE      4096U

/* 单次提交给 USB 栈的最大长度
 * 说明：
 * - 这里不是 USB 包长
 * - 底层会自动拆包（FS 下按 64 字节 bulk 包）
 * - 这个值越大，单次提交的数据越多；512 对 FS CDC 一般够用
 */
#define CDC_APP_TX_CHUNK_SIZE      512U

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* 你自己的应用层环形缓冲 */
static uint8_t s_rxRing[CDC_APP_RX_RING_SIZE];
static uint8_t s_txRing[CDC_APP_TX_RING_SIZE];

/* 环形缓冲读写索引
 * RX：USB回调写入，应用层读取
 * TX：应用层写入，CDC_App_TxTask() 消费
 */
static volatile uint32_t s_rxW = 0U;
static volatile uint32_t s_rxR = 0U;
static volatile uint32_t s_txW = 0U;
static volatile uint32_t s_txR = 0U;

/* 当前已经提交给USB栈、但还没真正发送完成的字节数 */
static volatile uint32_t s_txInflight = 0U;

/* RX 溢出丢包统计（调试用） */
static volatile uint32_t s_rxDropBytes = 0U;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @brief  计算环形缓冲已用字节数
  */
static uint32_t RB_Used(uint32_t w, uint32_t r, uint32_t size)
{
  return (w >= r) ? (w - r) : (size - r + w);
}

/**
  * @brief  计算环形缓冲剩余可用字节数
  * @note   留一个空位用于区分“满”和“空”
  */
static uint32_t RB_Free(uint32_t w, uint32_t r, uint32_t size)
{
  return (size - 1U) - RB_Used(w, r, size);
}

/**
  * @brief  向环形缓冲压入1个字节
  * @retval 1=成功, 0=失败(缓冲满)
  */
static uint8_t RB_PushByte(volatile uint32_t *w,
                           volatile uint32_t *r,
                           uint8_t *buf,
                           uint32_t size,
                           uint8_t data)
{
  uint32_t next = (*w + 1U) % size;

  if (next == *r)
  {
    return 0U; /* 满 */
  }

  buf[*w] = data;
  *w = next;
  return 1U;
}

/**
  * @brief  从环形缓冲弹出1个字节
  * @retval 1=成功, 0=失败(缓冲空)
  */
static uint8_t RB_PopByte(volatile uint32_t *w,
                          volatile uint32_t *r,
                          uint8_t *buf,
                          uint32_t size,
                          uint8_t *data)
{
  (void)w; /* 这里只是为了接口统一，实际只需要判断 r == w */

  if (*r == s_rxW && buf == s_rxRing)
  {
    return 0U;
  }

  if (*r == s_txW && buf == s_txRing)
  {
    return 0U;
  }

  *data = buf[*r];
  *r = (*r + 1U) % size;
  return 1U;
}



/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  /* 把 USB 栈的发送缓冲先设上 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0U);

  /* 设置 USB 栈接收缓冲 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);

  /* 清空应用层环形缓冲索引 */
  s_rxW = 0U;
  s_rxR = 0U;
  s_txW = 0U;
  s_txR = 0U;
  s_txInflight = 0U;
  s_rxDropBytes = 0U;

  /* 启动第一次接收（很重要） */
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
	while(0){//注释原来的
//  switch(cmd)
//  {
//  case CDC_SEND_ENCAPSULATED_COMMAND:

//    break;

//  case CDC_GET_ENCAPSULATED_RESPONSE:

//    break;

//  case CDC_SET_COMM_FEATURE:

//    break;

//  case CDC_GET_COMM_FEATURE:

//    break;

//  case CDC_CLEAR_COMM_FEATURE:

//    break;

//  /*******************************************************************************/
//  /* Line Coding Structure                                                       */
//  /*-----------------------------------------------------------------------------*/
//  /* Offset | Field       | Size | Value  | Description                          */
//  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
//  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
//  /*                                        0 - 1 Stop bit                       */
//  /*                                        1 - 1.5 Stop bits                    */
//  /*                                        2 - 2 Stop bits                      */
//  /* 5      | bParityType |  1   | Number | Parity                               */
//  /*                                        0 - None                             */
//  /*                                        1 - Odd                              */
//  /*                                        2 - Even                             */
//  /*                                        3 - Mark                             */
//  /*                                        4 - Space                            */
//  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
//  /*******************************************************************************/
//  case CDC_SET_LINE_CODING:

//    break;

//  case CDC_GET_LINE_CODING:

//    break;

//  case CDC_SET_CONTROL_LINE_STATE:

//    break;

//  case CDC_SEND_BREAK:

//    break;

//  default:
//    break;
//  }

//  return (USBD_OK);
 }
  (void)length;

  switch (cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;

    case CDC_SET_COMM_FEATURE:
      break;

    case CDC_GET_COMM_FEATURE:
      break;

    case CDC_CLEAR_COMM_FEATURE:
      break;

    case CDC_SET_LINE_CODING:
      /* pbuf[0..3] = bitrate
         pbuf[4]    = stop bits
         pbuf[5]    = parity
         pbuf[6]    = data bits
         对 USB CDC 来说，很多场景只是“形式参数”，可先不处理
       */
      break;

    case CDC_GET_LINE_CODING:
      /* 如果上位机要求读取串口参数，这里可返回默认值 */
      /* 例如 115200 8N1 */
      pbuf[0] = 0x00;
      pbuf[1] = 0xC2;
      pbuf[2] = 0x01;
      pbuf[3] = 0x00; /* 115200 = 0x0001C200 */
      pbuf[4] = 0x00; /* 1 stop bit */
      pbuf[5] = 0x00; /* no parity */
      pbuf[6] = 0x08; /* 8 data bits */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* DTR / RTS 等控制线变化 */
      break;

    case CDC_SEND_BREAK:
      break;

    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
//  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
//  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
//  return (USBD_OK);
	uint32_t i;

  /* 把 USB 收到的数据逐字节写入我们自己的 RX 环形缓冲 */
  for (i = 0U; i < *Len; i++)
  {
    if (RB_PushByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, Buf[i]) == 0U)
    {
      /* RX 环形缓冲满了：统计丢弃字节 */
      s_rxDropBytes++;
    }
  }

  /* 重新把当前缓冲交还给 USB 栈，并立刻挂起下一次接收 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, Buf);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
//  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
//  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
//  if (hcdc->TxState != 0){
//    return USBD_BUSY;
//  }
//  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
//  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS)
	uint32_t i;
  uint32_t free_space;

  if ((Buf == NULL) || (Len == 0U))
  {
    return (USBD_OK);
  }

  /* 检查 TX 环形缓冲是否有足够空间：这里采用“要么全进，要么不进” */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if ((uint32_t)Len > free_space)
  {
    return (USBD_BUSY);
  }

  /* 全部写入 TX 环形缓冲 */
  for (i = 0U; i < (uint32_t)Len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, Buf[i]);
  }

  /* 尝试启动一次发送（非阻塞） */
  CDC_App_TxTask();

  return (USBD_OK);;
  /* USER CODE END 12 */
//  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  CDC_App_Available
  *         查询当前 RX 环形缓冲中有多少字节可读
  */
/**
  * @brief  CDC_App_Available
  *         查询当前 RX 环形缓冲中有多少字节可读
  */
uint32_t CDC_App_Available(void)
{
  return RB_Used(s_rxW, s_rxR, CDC_APP_RX_RING_SIZE);
}

/**
  * @brief  CDC_App_TxFree
  *         查询当前 TX 环形缓冲剩余可写空间
  */
uint32_t CDC_App_TxFree(void)
{
  return RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
}

/**
  * @brief  CDC_App_GetRxDropped
  *         查询接收时因 RX 缓冲满而丢弃的字节数
  */
uint32_t CDC_App_GetRxDropped(void)
{
  return s_rxDropBytes;
}

/**
  * @brief  CDC_App_Read
  *         从 RX 环形缓冲中读出最多 max_len 个字节
  * @param  buf     输出缓冲
  * @param  max_len 最多读取多少字节
  * @retval 实际读取到的字节数
  *
  * 说明：
  * - 这是“原始字节流读取”
  * - 不保证一次就是一帧
  * - 你后续如果有协议，再在更上层做分帧解析
  */
uint32_t CDC_App_Read(uint8_t *buf, uint32_t max_len)
{
  uint32_t count = 0U;
  uint8_t  data;

  if ((buf == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  while (count < max_len)
  {
    if (RB_PopByte(&s_rxW, &s_rxR, s_rxRing, CDC_APP_RX_RING_SIZE, &data) == 0U)
    {
      break; /* 没数据了 */
    }

    buf[count++] = data;
  }

  return count;
}

/**
  * @brief  CDC_App_Write
  *         把一段数据写入 TX 环形缓冲，等待后续发送
  * @param  buf 数据指针
  * @param  len 数据长度
  * @retval 1=成功入队, 0=失败(空间不足)
  *
  * 说明：
  * - 和 CDC_Transmit_HS() 功能类似
  * - 这是更直观的应用层接口
  */
uint8_t CDC_App_Write(const uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint32_t free_space;

  if ((buf == NULL) || (len == 0U))
  {
    return 1U;
  }

  /* 要么全部写入，要么一个字节都不写 */
  free_space = RB_Free(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (len > free_space)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    (void)RB_PushByte(&s_txW, &s_txR, s_txRing, CDC_APP_TX_RING_SIZE, buf[i]);
  }

  /* 试着启动发送 */
  CDC_App_TxTask();

  return 1U;
}

/**
  * @brief  CDC_App_TxTask
  *         推动 TX 环形缓冲中的数据通过 USB 实际发送
  *
  * 说明：
  * - 这是整个“连续发送不被 USBD_BUSY 卡住”的关键函数
  * - 必须在主循环里反复调用
  * - 它是非阻塞的：每次只做必要的状态推进
  */
void CDC_App_TxTask(void)
{
  USBD_CDC_HandleTypeDef *hcdc;
  uint32_t used;
  uint32_t linear_len;

  /* CDC 类还没准备好，直接返回 */
  if (hUsbDeviceHS.pClassData == NULL)
  {
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

  /* 情况1：上一次提交的数据还在飞（已交给USB栈，还没确认发完） */
  if (s_txInflight != 0U)
  {
    if (hcdc->TxState != 0U)
    {
      /* 还没发完，本次先退出 */
      return;
    }

    /* TxState == 0，说明上次已完成，把读指针向前推进 */
    s_txR = (s_txR + s_txInflight) % CDC_APP_TX_RING_SIZE;
    s_txInflight = 0U;
  }
  else
  {
    /* 正常来说如果我们统一通过本模块发送，这里应当也是空闲
     * 如果外部有人绕过本模块直接操作底层发送，做个保护
     */
    if (hcdc->TxState != 0U)
    {
      return;
    }
  }

  /* 看看还有没有待发数据 */
  used = RB_Used(s_txW, s_txR, CDC_APP_TX_RING_SIZE);
  if (used == 0U)
  {
    return; /* 没有待发数据 */
  }

  /* 只发送一段“连续内存”
   * 如果环形缓冲在尾部绕回，这里只发尾部这段
   * 等下次再发开头那段
   */
  if (s_txW > s_txR)
  {
    linear_len = s_txW - s_txR;
  }
  else
  {
    linear_len = CDC_APP_TX_RING_SIZE - s_txR;
  }

  /* 限制单次提交长度 */
  if (linear_len > CDC_APP_TX_CHUNK_SIZE)
  {
    linear_len = CDC_APP_TX_CHUNK_SIZE;
  }

  /* 把这一段交给 USB CDC 底层 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, &s_txRing[s_txR], (uint16_t)linear_len);

  if (USBD_CDC_TransmitPacket(&hUsbDeviceHS) == USBD_OK)
  {
    /* 记录“已提交但未完成”的长度
     * 真正完成后，要等下次 CDC_App_TxTask() 检测 TxState==0 再推进 txR
     */
    s_txInflight = linear_len;
  }
  else
  {
    /* 如果底层此刻没收下（极少数情况），本次不推进索引，等下次再试 */
  }
}


/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
