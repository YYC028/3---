/**
  *************************(C) COPYRIGHT 2021 LionHeart*************************
  * @file       bsp_vision.c/h
  * @brief      通过串口（usart1）与视觉计算机通讯，包含了该串口的初始化方法。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2021 LionHeart*************************
  */

#ifndef _BSP_VISION_H_
#define _BSP_VISION_H_

#include "main.h"
#include "struct_typedef.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/**
  * @brief          初始化usart1的DMA接收空闲中断
  * @param[in]      uint8_t* ：缓冲区1首地址
  * @param[in]      uint8_t* ：缓冲区2首地址
  * @param[in]      uint16_t ：缓冲区长度（字节数）
  * @retval         none
  */
extern void vision_uart_init(uint8_t* rx1_buf, uint8_t* rx2_buf, uint16_t dma_buf_len);

/**
  * @brief          禁止usart1的接收中断
  * @param          none
  * @retval         none
  */
extern void vision_disable(void);

/**
  * @brief          重启usart1的DMA接收空闲中断
  * @param[in]      uint16_t ：缓冲区长度（字节数）
  * @retval         none
  */
extern void vision_restart(uint16_t dma_buf_len);

#endif
