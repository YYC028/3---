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

#include "bsp_vision.h"
#include "usart.h"


/**
  * @brief          初始化usart1的DMA接收空闲中断
  * @param[in]      uint8_t* ：缓冲区1首地址
  * @param[in]      uint8_t* ：缓冲区2首地址
  * @param[in]      uint16_t ：缓冲区长度（字节数）
  * @retval         none
  */
void vision_uart_init(uint8_t *rx1_buf,uint8_t *rx2_buf, uint16_t dma_buf_len)
{
    //使能DMA串口发送和接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    //hdma_usart1_rx.Instance->NDTR = dma_buf_len;
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, dma_buf_len);
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);


    //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //HAL_UART_Receive_DMA(&huart1, rx1_buf, dma_buf_len);
}


/**
  * @brief          禁止usart1的接收中断
  * @param          none
  * @retval         none
  */
void vision_disable(void)
{
    __HAL_UART_DISABLE(&huart1);
}


/**
  * @brief          重启usart1的DMA接收空闲中断
  * @param[in]      uint16_t ：缓冲区长度（字节数）
  * @retval         none
  */
void vision_restart(uint16_t dma_buf_len)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_len;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);
}
