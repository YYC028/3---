#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

static uint8_t usart6_tx_dma_is_busy = 0;

void usart1_tx_dma_init(void)
{
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    usart6_tx_dma_is_busy = 1;


    ////失效DMA
    //__HAL_DMA_DISABLE(&hdma_usart6_tx);
    //while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    //{
    //    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    //}
    //__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
    //hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    //__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
    //__HAL_DMA_ENABLE(&hdma_usart6_tx);

    HAL_UART_Transmit_DMA(&huart6, data, len);
}


/**
 * @brief          重新定义串口DMA发送完成的回调函数
 * @param[out]     UART句柄指针
 * @retval         none
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart6)
    {
        clear_usart6_tx_dma_busy_sign();
    }
}


/**
  * @brief          获取USART6的发送DMA忙碌标志，若DMA正忙则返回1
  * @param[in]      none
  * @retval         uint8_t
  */
uint8_t get_usart6_tx_dma_busy_flag(void)
{
    return usart6_tx_dma_is_busy;
    //return hdma_usart6_tx.State;
}


/**
  * @brief          清零USART6的发送DMA忙碌标志，在DMA发送完成回调中断函数中调用
  * @param[in]      none
  * @retval         none
  */
void clear_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 0;
}


/**
  * @brief          置位USART6的发送DMA忙碌标志，在DMA发送前调用
  * @param[in]      none
  * @retval         none
  */
void set_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 1;
}
