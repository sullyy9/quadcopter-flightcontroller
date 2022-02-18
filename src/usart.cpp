/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    usart.c
 * @brief   Module for controlling the USART peripheral
 *
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define USART1_TX_BUFFER_SIZE      512
#define USART1_TX_BUFFER_HALF_SIZE (USART1_TX_BUFFER_SIZE / 2)

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile uint8_t  usart1_tx_buffer[USART1_TX_BUFFER_SIZE];
static volatile uint16_t usart1_tx_write_ptr  = USART1_TX_BUFFER_HALF_SIZE;
static uint16_t          usart1_tx_free_space = USART1_TX_BUFFER_HALF_SIZE;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any used USART peripherals and corresponding DMA channels.
 */
void usart_initialise(void)
{
    /*
     * Setup USART1 for transmitting debug messages
     */
    LL_USART_InitTypeDef usart_init;
    usart_init.BaudRate            = 115200;
    usart_init.DataWidth           = LL_USART_DATAWIDTH_8B;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.OverSampling        = LL_USART_OVERSAMPLING_8;
    usart_init.Parity              = LL_USART_PARITY_NONE;
    usart_init.StopBits            = LL_USART_STOPBITS_1;
    usart_init.TransferDirection   = LL_USART_DIRECTION_TX;
    LL_USART_Init(USART1, &usart_init);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_Enable(USART1);

    /*
     * Setup DMA to transfer data from the transmit buffer to the
     * USART1 transmit register
     */
    LL_DMA_InitTypeDef dma_init;
    dma_init.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_init.MemoryOrM2MDstAddress  = (uint32_t)usart1_tx_buffer;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma_init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma_init.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma_init.NbData                 = USART1_TX_BUFFER_SIZE;
    dma_init.PeriphOrM2MSrcAddress =
        LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT);
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma_init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_4, &dma_init);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_4);
    LL_USART_ClearFlag_TC(USART1);

    NVIC_SetPriority(DMA1_Channel4_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Return the ammount of free space in the USART TX buffer
 * @return uint16_t Free space.
 */
uint16_t usart1_tx_free(void)
{
    return (usart1_tx_free_space);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Write a byte into the USART1 TX buffer. Start the DMA channel.
 * @param byte Byte.
 */
void usart1_tx_byte(uint8_t byte)
{
    usart1_tx_buffer[usart1_tx_write_ptr] = byte;
    usart1_tx_write_ptr++;
    usart1_tx_free_space--;

    if(LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4) == 0)
    {
        LL_USART_ClearFlag_TC(USART1);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for DMA1 channel 4. Called when the DMA has transferred half and all of the TX
 *        buffer.
 */
void usart_dma1_channel4_isr(void)
{
    /*
     * no more data in the buffer so turn off the channel.
     * all registers will be set to there initially configured
     * values
     */
    if((usart1_tx_write_ptr == 0) || (usart1_tx_write_ptr == USART1_TX_BUFFER_HALF_SIZE))
    {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        usart1_tx_write_ptr = USART1_TX_BUFFER_HALF_SIZE;
    }

    /*
     * Switch the write pointer to the half of the buffer that isn't currently being transferred.
     * Clear the half we plan to write to.
     */
    if(LL_DMA_IsActiveFlag_HT4(DMA1))
    {
        LL_DMA_ClearFlag_HT4(DMA1);
        usart1_tx_write_ptr  = 0;
        usart1_tx_free_space = USART1_TX_BUFFER_HALF_SIZE;
        for(uint16_t i = 0; i < USART1_TX_BUFFER_HALF_SIZE; i++)
        {
            usart1_tx_buffer[i] = 0;
        }
    }
    else
    {
        LL_DMA_ClearFlag_TC4(DMA1);
        usart1_tx_write_ptr  = USART1_TX_BUFFER_HALF_SIZE;
        usart1_tx_free_space = USART1_TX_BUFFER_HALF_SIZE;
        for(uint16_t i = USART1_TX_BUFFER_HALF_SIZE; i < USART1_TX_BUFFER_SIZE; i++)
        {
            usart1_tx_buffer[i] = 0;
        }
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
