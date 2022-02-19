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

#include "types.hpp"

#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"

#include "usart.hpp"

using namespace usart;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define TX_BUFFER_SIZE      512
#define TX_BUFFER_HALF_SIZE (TX_BUFFER_SIZE / 2)

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile uint8_t  tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t tx_write_ptr  = TX_BUFFER_HALF_SIZE;
static uint16_t          tx_free_space = TX_BUFFER_HALF_SIZE;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any used USART peripherals and corresponding DMA channels.
 */
void usart::initialise(void)
{
    /*
     * Setup USART1 for transmitting debug messages
     */
    LL_USART_InitTypeDef usart;
    usart.BaudRate            = 115200;
    usart.DataWidth           = LL_USART_DATAWIDTH_8B;
    usart.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart.OverSampling        = LL_USART_OVERSAMPLING_8;
    usart.Parity              = LL_USART_PARITY_NONE;
    usart.StopBits            = LL_USART_STOPBITS_1;
    usart.TransferDirection   = LL_USART_DIRECTION_TX;
    LL_USART_Init(USART1, &usart);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_Enable(USART1);

    /*
     * Setup DMA to transfer data from the transmit buffer to the
     * USART1 transmit register
     */
    LL_DMA_InitTypeDef dma;
    dma.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma.MemoryOrM2MDstAddress  = (uint32_t)tx_buffer;
    dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma.NbData                 = TX_BUFFER_SIZE;
    dma.PeriphOrM2MSrcAddress =
        LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT);
    dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_4, &dma);
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
uint16_t usart::tx_free(void)
{
    return (tx_free_space);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Write a byte into the USART1 TX buffer. Start the DMA channel.
 * @param byte Byte.
 */
void usart::tx_byte(uint8_t byte)
{
    tx_buffer[tx_write_ptr] = byte;
    tx_write_ptr++;
    tx_free_space--;

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
void usart::dma1_channel4_isr(void)
{
    /*
     * no more data in the buffer so turn off the channel.
     * all registers will be set to there initially configured
     * values
     */
    if((tx_write_ptr == 0) || (tx_write_ptr == TX_BUFFER_HALF_SIZE))
    {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        tx_write_ptr = TX_BUFFER_HALF_SIZE;
    }

    /*
     * Switch the write pointer to the half of the buffer that isn't currently being transferred.
     * Clear the half we plan to write to.
     */
    if(LL_DMA_IsActiveFlag_HT4(DMA1))
    {
        LL_DMA_ClearFlag_HT4(DMA1);
        tx_write_ptr  = 0;
        tx_free_space = TX_BUFFER_HALF_SIZE;
        for(uint16_t i = 0; i < TX_BUFFER_HALF_SIZE; i++)
        {
            tx_buffer[i] = 0;
        }
    }
    else
    {
        LL_DMA_ClearFlag_TC4(DMA1);
        tx_write_ptr  = TX_BUFFER_HALF_SIZE;
        tx_free_space = TX_BUFFER_HALF_SIZE;
        for(uint16_t i = TX_BUFFER_HALF_SIZE; i < TX_BUFFER_SIZE; i++)
        {
            tx_buffer[i] = 0;
        }
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
