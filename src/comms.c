/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : comms.c
    Description     : communication functions
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"

#include "comms.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define USART1_TX_BUFFER_SIZE 256
#define USART_TX_BUFFER_HALF_SIZE ( USART1_TX_BUFFER_SIZE / 2 )

#define TX_REG 0x40013828

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint8_t     usart1_tx_buffer[ USART1_TX_BUFFER_SIZE ];
static volatile uint16_t    usart1_tx_write_ptr = USART_TX_BUFFER_HALF_SIZE;
volatile uint16_t           transmissions = 0;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * @brief       setup communication peripherals
 * @param       none
 * @retval      none
 */
void comms_initialise( void )
{
    /*
     * Setup USART1 for transmitting debug messages
     */
    LL_USART_InitTypeDef usart_initialisation_structure;
    usart_initialisation_structure.BaudRate              = 115200;
    usart_initialisation_structure.DataWidth             = LL_USART_DATAWIDTH_8B;
    usart_initialisation_structure.HardwareFlowControl   = LL_USART_HWCONTROL_NONE;
    usart_initialisation_structure.OverSampling          = LL_USART_OVERSAMPLING_8;
    usart_initialisation_structure.Parity                = LL_USART_PARITY_NONE;
    usart_initialisation_structure.StopBits              = LL_USART_STOPBITS_1;
    usart_initialisation_structure.TransferDirection     = LL_USART_DIRECTION_TX;
    LL_USART_Init( USART1, &usart_initialisation_structure );
    LL_USART_EnableDMAReq_TX( USART1 );
    LL_USART_Enable( USART1 );

    NVIC_SetPriority( USART1_IRQn, 3 );
    NVIC_EnableIRQ( USART1_IRQn );

    /*
     * Setup DMA to transfer data from the transmit buffer to the
     * USART transmit register
     */
    LL_DMA_InitTypeDef dma_initialisation_structure;
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = LL_USART_DMA_GetRegAddr( USART1, LL_USART_DMA_REG_DATA_TRANSMIT );
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_NOINCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = USART1_TX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = (uint32_t)usart1_tx_buffer;
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_INCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_4, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_4 );
    LL_DMA_EnableIT_HT( DMA1, LL_DMA_CHANNEL_4 );
    LL_USART_ClearFlag_TC( USART1 );

    NVIC_SetPriority( DMA1_Channel4_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel4_IRQn );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       return the amount of free space in the current half of
 *              the USART1 transmission buffer
 * @param       none
 * @retval      number of free bytes
 */
uint16_t comms_usart1_tx_free( void )
{
    uint16_t free_space = 0;
    if( usart1_tx_write_ptr < USART_TX_BUFFER_HALF_SIZE )
    {
        free_space = USART_TX_BUFFER_HALF_SIZE - usart1_tx_write_ptr;
    }
    else
    {
        free_space = USART1_TX_BUFFER_SIZE - usart1_tx_write_ptr;
    }

    return( free_space );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       place a byte into the end of the usart1 transmit buffer
 *              turn on the DMA channel if its off
 * @param       data byte
 * @retval      none
 */
void comms_usart1_tx_byte( uint8_t data )
{
    usart1_tx_buffer[ usart1_tx_write_ptr ] = data;
    usart1_tx_write_ptr++;
    if( usart1_tx_write_ptr >= USART1_TX_BUFFER_SIZE )
    {
        usart1_tx_write_ptr = 0;
    }

    if( LL_DMA_IsEnabledChannel( DMA1, LL_DMA_CHANNEL_4 ) == 0 )
    {
        LL_USART_ClearFlag_TC( USART1 );
        LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_4 );
    }
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 4. called when half the buffer
 *              has been transfered. switch the write pointer to the half
 *              not being currently transfered. turn off the channel if
 *              there's no data
 * @param       none
 * @retval      none
 */
void comms_dma1_channel4_isr( void )
{
    /*
     * no more data in the buffer so turn off the channel.
     * all registers will be set to there initially configured
     * values
     */
    if( ( usart1_tx_write_ptr == 0 ) || ( usart1_tx_write_ptr == USART_TX_BUFFER_HALF_SIZE ) )
    {
        LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_4 );
        usart1_tx_write_ptr = USART_TX_BUFFER_HALF_SIZE;
    }

    if( LL_DMA_IsActiveFlag_HT4( DMA1 ) )
    {
        LL_DMA_ClearFlag_HT4( DMA1 );
        usart1_tx_write_ptr = 0;
    }
    else
    {
        LL_DMA_ClearFlag_TC4( DMA1 );
        usart1_tx_write_ptr = USART_TX_BUFFER_HALF_SIZE;
    }
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for usart1
 * @param       none
 * @retval      none
 */
void comms_usart1_isr( void )
{
    transmissions++;
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
