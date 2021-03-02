/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : usart.c
    Description     : USART and UART functions
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"

#include "usart.h"


/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define USART1_TX_BUFFER_SIZE 256
#define USART1_TX_BUFFER_HALF_SIZE ( USART1_TX_BUFFER_SIZE / 2 )

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint8_t     usart1_tx_buffer[ USART1_TX_BUFFER_SIZE ];
static volatile uint16_t    usart1_tx_write_ptr = USART1_TX_BUFFER_HALF_SIZE;
static          uint16_t    usart1_tx_free_space = USART1_TX_BUFFER_HALF_SIZE;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * @brief       setup usart peripherals
 * @param       none
 * @retval      none
 */
void usart_initialise( void )
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
     * USART1 transmit register
     */
    LL_DMA_InitTypeDef dma_initialisation_structure;
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = (uint32_t)usart1_tx_buffer;
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_INCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = USART1_TX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = LL_USART_DMA_GetRegAddr( USART1, LL_USART_DMA_REG_DATA_TRANSMIT );
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_NOINCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_LOW;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_4, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_4 );
    LL_DMA_EnableIT_HT( DMA1, LL_DMA_CHANNEL_4 );
    LL_USART_ClearFlag_TC( USART1 );

    NVIC_SetPriority( DMA1_Channel4_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel4_IRQn );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief       return the amount of free space in the current writable half
 *              of the USART1 transmission buffer
 * @param       none
 * @retval      number of free bytes
 */
uint16_t usart1_tx_free( void )
{
    return( usart1_tx_free_space );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       place a byte into the usart1 transmit buffer. turn on the
 *              DMA channel if its off
 * @param       data byte
 * @retval      none
 */
void usart1_tx_byte( uint8_t data )
{
    usart1_tx_buffer[ usart1_tx_write_ptr ] = data;
    usart1_tx_write_ptr++;
    usart1_tx_free_space--;

    if( LL_DMA_IsEnabledChannel( DMA1, LL_DMA_CHANNEL_4 ) == 0 )
    {
        LL_USART_ClearFlag_TC( USART1 );
        LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_4 );
    }
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 4. called when half the buffer
 *              ( top or bottom half ) has been transfered. switch the
 *              write pointer to the half not being currently transfered.
 *              turn off the channel if there's no more data in the buffer
 * @param       none
 * @retval      none
 */
void usart_dma1_channel4_isr( void )
{
    /*
     * no more data in the buffer so turn off the channel.
     * all registers will be set to there initially configured
     * values
     */
    if( ( usart1_tx_write_ptr == 0 ) || ( usart1_tx_write_ptr == USART1_TX_BUFFER_HALF_SIZE ) )
    {
        LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_4 );
        usart1_tx_write_ptr = USART1_TX_BUFFER_HALF_SIZE;
    }

    if( LL_DMA_IsActiveFlag_HT4( DMA1 ) )
    {
        LL_DMA_ClearFlag_HT4( DMA1 );
        usart1_tx_write_ptr     = 0;
        usart1_tx_free_space    = USART1_TX_BUFFER_HALF_SIZE;
        for( uint16_t i = 0; i < USART1_TX_BUFFER_HALF_SIZE; i++ )
        {
            usart1_tx_buffer[ i ] = 0;
        }
    }
    else
    {
        LL_DMA_ClearFlag_TC4( DMA1 );
        usart1_tx_write_ptr     = USART1_TX_BUFFER_HALF_SIZE;
        usart1_tx_free_space    = USART1_TX_BUFFER_HALF_SIZE;
        for( uint16_t i = USART1_TX_BUFFER_HALF_SIZE; i < USART1_TX_BUFFER_SIZE; i++ )
        {
            usart1_tx_buffer[ i ] = 0;
        }
    }
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
