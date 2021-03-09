/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : spi.c
    Description     : spi functions
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_spi.h"
#include "stm32f3xx_ll_dma.h"

#include "spi.h"
#include "port.h"

#include "debug.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define SPI1_TX_BUFFER_SIZE 128
#define SPI1_RX_BUFFER_SIZE 128

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint8_t     spi1_tx_buffer[ SPI1_TX_BUFFER_SIZE ];
static volatile uint32_t    spi1_tx_write_ptr       = 0;

static volatile uint8_t     spi1_rx_buffer[ SPI1_RX_BUFFER_SIZE ];
static volatile uint32_t    spi1_rx_read_ptr            = 0;
static volatile bool        spi1_transfer_ongoing       = false;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void spi_initialise( void )
{
    /*
     * Setup SPI1 for communication with the gyroscope
     */
    LL_SPI_InitTypeDef spi_initialisation_structure;
    spi_initialisation_structure.BaudRate           = LL_SPI_BAUDRATEPRESCALER_DIV8;
    spi_initialisation_structure.BitOrder           = LL_SPI_MSB_FIRST;
    spi_initialisation_structure.CRCCalculation     = LL_SPI_CRCCALCULATION_DISABLE;
    spi_initialisation_structure.CRCPoly            = 0;
    spi_initialisation_structure.ClockPhase         = LL_SPI_PHASE_2EDGE;
    spi_initialisation_structure.ClockPolarity      = LL_SPI_POLARITY_HIGH;
    spi_initialisation_structure.DataWidth          = LL_SPI_DATAWIDTH_8BIT;
    spi_initialisation_structure.Mode               = LL_SPI_MODE_MASTER;
    spi_initialisation_structure.NSS                = LL_SPI_NSS_SOFT;
    spi_initialisation_structure.TransferDirection  = LL_SPI_FULL_DUPLEX;
    LL_SPI_Init( SPI1, &spi_initialisation_structure );
    LL_SPI_SetRxFIFOThreshold( SPI1, LL_SPI_RX_FIFO_TH_QUARTER );
    LL_SPI_EnableDMAReq_RX( SPI1 );
    LL_SPI_EnableDMAReq_TX( SPI1 );
    LL_SPI_EnableIT_ERR( SPI1 );
    LL_SPI_Enable( SPI1 );

    NVIC_SetPriority( SPI1_IRQn, 3 );
    NVIC_EnableIRQ( SPI1_IRQn );

    /*
     * Setup DMA to transfer data from the SPI1 receive register to the
     * receive buffer
     */
    LL_DMA_InitTypeDef dma_initialisation_structure;
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = (uint32_t)spi1_rx_buffer;
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_INCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = SPI1_RX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = LL_SPI_DMA_GetRegAddr( SPI1 );
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_NOINCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_2, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_2 );

    NVIC_SetPriority( DMA1_Channel2_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel2_IRQn );

    /*
     * Setup DMA to transfer data from the transmit buffer to the
     * SPI1 transmit register
     */
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = (uint32_t)spi1_tx_buffer;
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_INCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = SPI1_TX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = LL_SPI_DMA_GetRegAddr( SPI1 );
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_NOINCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_3, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_3 );

    NVIC_SetPriority( DMA1_Channel3_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel3_IRQn );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       return whether a transfer is in progress
 * @param       data byte to be written
 * @retval      none
 */
bool spi1_transfer_in_progress( void )
{
    return( spi1_transfer_ongoing );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       read a byte of data from the RX buffer
 * @param       none
 * @retval      data byte
 */
uint8_t spi1_rx_buffer_read( void )
{
    return( spi1_rx_buffer[ spi1_rx_read_ptr++ ] );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       write a byte of data into the SPI1 TX buffer
 * @param       data byte to be written
 * @retval      none
 */
void spi1_tx_buffer_write( uint8_t data )
{
    spi1_tx_buffer[ spi1_tx_write_ptr++ ] = data;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       transfer data
 * @param       number of bytes to receive
 * @retval      none
 */
void spi1_transfer_data( uint32_t rx_bytes )
{
    spi1_transfer_ongoing = true;

    for( uint32_t i = 0; i < rx_bytes; i++ )
    {
        spi1_tx_buffer[ spi1_tx_write_ptr ] = 0;
        spi1_tx_write_ptr++;
    }

    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_2, spi1_tx_write_ptr );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_3, spi1_tx_write_ptr );
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_2 );
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_3 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 2. called when data reception, and thus
 *              a full transfer, is complete.
 * @param       none
 * @retval      none
 */
void spi1_dma1_channel2_isr( void )
{
    LL_DMA_ClearFlag_TC2( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_2 );

    spi1_rx_read_ptr  = 0;
    spi1_tx_write_ptr = 0;
    spi1_transfer_ongoing = false;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 3. called when data transmission is
 *              complete.
 * @param       none
 * @retval      none
 */
void spi1_dma1_channel3_isr( void )
{
    LL_DMA_ClearFlag_TC3( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_3 );
}

/*----------------------------------------------------------------------------*/

void spi1_error_isr( void )
{
    if( LL_SPI_IsActiveFlag_CRCERR( SPI1 ) == true )
    {
        LL_SPI_ClearFlag_CRCERR( SPI1 );
        debug_printf( "ERR:SPI1_CRC\r\n" );
    }
    else if( LL_SPI_IsActiveFlag_MODF( SPI1 ) == true )
    {
        LL_SPI_ClearFlag_MODF( SPI1 );
        debug_printf( "ERR:SPI1_MODE\r\n" );
    }
    else if( LL_SPI_IsActiveFlag_OVR( SPI1 ) == true )
    {
        LL_SPI_ClearFlag_OVR( SPI1 );
        debug_printf( "ERR:SPI1_OVERRUN\r\n" );
    }
    else if( LL_SPI_IsActiveFlag_FRE( SPI1 ) == true )
    {
        LL_SPI_ClearFlag_FRE( SPI1 );
        debug_printf( "ERR:SPI1_FORMAT\r\n" );
    }
    else
    {
        debug_printf( "ERR:SPI1_UNKNOWN\r\n" );
    }
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
