/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : i2c.c
    Description     : i2c functions
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_i2c.h"
#include "stm32f3xx_ll_dma.h"

#include "debug.h"
#include "utils.h"
#include "i2c.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define I2C1_TX_BUFFER_SIZE 256
#define I2C1_RX_BUFFER_SIZE 256

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint8_t     i2c1_tx_buffer[ I2C1_TX_BUFFER_SIZE ];
static volatile uint32_t    i2c1_tx_write_ptr       = 0;
static volatile bool        i2c1_tx_in_progress     = false;

static volatile uint8_t     i2c1_rx_buffer[ I2C1_RX_BUFFER_SIZE ];
static volatile uint32_t    i2c1_rx_buffer_count    = 0;
static volatile uint32_t    i2c1_rx_read_ptr        = 0;
static volatile bool        i2c1_rx_in_progress     = false;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * @brief       setup i2c peripherals
 * @param       none
 * @retval      none
 */
void i2c_initialise( void )
{
    /*
     * Setup I2C1 for communication with the accelerometer
     */
    LL_I2C_InitTypeDef i2c_initialisation_structure;
    i2c_initialisation_structure.AnalogFilter       = LL_I2C_ANALOGFILTER_ENABLE;
    i2c_initialisation_structure.DigitalFilter      = 0;
    i2c_initialisation_structure.OwnAddrSize        = LL_I2C_OWNADDRESS1_7BIT;
    i2c_initialisation_structure.OwnAddress1        = 0;
    i2c_initialisation_structure.PeripheralMode     = LL_I2C_MODE_I2C;
    i2c_initialisation_structure.Timing             = __LL_I2C_CONVERT_TIMINGS( 0x5, 0x1, 0x0, 0x1, 0x3 );//PRESC, SCLDEL, SDADEL, SCLH, SCLL
    i2c_initialisation_structure.TypeAcknowledge    = LL_I2C_ACK;
    LL_I2C_Init( I2C1, &i2c_initialisation_structure );
    LL_I2C_EnableDMAReq_RX( I2C1 );
    LL_I2C_EnableDMAReq_TX( I2C1 );
    LL_I2C_EnableIT_ERR( I2C1 );

    NVIC_SetPriority( I2C1_EV_IRQn, 3 );
    NVIC_EnableIRQ( I2C1_EV_IRQn );

    /*
     * Setup DMA to transfer data from the I2C1 receive register to the
     * receive buffer
     */
    LL_DMA_InitTypeDef dma_initialisation_structure;
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = (uint32_t)i2c1_rx_buffer;
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_INCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = I2C1_RX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = LL_I2C_DMA_GetRegAddr( I2C1, LL_I2C_DMA_REG_DATA_RECEIVE );
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_NOINCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_7, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_7 );

    NVIC_SetPriority( DMA1_Channel7_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel7_IRQn );

    /*
     * Setup DMA to transfer data from the transmit buffer to the
     * I2C1 transmit register
     */
    dma_initialisation_structure.Direction                 = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_initialisation_structure.MemoryOrM2MDstAddress     = (uint32_t)i2c1_tx_buffer;
    dma_initialisation_structure.MemoryOrM2MDstDataSize    = LL_DMA_MDATAALIGN_BYTE;
    dma_initialisation_structure.MemoryOrM2MDstIncMode     = LL_DMA_MEMORY_INCREMENT;
    dma_initialisation_structure.Mode                      = LL_DMA_MODE_CIRCULAR;
    dma_initialisation_structure.NbData                    = I2C1_TX_BUFFER_SIZE;
    dma_initialisation_structure.PeriphOrM2MSrcAddress     = LL_I2C_DMA_GetRegAddr( I2C1, LL_I2C_DMA_REG_DATA_TRANSMIT );
    dma_initialisation_structure.PeriphOrM2MSrcDataSize    = LL_DMA_PDATAALIGN_BYTE;
    dma_initialisation_structure.PeriphOrM2MSrcIncMode     = LL_DMA_PERIPH_NOINCREMENT;
    dma_initialisation_structure.Priority                  = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init( DMA1, LL_DMA_CHANNEL_6, &dma_initialisation_structure );
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_6 );

    NVIC_SetPriority( DMA1_Channel6_IRQn, 3 );
    NVIC_EnableIRQ( DMA1_Channel6_IRQn );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief       return number of free bytes in the I2C1 buffer.
 *              return 0 if a transmission is in progress
 * @param       data byte to be written
 * @retval      none
 */
uint16_t i2c1_buffer_free( void )
{
    if( i2c1_tx_in_progress == true )
    {
        return( 0 );
    }
    else
    {
        return( I2C1_TX_BUFFER_SIZE - i2c1_tx_write_ptr );
    }
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       write a byte of data into the I2C1 tx buffer
 * @param       data byte to be written
 * @retval      none
 */
void i2c1_tx_buffer_write( uint8_t data )
{
    i2c1_tx_buffer[ i2c1_tx_write_ptr ] = data;
    i2c1_tx_write_ptr++;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       clear the I2C1 tx buffer
 * @param       none
 * @retval      none
 */
void i2c1_tx_buffer_clear( void )
{
    for( uint16_t i = 0; i < I2C1_TX_BUFFER_SIZE; i++ )
    {
        i2c1_tx_buffer[ i ] = 0;
    }
    i2c1_tx_write_ptr = 0;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       transmit data in the tx buffer to a device
 * @param       none
 * @retval      none
 */
void i2c1_tx_data( uint32_t device, bool auto_end )
{
    uint32_t end_type = 0;
    i2c1_tx_in_progress = true;
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_6, i2c1_tx_write_ptr );

    if( auto_end == true )
    {
        end_type = LL_I2C_MODE_AUTOEND;
    }
    else
    {
        end_type = LL_I2C_MODE_SOFTEND;
    }

    LL_I2C_HandleTransfer
    (
        I2C1,
        device,
        LL_I2C_ADDRSLAVE_7BIT,
        i2c1_tx_write_ptr,
        end_type,
        LL_I2C_GENERATE_START_WRITE
    );

    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_6 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       get the amount of data in the I2C1 rx buffer
 * @param       none
 * @retval      amount of data available
 */
uint32_t i2c1_rx_buffer_available( void )
{
    if( i2c1_rx_in_progress == true )
    {
        return ( 0 );
    }
    else
    {
        return( i2c1_rx_buffer_count );
    }
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       read a byte of data from the rx buffer
 * @param       none
 * @retval      data byte
 */
uint8_t i2c1_rx_buffer_read( void )
{
    return( i2c1_rx_buffer[ i2c1_rx_read_ptr++ ] );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       clear the I2C1 rx buffer
 * @param       none
 * @retval      none
 */
void i2c1_rx_buffer_clear( void )
{
    for( uint16_t i = 0; i < I2C1_RX_BUFFER_SIZE; i++ )
    {
        i2c1_rx_buffer[ i ] = 0;
    }
    i2c1_rx_read_ptr = 0;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       receive data into the rx buffer. The last byte must be
 * @param       number of bytes to receive
 * @retval      none
 */
void i2c1_rx_data( uint32_t device, uint32_t number_bytes )
{
    i2c1_rx_in_progress = true;
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_7, number_bytes );
    i2c1_rx_buffer_count = number_bytes;

    while( i2c1_tx_in_progress == true );
    LL_I2C_HandleTransfer
    (
        I2C1,
        device,
        LL_I2C_ADDRSLAVE_7BIT,
        number_bytes,
        LL_I2C_MODE_SOFTEND,
        LL_I2C_GENERATE_START_READ
    );

    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_7 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 6. called when a data transfer is
 *              complete. clear transmit buffer
 * @param       none
 * @retval      none
 */
void i2c_dma1_channel6_isr( void )
{

    LL_DMA_ClearFlag_TC6( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_6 );

    for( uint16_t i = 0; i < I2C1_TX_BUFFER_SIZE; i++ )
    {
        i2c1_tx_buffer[ i2c1_tx_write_ptr ] = 0;
    }
    i2c1_tx_write_ptr = 0;
    i2c1_tx_in_progress = false;
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       interrupt for dma1 channel 7. called when half the buffer
 *              ( top or bottom half ) has been transfered. switch the
 *              write pointer to the half not being currently transfered.
 *              turn off the channel if there's no more data in the buffer
 * @param       none
 * @retval      none
 */
void i2c_dma1_channel7_isr( void )
{
    LL_DMA_ClearFlag_TC7( DMA1 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_7 );
    i2c1_rx_in_progress = false;
}

/*----------------------------------------------------------------------------*/

void i2c1_er_isr( void )
{
    if( LL_I2C_IsActiveFlag_BERR( I2C1 ) )
    {
        LL_I2C_ClearFlag_BERR( I2C1 );
        debug_printf( "i2c bus error interupt\r\n" );
    }
    if( LL_I2C_IsActiveFlag_ARLO( I2C1 ) )
    {
        LL_I2C_ClearFlag_ARLO( I2C1 );
        debug_printf( "i2c arbitration loss interupt\r\n" );
    }
    if( LL_I2C_IsActiveFlag_OVR( I2C1 ) )
    {
        LL_I2C_ClearFlag_OVR( I2C1 );
        debug_printf( "i2c over/underrun interupt\r\n" );
    }
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
