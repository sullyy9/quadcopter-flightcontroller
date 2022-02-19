/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    i2c.c
 * @brief   Module for controlling the I2C peripheral.
 *
 * @date    2021-04-04
 * -------------------------------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_i2c.h"
#include "stm32f3xx_ll_dma.h"

#include "i2c.hpp"

#include "debug.hpp"

using namespace i2c;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile uint8_t  tx_buffer[TX_BUFFER_SIZE];
static volatile uint32_t tx_write_ptr = 0;

static volatile uint8_t  rx_buffer[RX_BUFFER_SIZE];
static volatile uint32_t rx_read_ptr = 0;

static volatile bool transfer_ongoing = false;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the I2C peripherals and respective DMA channels.
 */
void i2c::initialise(void)
{
    /*
     * Setup I2C1 for communication with the accelerometer
     */
    LL_I2C_InitTypeDef i2c_init;
    i2c_init.AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE;
    i2c_init.DigitalFilter   = 0;
    i2c_init.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
    i2c_init.OwnAddress1     = 0;
    i2c_init.PeripheralMode  = LL_I2C_MODE_I2C;
    i2c_init.Timing          = __LL_I2C_CONVERT_TIMINGS(0x5, 0x1, 0x0, 0x1, 0x3);
    i2c_init.TypeAcknowledge = LL_I2C_ACK;
    LL_I2C_Init(I2C1, &i2c_init);
    LL_I2C_EnableDMAReq_RX(I2C1);
    LL_I2C_EnableDMAReq_TX(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);

    NVIC_SetPriority(I2C1_EV_IRQn, 3);
    NVIC_SetPriority(I2C1_ER_IRQn, 3);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    /*
     * Setup DMA1 channel 7 to transfer data from the I2C1 receive register to the receive buffer
     */
    LL_DMA_InitTypeDef dma_init;
    dma_init.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_init.MemoryOrM2MDstAddress  = (uint32_t)rx_buffer;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma_init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma_init.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma_init.NbData                 = RX_BUFFER_SIZE;
    dma_init.PeriphOrM2MSrcAddress  = LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_RECEIVE);
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma_init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_7, &dma_init);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);

    NVIC_SetPriority(DMA1_Channel7_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    /*
     * Setup DMA1 channel 6 to transfer data from the transmit buffer to the I2C1 transmit register
     */
    dma_init.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_init.MemoryOrM2MDstAddress  = (uint32_t)tx_buffer;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma_init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma_init.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma_init.NbData                 = TX_BUFFER_SIZE;
    dma_init.PeriphOrM2MSrcAddress  = LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_TRANSMIT);
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma_init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_6, &dma_init);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);

    NVIC_SetPriority(DMA1_Channel6_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return whether a transfer is in progress.
 * @return bool Transfer in progress.
 */
bool i2c::transfer_in_progress(void)
{
    return (transfer_ongoing);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Write a byte into the transmission buffer.
 * @param data Byte to be written.
 */
void i2c::tx_buffer_write(uint8_t data)
{
    tx_buffer[tx_write_ptr] = data;
    tx_write_ptr++;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief        Transmit the tx buffer to a slave device.
 * @param device Address of the slave device.
 * @param type   Type of transmission. Request transmissions must be left open so data can be
 *               received.
 */
void i2c::tx_data(address_t device, transmit_type_t type)
{
    uint32_t end_type     = 0;
    transfer_ongoing = true;
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, tx_write_ptr);

    switch(type)
    {
        case TRANSMIT_NORMAL:
        {
            end_type = LL_I2C_MODE_AUTOEND;
            break;
        }
        case TRANSMIT_REQUEST:
        {
            end_type = LL_I2C_MODE_SOFTEND;
            break;
        }
        default:
        {
            break;
        }
    }

    LL_I2C_HandleTransfer(I2C1,
                          device,
                          LL_I2C_ADDRSLAVE_7BIT,
                          tx_write_ptr,
                          end_type,
                          LL_I2C_GENERATE_START_WRITE);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief          Read a byte from the receive buffer.
 * @return uint8_t Read byte.
 */
uint8_t i2c::rx_buffer_read(void)
{
    return (rx_buffer[rx_read_ptr++]);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief              Receive data from the specified device into the rx buffer.
 * @param device       Device address.
 * @param number_bytes Number of bytes to receive.
 */
void i2c::rx_data(address_t device, uint32_t number_bytes)
{
    transfer_ongoing = true;
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, number_bytes);

    LL_I2C_HandleTransfer(I2C1,
                          device,
                          LL_I2C_ADDRSLAVE_7BIT,
                          number_bytes,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_READ);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for DMA1 channel 6. called when a data transmission is complete.
 */
void i2c::dma1_channel6_isr(void)
{
    LL_DMA_ClearFlag_TC6(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

    tx_write_ptr     = 0;
    transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for DMA1 channel 7. called when a data reception is complete.
 */
void i2c::dma1_channel7_isr(void)
{
    LL_DMA_ClearFlag_TC7(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);

    rx_read_ptr      = 0;
    transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for I2C1 event errors.
 */
void i2c::ev_isr(void)
{
    if(LL_I2C_IsActiveFlag_NACK(I2C1))
    {
        LL_I2C_ClearFlag_NACK(I2C1);
        debug::printf("ERR:I2C1_NACK\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interupt for I2C1 errors.
 */
void i2c::er_isr(void)
{
    if(LL_I2C_IsActiveFlag_BERR(I2C1))
    {
        LL_I2C_ClearFlag_BERR(I2C1);
        debug::printf("ERR:I2C1_BUS\r\n");
    }
    if(LL_I2C_IsActiveFlag_ARLO(I2C1))
    {
        LL_I2C_ClearFlag_ARLO(I2C1);
        debug::printf("ERR:I2C1_ARBRITRATION_LOSS\r\n");
    }
    if(LL_I2C_IsActiveFlag_OVR(I2C1))
    {
        LL_I2C_ClearFlag_OVR(I2C1);
        debug::printf("ERR:I2C1_OVER_UNDERRUN\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
