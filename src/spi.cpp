/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    spi.c
 * @brief   Module for controlling the SPI peripheral
 *
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_spi.h"
#include "stm32f3xx_ll_dma.h"

#include "spi.hpp"

#include "debug.hpp"

using namespace spi;
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
static volatile uint32_t rx_read_ptr      = 0;
static volatile bool     transfer_ongoing = false;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any used SPI peripherals and their respective DMA channels.
 */
void spi::initialise(void)
{
    /*
     * Setup SPI1 for communication with the gyroscope
     */
    LL_SPI_InitTypeDef spi;
    spi.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV16;
    spi.BitOrder          = LL_SPI_MSB_FIRST;
    spi.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
    spi.CRCPoly           = 0;
    spi.ClockPhase        = LL_SPI_PHASE_2EDGE;
    spi.ClockPolarity     = LL_SPI_POLARITY_HIGH;
    spi.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
    spi.Mode              = LL_SPI_MODE_MASTER;
    spi.NSS               = LL_SPI_NSS_SOFT;
    spi.TransferDirection = LL_SPI_FULL_DUPLEX;
    LL_SPI_Init(SPI1, &spi);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_EnableDMAReq_RX(SPI1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_SPI_EnableIT_ERR(SPI1);
    LL_SPI_Enable(SPI1);

    NVIC_SetPriority(SPI1_IRQn, 3);
    NVIC_EnableIRQ(SPI1_IRQn);

    /*
     * Setup a DMA channel to transfer data from the SPI1 receive register to the
     * receive buffer
     */
    LL_DMA_InitTypeDef dma;
    dma.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma.MemoryOrM2MDstAddress  = (uint32_t)rx_buffer;
    dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma.NbData                 = RX_BUFFER_SIZE;
    dma.PeriphOrM2MSrcAddress  = LL_SPI_DMA_GetRegAddr(SPI1);
    dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &dma);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

    NVIC_SetPriority(DMA1_Channel2_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /*
     * Setup a DMA channel to transfer data from the transmit buffer to the
     * SPI1 transmit register
     */
    dma.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma.MemoryOrM2MDstAddress  = (uint32_t)tx_buffer;
    dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma.Mode                   = LL_DMA_MODE_CIRCULAR;
    dma.NbData                 = TX_BUFFER_SIZE;
    dma.PeriphOrM2MSrcAddress  = LL_SPI_DMA_GetRegAddr(SPI1);
    dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_3, &dma);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);

    NVIC_SetPriority(DMA1_Channel3_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return whether a transfer is in progress.
 * @return bool True or false.
 */
bool spi::transfer_in_progress(void)
{
    return (transfer_ongoing);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief          Read a byte from the SPI1 RX buffer.
 * @return uint8_t Byte.
 */
uint8_t spi::rx_buffer_read(void)
{
    uint8_t byte;
    byte = rx_buffer[rx_read_ptr];
    rx_read_ptr++;

    return (byte);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Write a byte into the SPI1 TX buffer.
 * @param byte Byte.
 */
void spi::tx_buffer_write(uint8_t byte)
{
    tx_buffer[tx_write_ptr] = byte;
    tx_write_ptr++;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief          Begin a data transfer.
 * @param rx_bytes Number of bytes to receive.
 */
void spi::transfer_data(uint32_t rx_bytes)
{
    transfer_ongoing = true;

    for(uint32_t i = 0; i < rx_bytes; i++)
    {
        tx_buffer[tx_write_ptr] = 0;
        tx_write_ptr++;
    }

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, tx_write_ptr);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, tx_write_ptr);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for DMA1 channel 2. Called when all data has been transferred to the RX buffer.
 */
void spi::dma1_channel2_isr(void)
{
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    rx_read_ptr      = 0;
    tx_write_ptr     = 0;
    transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for DMA1 channel 3. Called when all data has been transfer from the TX buffer
 *        to the TX register.
 */
void spi::dma1_channel3_isr(void)
{
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt for SPI1 errors.
 */
void spi::error_isr(void)
{
    if(LL_SPI_IsActiveFlag_CRCERR(SPI1) == true)
    {
        LL_SPI_ClearFlag_CRCERR(SPI1);
        debug::printf("ERR:SPI1_CRC\r\n");
    }
    else if(LL_SPI_IsActiveFlag_MODF(SPI1) == true)
    {
        LL_SPI_ClearFlag_MODF(SPI1);
        debug::printf("ERR:SPI1_MODE\r\n");
    }
    else if(LL_SPI_IsActiveFlag_OVR(SPI1) == true)
    {
        LL_SPI_ClearFlag_OVR(SPI1);
        debug::printf("ERR:SPI1_OVERRUN\r\n");
    }
    else if(LL_SPI_IsActiveFlag_FRE(SPI1) == true)
    {
        LL_SPI_ClearFlag_FRE(SPI1);
        debug::printf("ERR:SPI1_FORMAT\r\n");
    }
    else
    {
        debug::printf("ERR:SPI1_UNKNOWN\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
