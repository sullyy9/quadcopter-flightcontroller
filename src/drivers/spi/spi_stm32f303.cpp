////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    i2c_stm32f303.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>

#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_spi.h"

#include "spi.hpp"
#include "spi_stm32f303.hpp"

using SPI = spi::stm32f303::SPI;

/*------------------------------------------------------------------------------------------------*/
// private objects
/*------------------------------------------------------------------------------------------------*/

namespace {

constinit volatile auto dma_tx_transfer_ongoing = false;
constinit volatile auto dma_rx_transfer_ongoing = false;

constexpr volatile auto DUMMY_TX_BYTE = std::byte{0};
constinit volatile auto dummy_rx_byte = std::byte{0};

constexpr auto DMA_RX_CHANNEL{LL_DMA_CHANNEL_2};
constexpr auto DMA_TX_CHANNEL{LL_DMA_CHANNEL_3};

}

/*------------------------------------------------------------------------------------------------*/
// private functions
/*------------------------------------------------------------------------------------------------*/

namespace {

auto dma_tx_start_transfer(const std::byte* source, size_t length) -> void;
auto dma_tx_start_dummy_transfer(size_t length) -> void;

auto dma_rx_start_transfer(const std::byte* destination, size_t length) -> void;
auto dma_rx_start_dummy_transfer(size_t length) -> void;

}

/*------------------------------------------------------------------------------------------------*/
// driver method definitions
/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise the I2C peripherals and respective DMA channels.
auto spi::stm32f303::SPI::init() noexcept -> Result<void> {
    // Setup I2C1
    LL_SPI_InitTypeDef spi{
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode = LL_SPI_MODE_MASTER,
        .DataWidth = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity = LL_SPI_POLARITY_HIGH,
        .ClockPhase = LL_SPI_PHASE_2EDGE,
        .NSS = LL_SPI_NSS_SOFT,
        .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16,
        .BitOrder = LL_SPI_MSB_FIRST,
        .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
        .CRCPoly = 0,
    };
    LL_SPI_Init(SPI1, &spi);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_EnableDMAReq_RX(SPI1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_SPI_EnableIT_ERR(SPI1);
    LL_SPI_Enable(SPI1);

    NVIC_SetPriority(SPI1_IRQn, 3);
    NVIC_EnableIRQ(SPI1_IRQn);

    // Setup DMA1 channel 2 to transfer data from the SPI1 receive register to a buffer.
    LL_DMA_InitTypeDef rx_dma_init{
        .PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(SPI1),
        .MemoryOrM2MDstAddress = 0,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = 0,
        .Priority = LL_DMA_PRIORITY_MEDIUM,
    };
    LL_DMA_Init(DMA1, DMA_RX_CHANNEL, &rx_dma_init);
    LL_DMA_EnableIT_TC(DMA1, DMA_RX_CHANNEL);

    NVIC_SetPriority(DMA1_Channel2_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    // Setup DMA1 channel 3 to transfer data from a buffer to the SPI1 transmit register.
    LL_DMA_InitTypeDef tx_dma_init{
        .PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(SPI1),
        .MemoryOrM2MDstAddress = 0,
        .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = 0,
        .Priority = LL_DMA_PRIORITY_MEDIUM,
    };
    LL_DMA_Init(DMA1, DMA_TX_CHANNEL, &tx_dma_init);
    LL_DMA_EnableIT_TC(DMA1, DMA_TX_CHANNEL);

    NVIC_SetPriority(DMA1_Channel3_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    return {};
}

/*------------------------------------------------------------------------------------------------*/

auto SPI::transfer(const std::span<const std::byte> write, const std::span<std::byte> read) noexcept
    -> Result<void> {
    dma_rx_start_transfer(read.data(), read.size());
    dma_tx_start_transfer(write.data(), write.size());

    // The RX DMA transfer will be the last to finish, so wait for that.
    while (dma_rx_transfer_ongoing) continue;
    return {};
}

/*------------------------------------------------------------------------------------------------*/

auto SPI::read(const std::span<std::byte> buf) noexcept -> Result<void> {
    dma_rx_start_transfer(buf.data(), buf.size());
    dma_tx_start_dummy_transfer(buf.size());

    // The RX DMA transfer will be the last to finish, so wait for that.
    while (dma_rx_transfer_ongoing) continue;
    return {};
}

/*------------------------------------------------------------------------------------------------*/

auto SPI::write(const std::span<const std::byte> buf) noexcept -> Result<void> {
    dma_rx_start_dummy_transfer(buf.size());
    dma_tx_start_transfer(buf.data(), buf.size());

    // The RX DMA transfer will be the last to finish, so wait for that.
    while (dma_rx_transfer_ongoing) continue;
    return {};
}

/*------------------------------------------------------------------------------------------------*/
// interupts
/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for DMA1 channel 2. Called when a read operation is complete.
auto spi::stm32f303::dma1_channel2_isr() noexcept -> void {
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableChannel(DMA1, DMA_RX_CHANNEL);
    dma_rx_transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for DMA1 channel 3. Called when a write operation is complete.
auto spi::stm32f303::dma1_channel3_isr() noexcept -> void {
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_DisableChannel(DMA1, DMA_TX_CHANNEL);
    dma_tx_transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Interupt for SPI1 errors.
auto spi::stm32f303::error_isr() noexcept -> void {
    if (static_cast<bool>(LL_SPI_IsActiveFlag_CRCERR(SPI1))) {
        LL_SPI_ClearFlag_CRCERR(SPI1);
        while(true) continue;
        // SerialDebug::print("ERR:SPI1_CRC\r\n");
    }

    if (static_cast<bool>(LL_SPI_IsActiveFlag_MODF(SPI1))) {
        LL_SPI_ClearFlag_MODF(SPI1);
        while(true) continue;
        // SerialDebug::print("ERR:SPI1_MODE\r\n");
    }

    if (static_cast<bool>(LL_SPI_IsActiveFlag_OVR(SPI1))) {
        LL_SPI_ClearFlag_OVR(SPI1);
        while(true) continue;
        // SerialDebug::print("ERR:SPI1_OVERRUN\r\n");
    }

    if (static_cast<bool>(LL_SPI_IsActiveFlag_FRE(SPI1))) {
        LL_SPI_ClearFlag_FRE(SPI1);
        while(true) continue;
        // SerialDebug::print("ERR:SPI1_FORMAT\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/
// private functions
/*------------------------------------------------------------------------------------------------*/

namespace {

/// @brief Start a transfer from memory to the SPI1 TX buffer.
auto dma_tx_start_transfer(const std::byte* const source, const size_t length) -> void {
    dma_tx_transfer_ongoing = true;

    LL_DMA_SetMemoryAddress(DMA1, DMA_TX_CHANNEL, (uint32_t)source);
    LL_DMA_SetDataLength(DMA1, DMA_TX_CHANNEL, length);
    LL_DMA_SetMemoryIncMode(DMA1, DMA_TX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_EnableChannel(DMA1, DMA_TX_CHANNEL);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Start a dummy transfer to the SPI1 TX buffer.
auto dma_tx_start_dummy_transfer(const size_t length) -> void {
    dma_tx_transfer_ongoing = true;

    LL_DMA_SetMemoryAddress(DMA1, DMA_TX_CHANNEL, (uint32_t)&DUMMY_TX_BYTE);
    LL_DMA_SetDataLength(DMA1, DMA_TX_CHANNEL, length);
    LL_DMA_SetMemoryIncMode(DMA1, DMA_TX_CHANNEL, LL_DMA_MEMORY_NOINCREMENT);
    LL_DMA_EnableChannel(DMA1, DMA_TX_CHANNEL);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Start a transfer from the SPI1 RX buffer to memory.
auto dma_rx_start_transfer(const std::byte* const destination, const size_t length) -> void {
    dma_rx_transfer_ongoing = true;

    LL_DMA_SetMemoryAddress(DMA1, DMA_RX_CHANNEL, (uint32_t)destination);
    LL_DMA_SetDataLength(DMA1, DMA_RX_CHANNEL, length);
    LL_DMA_SetMemoryIncMode(DMA1, DMA_RX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_EnableChannel(DMA1, DMA_RX_CHANNEL);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Start a dummy transfer from the SPI1 RX buffer to memory.
auto dma_rx_start_dummy_transfer(const size_t length) -> void {
    dma_rx_transfer_ongoing = true;

    LL_DMA_SetMemoryAddress(DMA1, DMA_RX_CHANNEL, (uint32_t)&dummy_rx_byte);
    LL_DMA_SetDataLength(DMA1, DMA_RX_CHANNEL, length);
    LL_DMA_SetMemoryIncMode(DMA1, DMA_RX_CHANNEL, LL_DMA_MEMORY_NOINCREMENT);
    LL_DMA_EnableChannel(DMA1, DMA_RX_CHANNEL);
}

}

/*------------------------------------------------------------------------------------------------*/
