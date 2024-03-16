////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    i2c_stm32f303.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <optional>
#include <variant>

#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_i2c.h"

#include "i2c.hpp"
#include "i2c_stm32f303.hpp"

using I2C = i2c::stm32f303::I2C;

/*------------------------------------------------------------------------------------------------*/
// private objects
/*------------------------------------------------------------------------------------------------*/

namespace {

constinit volatile auto dma_tx_transfer_ongoing = false;
constinit volatile auto dma_rx_transfer_ongoing = false;
constinit volatile auto dummy_byte = std::byte{0};

constexpr auto DMA_RX_CHANNEL{LL_DMA_CHANNEL_7};
constexpr auto DMA_TX_CHANNEL{LL_DMA_CHANNEL_6};

}

/*------------------------------------------------------------------------------------------------*/
// private functions
/*------------------------------------------------------------------------------------------------*/

namespace {

auto dma_tx_start_transfer(const std::byte* source, size_t length) -> void;
auto dma_rx_start_transfer(const std::byte* destination, size_t length) -> void;

}

/*------------------------------------------------------------------------------------------------*/
// driver method definitions
/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise the I2C peripherals and respective DMA channels.
auto i2c::stm32f303::I2C::init() noexcept -> Result<void> {
    // Setup I2C1
    LL_I2C_InitTypeDef i2c_init;
    i2c_init.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    i2c_init.DigitalFilter = 0;
    i2c_init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    i2c_init.OwnAddress1 = 0;
    i2c_init.PeripheralMode = LL_I2C_MODE_I2C;
    i2c_init.Timing = __LL_I2C_CONVERT_TIMINGS(0x5, 0x1, 0x0, 0x1, 0x3);
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

    // Setup DMA1 channel 7 to transfer data from the I2C1 receive register to the receive buffer.
    LL_DMA_InitTypeDef rx_dma_init{
        .PeriphOrM2MSrcAddress = LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_RECEIVE),
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

    NVIC_SetPriority(DMA1_Channel7_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    // Setup DMA1 channel 6 to transfer data from the transmit buffer to the I2C1 transmit register.
    LL_DMA_InitTypeDef tx_dma_init{
        .PeriphOrM2MSrcAddress = LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_TRANSMIT),
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

    NVIC_SetPriority(DMA1_Channel6_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    return {};
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::transaction(const Address address, const std::span<Operation> operations) noexcept
    -> Result<void> {
    enum class LastOp { Read, Write };
    auto last_operation = std::optional<LastOp>{};

    for (const auto& operation : operations) {
        // Setup the next transfer.
        if (std::holds_alternative<operation::Write>(operation)) {
            const auto& write = std::get<operation::Write>(operation);
            const auto start = (last_operation == LastOp::Write) ? LL_I2C_GENERATE_NOSTARTSTOP
                                                                 : LL_I2C_GENERATE_START_WRITE;
            last_operation = LastOp::Write;

            dma_tx_transfer_ongoing = true;
            dma_tx_start_transfer(write.source.data(), write.source.size());
            LL_I2C_HandleTransfer(I2C1,
                                  address,
                                  LL_I2C_ADDRSLAVE_7BIT,
                                  write.source.size(),
                                  LL_I2C_MODE_SOFTEND,
                                  start);

            while (dma_tx_transfer_ongoing || LL_I2C_IsActiveFlag_TXE(I2C1) == 0) continue;

        } else if (std::holds_alternative<operation::Read>(operation)) {
            const auto& read = std::get<operation::Read>(operation);
            const auto start = (last_operation == LastOp::Read) ? LL_I2C_GENERATE_NOSTARTSTOP
                                                                : LL_I2C_GENERATE_START_READ;
            last_operation = LastOp::Read;

            dma_rx_transfer_ongoing = true;
            dma_rx_start_transfer(read.destination.data(), read.destination.size());
            LL_I2C_HandleTransfer(I2C1,
                                  address,
                                  LL_I2C_ADDRSLAVE_7BIT,
                                  read.destination.size(),
                                  LL_I2C_MODE_SOFTEND,
                                  start);

            while (dma_rx_transfer_ongoing) continue;
            LL_DMA_DisableChannel(DMA1, DMA_TX_CHANNEL);
        }
    }

    // Generate the stop condition and wait for the transfer to fully complete so we can be certain
    // it's safe to start a new transfer immediately afterwords.
    LL_I2C_GenerateStopCondition(I2C1);
    while (LL_I2C_IsActiveFlag_STOP(I2C1) == 0) continue;

    return {};
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::read(const Address address, const std::span<std::byte> buffer) noexcept -> Result<void> {
    return i2c::default_impl::read<I2C>(address, buffer);
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::write(const Address address, const std::span<const std::byte> buffer) noexcept
    -> Result<void> {
    return i2c::default_impl::write<I2C>(address, buffer);
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::write_read(const Address address,
                     const std::span<const std::byte> write_buffer,
                     const std::span<std::byte> read_buffer) noexcept -> Result<void> {
    return i2c::default_impl::write_read<I2C>(address, write_buffer, read_buffer);
}

/*------------------------------------------------------------------------------------------------*/
// interupts
/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for DMA1 channel 6. Called when a write operation is complete.
auto i2c::stm32f303::dma1_channel6_isr() noexcept -> void {
    LL_DMA_ClearFlag_TC6(DMA1);
    LL_DMA_DisableChannel(DMA1, DMA_TX_CHANNEL);
    dma_tx_transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for DMA1 channel 7. Called when a read operation is complete.
auto i2c::stm32f303::dma1_channel7_isr() noexcept -> void {
    LL_DMA_ClearFlag_TC7(DMA1);
    LL_DMA_DisableChannel(DMA1, DMA_RX_CHANNEL);
    dma_rx_transfer_ongoing = false;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for I2C1 event errors.
auto i2c::stm32f303::ev_isr() noexcept -> void {
    if (LL_I2C_IsActiveFlag_NACK(I2C1) != 0) {
        LL_I2C_ClearFlag_NACK(I2C1);
        // SerialDebug::print("ERR:I2C1_NACK\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Interupt for I2C1 errors.
auto i2c::stm32f303::er_isr() noexcept -> void {
    if (LL_I2C_IsActiveFlag_BERR(I2C1) != 0) {
        LL_I2C_ClearFlag_BERR(I2C1);
        // SerialDebug::print("ERR:I2C1_BUS\r\n");
    }

    if (LL_I2C_IsActiveFlag_ARLO(I2C1) != 0) {
        LL_I2C_ClearFlag_ARLO(I2C1);
        // SerialDebug::print("ERR:I2C1_ARBRITRATION_LOSS\r\n");
    }

    if (LL_I2C_IsActiveFlag_OVR(I2C1) != 0) {
        LL_I2C_ClearFlag_OVR(I2C1);
        // SerialDebug::print("ERR:I2C1_OVER_UNDERRUN\r\n");
    }
}

/*------------------------------------------------------------------------------------------------*/
// private functions
/*------------------------------------------------------------------------------------------------*/

namespace {

/// @brief Start a transfer from memory to the I2C TX buffer.
auto dma_tx_start_transfer(const std::byte* const source, const size_t length) -> void {
    LL_DMA_SetMemoryAddress(DMA1, DMA_TX_CHANNEL, (uint32_t)source);
    LL_DMA_SetDataLength(DMA1, DMA_TX_CHANNEL, length);
    LL_DMA_SetMode(DMA1, DMA_TX_CHANNEL, LL_DMA_MODE_NORMAL);
    LL_DMA_EnableChannel(DMA1, DMA_TX_CHANNEL);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Start a transfer from the I2C RX buffer to memory.
auto dma_rx_start_transfer(const std::byte* const destination, const size_t length) -> void {
    // Setup the tx to transfer dummy bytes to the tx register.
    LL_DMA_SetMemoryAddress(DMA1, DMA_TX_CHANNEL, (uint32_t)&dummy_byte);
    LL_DMA_SetDataLength(DMA1, DMA_TX_CHANNEL, 1);
    LL_DMA_SetMode(DMA1, DMA_TX_CHANNEL, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetMemoryAddress(DMA1, DMA_RX_CHANNEL, (uint32_t)destination);
    LL_DMA_SetDataLength(DMA1, DMA_RX_CHANNEL, length);

    LL_DMA_EnableChannel(DMA1, DMA_RX_CHANNEL);
    LL_DMA_EnableChannel(DMA1, DMA_TX_CHANNEL);
}

}

/*------------------------------------------------------------------------------------------------*/
