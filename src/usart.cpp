////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    usart.cpp
/// @brief   Module for controlling the USART peripheral
////////////////////////////////////////////////////////////////////////////////////////////////////


#include <optional>
#include <functional>
#include <span>
#include <algorithm>
#include <utility>

#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/

// Tx buffer for USART. 
static constinit std::array<volatile std::byte, 512> tx_buffer {};
static constinit auto tx_in_buffer  {std::span(tx_buffer).first(tx_buffer.size() / 2)};
static constinit auto tx_out_buffer {std::span(tx_buffer).last(tx_buffer.size() / 2)};

static constinit volatile uint32_t tx_in_index {0};

/*------------------------------------------------------------------------------------------------*/
// Forward declarations.
/*------------------------------------------------------------------------------------------------*/

auto get_data_bits(const uint32_t bits) -> std::optional<decltype(LL_USART_InitTypeDef::DataWidth)>;
auto get_stop_bits(const uint32_t bits) -> std::optional<decltype(LL_USART_InitTypeDef::StopBits)>;
auto get_parity(const usart::Parity parity) -> decltype(LL_USART_InitTypeDef::Parity);
auto get_flow_control(const usart::FlowControl flow_control) -> decltype(LL_USART_InitTypeDef::HardwareFlowControl);

/*------------------------------------------------------------------------------------------------*/
// Class method definitions.
/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise an instance of a USART peripheral.
/// 
/// @param config Peripheral configuration.
///
auto usart::USART::init(const USARTConfig& config) -> std::tuple<std::optional<USART>, std::error_code> {
    LL_USART_InitTypeDef usart_conf {};
    usart_conf.Parity = get_parity(config.parity);
    usart_conf.HardwareFlowControl = get_flow_control(config.flow_control);
    usart_conf.OverSampling = LL_USART_OVERSAMPLING_8;

    // Baud rate TODO: check its valid.
    if(config.baud_rate) {
        usart_conf.BaudRate = config.baud_rate;
    }
    else return {std::nullopt, StatusCode::InvalidBaudRate};

    // Data bits.
    if(auto data_bits = get_data_bits(config.data_bits); data_bits.has_value()) {
        usart_conf.DataWidth = data_bits.value();
    }
    else return {std::nullopt, StatusCode::InvalidDataBits};

    // Stop bits.
    if(auto stop_bits = get_stop_bits(config.stop_bits); stop_bits.has_value()) {
        usart_conf.StopBits = stop_bits.value();
    }
    else return {std::nullopt, StatusCode::InvalidStopBits};

    // Rx / Tx enable.
    if(config.enable_rx && config.enable_tx) {
        usart_conf.TransferDirection = LL_USART_DIRECTION_TX_RX;
    }
    else if(config.enable_rx) {
        usart_conf.TransferDirection = LL_USART_DIRECTION_RX;
    }
    else if(config.enable_tx) {
        usart_conf.TransferDirection = LL_USART_DIRECTION_TX;
    }
    else return {std::nullopt, StatusCode::InvalidTxRxConf};

    // Configure the USART peripheral.
    LL_USART_Init(USART1, &usart_conf);
    LL_USART_Enable(USART1);

    // Configure the DMA peripheral if requested.
    if(config.enable_dma) {
        LL_DMA_InitTypeDef dma {
            .PeriphOrM2MSrcAddress  = LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
            .MemoryOrM2MDstAddress  = reinterpret_cast<uint32_t>(tx_out_buffer.data()),
            .Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
            .Mode                   = LL_DMA_MODE_NORMAL,
            .PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT,
            .MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .NbData                 = tx_out_buffer.size(),
            .Priority               = LL_DMA_PRIORITY_MEDIUM,
        };
        LL_DMA_Init(DMA1, LL_DMA_CHANNEL_4, &dma);
        LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

        LL_USART_EnableDMAReq_TX(USART1);
        LL_USART_ClearFlag_TC(USART1);

        NVIC_SetPriority(DMA1_Channel4_IRQn, 3);
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    }
    else return {std::nullopt, StatusCode::Unimplemented};

    return {USART(), StatusCode::Ok};
}

/*------------------------------------------------------------------------------------------------*/

/// @brief           Return the ammount of free space in the USART TX buffer.
/// @return uint32_t Free space.
/// 
auto usart::USART::tx_free() -> uint32_t {
    return tx_in_buffer.size() - tx_in_index;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief      Write a byte into the USART TX buffer. Start the DMA channel if the buffer is full.
/// @param byte Byte to transmit.
/// 
auto usart::USART::tx_byte(const std::byte byte) -> void {
    tx_in_buffer[tx_in_index++] = byte;
    if(tx_in_index >= tx_in_buffer.size()) tx_flush();
}

/*------------------------------------------------------------------------------------------------*/

/// @brief      Swap the tx in and out buffers and start a DMA transaction to send data in the out
///             buffer to the UART TX register. If a DMA transaction is already in progress, block
///             until it is finshed. 
/// 
auto usart::USART::tx_flush() -> void {
    if(tx_in_index > 0) {
        while(LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4));

        std::swap(tx_in_buffer, tx_out_buffer);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, reinterpret_cast<uint32_t>(tx_out_buffer.data()));
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, tx_in_index);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
        tx_in_index = 0;
    }
}

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

/// @brief Interrupt for DMA1 channel 4. Called when the DMA has transferred half and all of the TX
///        buffer.
/// 
auto usart::dma1_channel4_isr() -> void {
    if(LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    }
}

/*------------------------------------------------------------------------------------------------*/
// Module private functions.
/*------------------------------------------------------------------------------------------------*/

auto get_data_bits(const uint32_t bits) -> std::optional<decltype(LL_USART_InitTypeDef::DataWidth)> {
    switch(bits) {
        case 8:  return LL_USART_DATAWIDTH_8B;
        case 9:  return LL_USART_DATAWIDTH_9B;
        default: return std::nullopt;
    }
}

/*------------------------------------------------------------------------------------------------*/

auto get_stop_bits(const uint32_t bits) -> std::optional<decltype(LL_USART_InitTypeDef::StopBits)> {
    switch(bits) {
        case 1: return LL_USART_STOPBITS_1;
        case 2: return LL_USART_STOPBITS_2;
        default: return std::nullopt;
    }
}

/*------------------------------------------------------------------------------------------------*/

auto get_parity(const usart::Parity parity) -> decltype(LL_USART_InitTypeDef::Parity) {
    using enum usart::Parity;
    switch(parity) {
        case None: return LL_USART_PARITY_NONE;
        case Even: return LL_USART_PARITY_EVEN;
        case Odd:  return LL_USART_PARITY_ODD;
        default: return 0;
    }
}

/*------------------------------------------------------------------------------------------------*/

auto get_flow_control(const usart::FlowControl flow_control) -> decltype(LL_USART_InitTypeDef::HardwareFlowControl) {
    using enum usart::FlowControl;
    switch(flow_control) {
        case None:    return LL_USART_HWCONTROL_NONE;
        case RTS:     return LL_USART_HWCONTROL_RTS;
        case CTS:     return LL_USART_HWCONTROL_CTS;
        case RTS_CTS: return LL_USART_HWCONTROL_RTS_CTS;
        default: return 0;
    }
}

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct UsartStatusCategory : std::error_category {
    const char* name() const noexcept override;
    std::string message(int ev) const override;
};
const UsartStatusCategory USART_STATUS_CATEGORY {};

/*------------------------------------------------------------------------------------------------*/
 
const char* UsartStatusCategory::name() const noexcept {
    return "USART";
}

/*------------------------------------------------------------------------------------------------*/

std::string UsartStatusCategory::message(int status) const {
    using enum usart::StatusCode;
    switch (static_cast<usart::StatusCode>(status)) {
        case Ok:              return "Ok";
        case Unimplemented:   return "Use of unimplemented feature";
        case InvalidBaudRate: return "Invalid baud rate";
        case InvalidDataBits: return "Invalid data bits";
        case InvalidStopBits: return "Invalid stop bits";
        case InvalidTxRxConf: return "Invalid Tx/Rx configuration";
        default:              return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

std::error_code usart::make_error_code(usart::StatusCode e) {
    return {static_cast<int>(e), USART_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
