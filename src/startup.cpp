/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 * 
 * @file    startup.c
 * @brief   Vector table and startup code.
 * 
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

#include <cstdint>

#include <cstddef>
#include <cstring>

#include "stm32f3xx.h"

#include "usart_stm32f303.hpp"

#include "main.hpp"
#include "io.hpp"
#include "i2c.hpp"
#include "spi.hpp"

/*----------------------------------------------------------------------------*/

extern unsigned long __data_start__;
extern unsigned long __data_len__;
extern unsigned long __code_end__;

extern unsigned long __bss_start__;
extern unsigned long __bss_len__;

extern unsigned long __stack_start__;
extern unsigned long __stack_end__;
extern unsigned long __stack_len__;

extern unsigned long __heap_start__;
extern unsigned long __heap_len__;

extern unsigned long __empty__;

void *__dso_handle = nullptr; // NOLINT - ignore clangd warning

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

[[noreturn]] auto reset_isr()             -> void;
[[noreturn]] auto dummy_isr()             -> void;
[[noreturn]] auto non_maskable_int_isr()  -> void;
[[noreturn]] auto hard_fault_isr()        -> void;
[[noreturn]] auto memory_management_isr() -> void;
[[noreturn]] auto bus_fault_isr()         -> void;
[[noreturn]] auto usage_fault_isr()       -> void;

/*----------------------------------------------------------------------------*/
/*-vector-table---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* stm32 vectors for application - these are linked to the start of the App area */
__attribute__((section(".isr_vector"), used)) static void (*const vector_table[])(void) = {
    /* 0x0000 Reset Stack Pointer        */ (void (*)(void))((unsigned long)&__stack_end__),
    /* 0x0004 Reset Vector               */ reset_isr,
    /* 0x0008 NonMaskableInt_IRQn        */ non_maskable_int_isr,
    /* 0x000c HardFault_IRQn             */ hard_fault_isr,
    /* 0x0010 MemoryManagement_IRQn      */ memory_management_isr,
    /* 0x0014 BusFault_IRQn              */ bus_fault_isr,
    /* 0x0018 UsageFault_IRQn            */ usage_fault_isr,
    /* 0x001C Reserved                   */ nullptr,
    /* 0x0020 Reserved                   */ nullptr,
    /* 0x0024 Reserved                   */ nullptr,
    /* 0x0028 Reserved                   */ nullptr,
    /* 0x002c SVCall_IRQn                */ dummy_isr,
    /* 0x0030 DebugMonitor_IRQn          */ dummy_isr,
    /* 0x0034 Reserved                   */ nullptr,
    /* 0x0038 PendSV_IRQn                */ dummy_isr,
    /* 0x003C SysTick_IRQn               */ main_1ms_timer_isr,
    /* 0x0040 WWDG_IRQn             (00) */ dummy_isr,
    /* 0x0044 PVD_IRQn              (01) */ dummy_isr,
    /* 0x0048 TAMPER_IRQn           (02) */ dummy_isr,
    /* 0x004C RTC_IRQn              (03) */ dummy_isr,
    /* 0x0050 FLASH_IRQn            (04) */ dummy_isr,
    /* 0x0054 RCC_IRQn              (05) */ dummy_isr,
    /* 0x0058 EXTI0_IRQn            (06) */ dummy_isr,
    /* 0x005C EXTI1_IRQn            (07) */ io::external_interupt_1_isr,
    /* 0x0060 EXTI2_IRQn            (08) */ io::external_interupt_2_isr,
    /* 0x0064 EXTI3_IRQn            (09) */ dummy_isr,
    /* 0x0068 EXTI4_IRQn            (10) */ io::external_interupt_4_isr,
    /* 0x006C DMA1_Channel1_IRQn    (11) */ dummy_isr,
    /* 0x0070 DMA1_Channel2_IRQn    (12) */ spi::dma1_channel2_isr,
    /* 0x0074 DMA1_Channel3_IRQn    (13) */ spi::dma1_channel3_isr,
    /* 0x0078 DMA1_Channel4_IRQn    (14) */ usart::stm32f303::dma1_channel4_isr,
    /* 0x007C DMA1_Channel5_IRQn    (15) */ dummy_isr,
    /* 0x0080 DMA1_Channel6_IRQn    (16) */ i2c::dma1_channel6_isr,
    /* 0x0084 DMA1_Channel7_IRQn    (17) */ i2c::dma1_channel7_isr,
    /* 0x0088 ADC1_2_IRQn           (18) */ dummy_isr,
    /* 0x008C USB_HP_CAN1_TX_IRQn   (19) */ dummy_isr,
    /* 0x0090 USB_LP_CAN1_RX0_IRQn  (20) */ dummy_isr,
    /* 0x0094 CAN1_RX1_IRQn         (21) */ dummy_isr,
    /* 0x0098 CAN1_SCE_IRQn         (22) */ dummy_isr,
    /* 0x009C EXTI9_5_IRQn          (23) */ dummy_isr,
    /* 0x00A0 TIM1_BRK_IRQn         (24) */ dummy_isr,
    /* 0x00A4 TIM1_UP_IRQn          (25) */ dummy_isr,
    /* 0x00A8 TIM1_TRG_COM_IRQn     (26) */ dummy_isr,
    /* 0x00AC TIM1_CC_IRQn          (27) */ dummy_isr,
    /* 0x00B0 TIM2_IRQn             (28) */ dummy_isr,
    /* 0x00B4 TIM3_IRQn             (29) */ dummy_isr,
    /* 0x00B8 TIM4_IRQn             (30) */ dummy_isr,
    /* 0x00BC I2C1_EV_IRQn          (31) */ i2c::ev_isr,
    /* 0x00C0 I2C1_ER_IRQn          (32) */ i2c::er_isr,
    /* 0x00C4 I2C2_EV_IRQn          (33) */ dummy_isr,
    /* 0x00C8 I2C2_ER_IRQn          (34) */ dummy_isr,
    /* 0x00CC SPI1_IRQn             (35) */ spi::error_isr,
    /* 0x00D0 SPI2_IRQn             (36) */ dummy_isr,
    /* 0x00D4 USART1_IRQn           (37) */ dummy_isr,
    /* 0x00D8 USART2_IRQn           (38) */ dummy_isr,
    /* 0x00DC USART3_IRQn           (39) */ dummy_isr,
    /* 0x00E0 EXTI15_10_IRQn        (40) */ dummy_isr,
    /* 0x00E4 RTCAlarm_IRQn         (41) */ dummy_isr,
    /* 0x00E8 USBWakeUp_IRQn        (42) */ dummy_isr,
    /* TIM8_BRK_IRQn                (43) */ dummy_isr,
    /* TIM8_UP_IRQn                 (44) */ dummy_isr,
    /* TIM8_TRG_COM_IRQn            (45) */ dummy_isr,
    /* TIM8_CC_IRQn                 (46) */ dummy_isr,
    /* ADC3_IRQn                    (47) */ dummy_isr,
    /* FSMC_IRQn                    (48) */ nullptr,
    /* SDIO_IRQn                    (49) */ nullptr,
    /* TIM5_IRQn                    (50) */ nullptr,
    /* SPI3_IRQn                    (51) */ dummy_isr,
    /* UART4_IRQn                   (52) */ dummy_isr,
    /* UART5_IRQn                   (53) */ dummy_isr,
    /* TIM6_IRQn                    (54) */ dummy_isr,
    /* TIM7_IRQn                    (55) */ dummy_isr,
    /* DMA2_Channel1_IRQn           (56) */ dummy_isr,
    /* DMA2_Channel2_IRQn           (57) */ dummy_isr,
    /* DMA2_Channel3_IRQn           (58) */ dummy_isr,
    /* DMA2_Channel4_IRQn           (60) */ dummy_isr,
    /* DMA2_Channel5_IRQn           (61) */ dummy_isr,
    /* ADC4_IRQn                    (62) */ dummy_isr,
    /* Reserved                     (63) */ nullptr,
    /* Reserved                     (64) */ nullptr,
    /* COMP1_2_3_IRQn               (65) */ dummy_isr,
    /* COMP4_5_6_IRQn               (66) */ dummy_isr,
    /* COMP7_IRQn                   (66) */ dummy_isr,
    /* Reserved                     (67) */ nullptr,
    /* Reserved                     (68) */ nullptr,
    /* Reserved                     (69) */ nullptr,
    /* Reserved                     (70) */ nullptr,
    /* Reserved                     (71) */ nullptr,
    /* Reserved                     (72) */ nullptr,
    /* Reserved                     (73) */ nullptr,
    /* USB_HP_IRQn                  (74) */ dummy_isr,
    /* USB_LP_IRQn                  (75) */ dummy_isr,
    /* USB_WU_RMP_IRQn              (76) */ dummy_isr,
    /* Reserved                     (77) */ nullptr,
    /* Reserved                     (78) */ nullptr,
    /* Reserved                     (79) */ nullptr,
    /* Reserved                     (80) */ nullptr,
    /* FPU_IRQn                     (81) */ dummy_isr,
};

/*----------------------------------------------------------------------------*/
/*-startup-code---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

[[noreturn]]
void reset_isr(void) {
    SCB->VTOR = 0 | ((uint32_t)vector_table & (uint32_t)0x1FFFFF80);
    
    // Enable the FPU.
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
    
    // Copy the initialised data (which is initialy placed after the read only
    // data in flash) to it's location in RAM.
    memcpy(&__data_start__, &__code_end__, reinterpret_cast<size_t>(&__data_len__)); // NOLINT - ignore clangd warning
    
    // Make sure any variable that are in the uninitialised data section are set
    // to 0. 
    memset(&__bss_start__, 0x00, reinterpret_cast<size_t>(&__bss_len__)); // NOLINT - ignore clangd warning

    // Run the application.
    main(); // NOLINT - ignore clangd warning
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto dummy_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto non_maskable_int_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto hard_fault_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto memory_management_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto bus_fault_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/

[[noreturn]]
auto usage_fault_isr() -> void {
    while(1) {}
}

/*----------------------------------------------------------------------------*/
