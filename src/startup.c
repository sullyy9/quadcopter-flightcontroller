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

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx.h"

#include "main.h"
#include "debug.h"
#include "io.h"
#include "system.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"

/*----------------------------------------------------------------------------*/

extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;
extern unsigned long _estack;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void reset_isr(void) __attribute__((__interrupt__, used));
void dummy_isr(void) __attribute__((__interrupt__, used));
void non_maskable_int_isr(void) __attribute__((__interrupt__, used));
void hard_fault_isr(void) __attribute__((__interrupt__, used));
void memory_management_isr(void) __attribute__((__interrupt__, used));
void bus_fault_isr(void) __attribute__((__interrupt__, used));
void usage_fault_isr(void) __attribute__((__interrupt__, used));
void watch_dog_isr(void) __attribute__((__interrupt__, used));

/*----------------------------------------------------------------------------*/
/*-vector-table---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* stm32 vectors for application - these are linked to the start of the App area */
__attribute__((section(".isr_vector"), used)) static void (*const vector_table[])(void) = {
    /* 0x0000 Reset Stack Pointer        */ (void (*)(void))((unsigned long)&_estack),
    /* 0x0004 Reset Vector               */ reset_isr,
    /* 0x0008 NonMaskableInt_IRQn        */ non_maskable_int_isr,
    /* 0x000c HardFault_IRQn             */ hard_fault_isr,
    /* 0x0010 MemoryManagement_IRQn      */ memory_management_isr,
    /* 0x0014 BusFault_IRQn              */ bus_fault_isr,
    /* 0x0018 UsageFault_IRQn            */ usage_fault_isr,
    /* 0x001C Reserved                   */ 0,
    /* 0x0020 Reserved                   */ 0,
    /* 0x0024 Reserved                   */ 0,
    /* 0x0028 Reserved                   */ 0,
    /* 0x002c SVCall_IRQn                */ dummy_isr,
    /* 0x0030 DebugMonitor_IRQn          */ dummy_isr,
    /* 0x0034 Reserved                   */ 0,
    /* 0x0038 PendSV_IRQn                */ dummy_isr,
    /* 0x003C SysTick_IRQn               */ main_1ms_timer_isr,
    /* 0x0040 WWDG_IRQn             (00) */ system_wwdg_isr,
    /* 0x0044 PVD_IRQn              (01) */ dummy_isr,
    /* 0x0048 TAMPER_IRQn           (02) */ dummy_isr,
    /* 0x004C RTC_IRQn              (03) */ dummy_isr,
    /* 0x0050 FLASH_IRQn            (04) */ dummy_isr,
    /* 0x0054 RCC_IRQn              (05) */ dummy_isr,
    /* 0x0058 EXTI0_IRQn            (06) */ dummy_isr,
    /* 0x005C EXTI1_IRQn            (07) */ external_interupt_1_isr,
    /* 0x0060 EXTI2_IRQn            (08) */ external_interupt_2_isr,
    /* 0x0064 EXTI3_IRQn            (09) */ dummy_isr,
    /* 0x0068 EXTI4_IRQn            (10) */ external_interupt_4_isr,
    /* 0x006C DMA1_Channel1_IRQn    (11) */ dummy_isr,
    /* 0x0070 DMA1_Channel2_IRQn    (12) */ spi1_dma1_channel2_isr,
    /* 0x0074 DMA1_Channel3_IRQn    (13) */ spi1_dma1_channel3_isr,
    /* 0x0078 DMA1_Channel4_IRQn    (14) */ usart_dma1_channel4_isr,
    /* 0x007C DMA1_Channel5_IRQn    (15) */ dummy_isr,
    /* 0x0080 DMA1_Channel6_IRQn    (16) */ i2c1_dma1_channel6_isr,
    /* 0x0084 DMA1_Channel7_IRQn    (17) */ i2c1_dma1_channel7_isr,
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
    /* 0x00BC I2C1_EV_IRQn          (31) */ i2c1_ev_isr,
    /* 0x00C0 I2C1_ER_IRQn          (32) */ i2c1_er_isr,
    /* 0x00C4 I2C2_EV_IRQn          (33) */ dummy_isr,
    /* 0x00C8 I2C2_ER_IRQn          (34) */ dummy_isr,
    /* 0x00CC SPI1_IRQn             (35) */ spi1_error_isr,
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
    /* FSMC_IRQn                    (48) */ 0,
    /* SDIO_IRQn                    (49) */ 0,
    /* TIM5_IRQn                    (50) */ 0,
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
    /* Reserved                     (63) */ 0,
    /* Reserved                     (64) */ 0,
    /* COMP1_2_3_IRQn               (65) */ dummy_isr,
    /* COMP4_5_6_IRQn               (66) */ dummy_isr,
    /* COMP7_IRQn                   (66) */ dummy_isr,
    /* Reserved                     (67) */ 0,
    /* Reserved                     (68) */ 0,
    /* Reserved                     (69) */ 0,
    /* Reserved                     (70) */ 0,
    /* Reserved                     (71) */ 0,
    /* Reserved                     (72) */ 0,
    /* Reserved                     (73) */ 0,
    /* USB_HP_IRQn                  (74) */ dummy_isr,
    /* USB_LP_IRQn                  (75) */ dummy_isr,
    /* USB_WU_RMP_IRQn              (76) */ dummy_isr,
    /* Reserved                     (77) */ 0,
    /* Reserved                     (78) */ 0,
    /* Reserved                     (79) */ 0,
    /* Reserved                     (80) */ 0,
    /* FPU_IRQn                     (81) */ dummy_isr
};

/*----------------------------------------------------------------------------*/
/*-startup-code---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void reset_isr(void)
{
    SCB->VTOR = 0 | ((uint32_t)vector_table & (uint32_t)0x1FFFFF80);

    unsigned long *pSrc, *pDest;

    /*
     * copy the data segment initialisers from flash to SRAM
     */
    pSrc = &_etext;
    for(pDest = &_sdata; pDest < &_edata;)
    {
        *pDest++ = *pSrc++;
    }

    /*
     * zero fill the bss segment
     */
    for(pDest = &__bss_start__; pDest < &__bss_end__;)
    {
        *pDest++ = 0;
    }

    /*
     * call the application
     */
    main(); // NOLINT - ignore clangd warning

    /*
     * if main exits, sit here
     */
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void dummy_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void non_maskable_int_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void hard_fault_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void memory_management_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void bus_fault_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void usage_fault_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/

void watch_dog_isr(void)
{
    while(1) {}
}

/*----------------------------------------------------------------------------*/
