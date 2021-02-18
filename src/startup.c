/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan
    
    Module Name     : Startup.c
    Description     : startup code for main program
*/
/*----------------------------------------------------------------------------*/

#include "commonio.h"
#include "io.h"
#include "main.h"

/*----------------------------------------------------------------------------*/

extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long __bss_start__;
extern unsigned long __bss_end__;
extern unsigned long __stack_end__;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void reset_isr(void) __attribute__((__interrupt__,used));
void dummy_isr(void) __attribute__((__interrupt__,used));

/*----------------------------------------------------------------------------*/
/*-vector-table---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* stm32 vectors for application - these are linked to the start of the App area */
__attribute__ ((section(".vectors"), used))
void (* const vector_table[])(void) = 
{
/* 0x0000 Reset Stack Pointer        */  (void (*)(void))((unsigned long)&__stack_end__),
/* 0x0004 Reset Vector               */  reset_isr,
/* 0x0008 NonMaskableInt_IRQn        */  dummy_isr,
/* 0x000c Reserved                   */  dummy_isr,
/* 0x0010 MemoryManagement_IRQn      */  dummy_isr,
/* 0x0014 BusFault_IRQn              */  dummy_isr,
/* 0x0018 UsageFault_IRQn            */  dummy_isr,
/* 0x001C Reserved                   */  0,
/* 0x0020 Reserved                   */  0,
/* 0x0024 Reserved                   */  0,
/* 0x0028 Reserved                   */  0,
/* 0x002c SVCall_IRQn                */  dummy_isr,
/* 0x0030 DebugMonitor_IRQn          */  dummy_isr,
/* 0x0034 Reserved                   */  0,
/* 0x0038 PendSV_IRQn                */  dummy_isr,
/* 0x003C SysTick_IRQn               */  main_1ms_timer_isr,
/* 0x0040 WWDG_IRQn             (00) */  dummy_isr,
/* 0x0044 PVD_IRQn              (01) */  dummy_isr,
/* 0x0048 TAMPER_IRQn           (02) */  dummy_isr,
/* 0x004C RTC_IRQn              (03) */  dummy_isr,
/* 0x0050 FLASH_IRQn            (04) */  dummy_isr,
/* 0x0054 RCC_IRQn              (05) */  dummy_isr,
/* 0x0058 EXTI0_IRQn            (06) */  dummy_isr,
/* 0x005C EXTI1_IRQn            (07) */  dummy_isr,
/* 0x0060 EXTI2_IRQn            (08) */  dummy_isr,
/* 0x0064 EXTI3_IRQn            (09) */  dummy_isr,
/* 0x0068 EXTI4_IRQn            (10) */  dummy_isr,
/* 0x006C DMA1_Channel1_IRQn    (11) */  dummy_isr,
/* 0x0070 DMA1_Channel2_IRQn    (12) */  dummy_isr,
/* 0x0074 DMA1_Channel3_IRQn    (13) */  dummy_isr,
/* 0x0078 DMA1_Channel4_IRQn    (14) */  dummy_isr,
/* 0x007C DMA1_Channel5_IRQn    (15) */  dummy_isr,
/* 0x0080 DMA1_Channel6_IRQn    (16) */  dummy_isr,
/* 0x0084 DMA1_Channel7_IRQn    (17) */  dummy_isr,
/* 0x0088 ADC1_2_IRQn           (18) */  dummy_isr,
/* 0x008C USB_HP_CAN1_TX_IRQn   (19) */  dummy_isr,
/* 0x0090 USB_LP_CAN1_RX0_IRQn  (20) */  dummy_isr,
/* 0x0094 CAN1_RX1_IRQn         (21) */  dummy_isr,
/* 0x0098 CAN1_SCE_IRQn         (22) */  dummy_isr,
/* 0x009C EXTI9_5_IRQn          (23) */  dummy_isr,
/* 0x00A0 TIM1_BRK_IRQn         (24) */  dummy_isr,
/* 0x00A4 TIM1_UP_IRQn          (25) */  dummy_isr,
/* 0x00A8 TIM1_TRG_COM_IRQn     (26) */  dummy_isr,
/* 0x00AC TIM1_CC_IRQn          (27) */  dummy_isr,
/* 0x00B0 TIM2_IRQn             (28) */  dummy_isr,
/* 0x00B4 TIM3_IRQn             (29) */  dummy_isr,
/* 0x00B8 TIM4_IRQn             (30) */  dummy_isr,
/* 0x00BC I2C1_EV_IRQn          (31) */  dummy_isr,
/* 0x00C0 I2C1_ER_IRQn          (32) */  dummy_isr,
/* 0x00C4 I2C2_EV_IRQn          (33) */  dummy_isr,
/* 0x00C8 I2C2_ER_IRQn          (34) */  dummy_isr,
/* 0x00CC SPI1_IRQn             (35) */  dummy_isr,
/* 0x00D0 SPI2_IRQn             (36) */  dummy_isr,
/* 0x00D4 USART1_IRQn           (37) */  dummy_isr,
/* 0x00D8 USART2_IRQn           (38) */  dummy_isr,
/* 0x00DC USART3_IRQn           (39) */  dummy_isr,
/* 0x00E0 EXTI15_10_IRQn        (40) */  dummy_isr,
/* 0x00E4 RTCAlarm_IRQn         (41) */  dummy_isr,
/* 0x00E8 USBWakeUp_IRQn        (42) */  dummy_isr,
/* TIM8_BRK_IRQn                (43) */  dummy_isr,
/* TIM8_UP_IRQn                 (44) */  dummy_isr,
/* TIM8_TRG_COM_IRQn            (45) */  dummy_isr,
/* TIM8_CC_IRQn                 (46) */  dummy_isr,
/* ADC3_IRQn                    (47) */  dummy_isr,
/* FSMC_IRQn                    (48) */  dummy_isr,
/* SDIO_IRQn                    (49) */  dummy_isr,
/* TIM5_IRQn                    (50) */  dummy_isr,
/* SPI3_IRQn                    (51) */  dummy_isr,
/* UART4_IRQn                   (52) */  dummy_isr,
/* UART5_IRQn                   (53) */  dummy_isr,
/* TIM6_IRQn                    (54) */  dummy_isr,
/* TIM7_IRQn                    (55) */  dummy_isr,
/* DMA2_Channel1_IRQn           (56) */  dummy_isr,
/* DMA2_Channel2_IRQn           (57) */  dummy_isr,
/* DMA2_Channel3_IRQn           (58) */  dummy_isr,
/* DMA2_Channel4_5_IRQn         (59) */  dummy_isr
};

/*----------------------------------------------------------------------------*/
/*-startup-code---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void reset_isr(void)
{
    unsigned long *pSrc, *pDest;

    /*
     * copy the data segment initialisers from flash to SRAM
     */
    pSrc = &_etext;
    for(pDest = &_sdata; pDest < &_edata; )
    {
            *pDest++ = *pSrc++;
    }

    /*
     * zero fill the bss segment
     */
    for(pDest = &__bss_start__; pDest < &__bss_end__; )
    {
            *pDest++ = 0;
    }

    /*
     * call the application
     */
    main();
    
    /*
     * if main exits, sit here
     */
    while( 1 );
}

/*----------------------------------------------------------------------------*/

void dummy_isr( void )
{
    while( 1 );
}

/*----------------------------------------------------------------------------*/
