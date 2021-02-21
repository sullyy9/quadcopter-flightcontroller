/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :   commonio.c
    Description     :   functions common to STM32
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_rcc.h"

#include "commonio.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define     COMMONIO_HCLK_HZ        48000000L                       // HCLK

#define     COMMONIO_CLOCK_HZ       (COMMONIO_HCLK_HZ)           // peripheral clock = HCLK/8
#define     COMMONIO_TIMER_1US      (COMMONIO_CLOCK_HZ/1000000)     // 1.000us
#define     COMMONIO_TIMER_1MS      ( 1L*(COMMONIO_CLOCK_HZ/1000))  // 1.000ms
#define     COMMONIO_TIMER_20MS     (20L*(COMMONIO_CLOCK_HZ/1000))  // 20.000ms

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void commonio_clear_reset_flags( void )
{
    LL_RCC_ClearResetFlags();
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       Setup HSI and LSI clocks
 * @param       none
 * @retval      none
 */
void commonio_initialise_clocks( void )
{
    /*
     * Setup clock sources ( HSI running at 8MHz, LSI at 40KHz )
     */
    LL_RCC_DeInit( );
    LL_RCC_HSI_Enable( );
    LL_RCC_HSE_Disable( );
    LL_RCC_LSI_Enable( );
    LL_RCC_LSE_Disable( );
    while( LL_RCC_HSI_IsReady( ) == 0 );

    /*
     * Setup system, AHB and APB clocks ( all set to  MHz )
     * AHB and APB2 = 48MHz
     * APB1 = 24MHz
     */
    LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
    LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
    LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );

    LL_RCC_PLL_SetMainSource( LL_RCC_PLLSOURCE_HSI_DIV_2 );
    LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12 );
    LL_RCC_PLL_Enable( );
    while( LL_RCC_PLL_IsReady( ) == 0 );

    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );
    while( LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL );

    /*
     * Enable peripheral clocks
     */
    LL_AHB1_GRP1_EnableClock
    (
          LL_AHB1_GRP1_PERIPH_GPIOA
        | LL_AHB1_GRP1_PERIPH_GPIOB
        | LL_AHB1_GRP1_PERIPH_GPIOC
        | LL_AHB1_GRP1_PERIPH_GPIOD
        | LL_AHB1_GRP1_PERIPH_GPIOE
        | LL_AHB1_GRP1_PERIPH_GPIOF
    );

    LL_APB2_GRP1_EnableClock
    (
          LL_APB2_GRP1_PERIPH_USART1
    );

    SysTick_Config( COMMONIO_TIMER_1MS );
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
