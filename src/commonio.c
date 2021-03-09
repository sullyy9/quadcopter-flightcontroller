/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :   commonio.c
    Description     :   functions common to STM32
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_wwdg.h"

#include "io.h"
#include "commonio.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define     HCLK_HZ        48000000L                    // HCLK
#define     PCLK_HZ        (HCLK_HZ)                    // Peripheral bus 1 clock
#define     SYS_CLOCK_HZ   (HCLK_HZ)                    // System timer clock
#define     TIMER_1US      (CLOCK_HZ/1000000)           // 1.000us
#define     TIMER_1MS      ( 1L*(SYS_CLOCK_HZ/1000))    // 1.000ms
#define     TIMER_20MS     (20L*(CLOCK_HZ/1000))        // 20.000ms

#define     WWDG_RESET_TIME_MAX     ( ( 64 * 4096 * ( 1 << 3 ) ) / ( PCLK_HZ / 1000 ) ) // 43ms

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static          uint32_t wwdg_reset_value = 0;

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
 * @brief       Setup window watch dog
 * @param       none
 * @retval      none
 */
void commonio_initialise_wwdg( uint32_t reset_time_ms )
{
    uint32_t wwdg_prescaler = 0;
    if( reset_time_ms < WWDG_RESET_TIME_MAX )
    {
        while
        (
            ( ( wwdg_reset_value * 4096 * ( 1 << wwdg_prescaler ) ) / ( PCLK_HZ / 1000 ) ) <
            reset_time_ms
        )
        {
            wwdg_reset_value++;
            if( wwdg_reset_value > 64 )
            {
                wwdg_reset_value = 0;
                wwdg_prescaler++;
            }
        }
    }
    else
    {
        wwdg_prescaler = 3;
        wwdg_reset_value = 64;
    }

    switch( wwdg_prescaler )
    {
        case 0:
        {
            LL_WWDG_SetPrescaler( WWDG, LL_WWDG_PRESCALER_1 );
            break;
        }
        case 1:
        {
            LL_WWDG_SetPrescaler( WWDG, LL_WWDG_PRESCALER_2 );
            break;
        }
        case 2:
        {
            LL_WWDG_SetPrescaler( WWDG, LL_WWDG_PRESCALER_4 );
            break;
        }
        case 3:
        {
            LL_WWDG_SetPrescaler( WWDG, LL_WWDG_PRESCALER_8 );
            break;
        }
        default:
        {
            break;
        }
    }

    LL_WWDG_SetWindow( WWDG, 127 );
    LL_WWDG_SetCounter( WWDG, ( 63 + wwdg_reset_value ) );

    LL_WWDG_EnableIT_EWKUP( WWDG );
    LL_WWDG_ClearFlag_EWKUP( WWDG );
    NVIC_SetPriority( WWDG_IRQn, 4 );
    NVIC_EnableIRQ( WWDG_IRQn );

    LL_WWDG_Enable( WWDG );

}

/*----------------------------------------------------------------------------*/

void commonio_reset_wwdg( void )
{
    LL_WWDG_SetCounter( WWDG, ( 63 + wwdg_reset_value ) );
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
     * Setup system, AHB and APB clocks ( all set to 48MHz )
     * AHB and APB2 = 48MHz
     * APB1 = 48MHz
     */
    LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
    LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_1 );
    LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );

    LL_FLASH_SetLatency( LL_FLASH_LATENCY_1 );
    LL_FLASH_EnablePrefetch( );

    LL_RCC_PLL_SetMainSource( LL_RCC_PLLSOURCE_HSI_DIV_2 );
    LL_RCC_PLL_ConfigDomain_SYS( LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12 );
    LL_RCC_PLL_Enable( );
    while( LL_RCC_PLL_IsReady( ) == 0 );

    LL_RCC_SetSysClkSource( LL_RCC_SYS_CLKSOURCE_PLL );
    while( LL_RCC_GetSysClkSource( ) != LL_RCC_SYS_CLKSOURCE_STATUS_PLL );

    /*
     * Enable peripheral clocks
     */
    LL_RCC_SetUSARTClockSource( LL_RCC_USART1_CLKSOURCE_PCLK2 );
    while( LL_RCC_GetUSARTClockSource( LL_RCC_USART1_CLKSOURCE ) != LL_RCC_USART1_CLKSOURCE_PCLK2 );

    LL_RCC_SetI2CClockSource( LL_RCC_I2C1_CLKSOURCE_SYSCLK );
    while( LL_RCC_GetI2CClockSource( LL_RCC_I2C1_CLKSOURCE ) != LL_RCC_I2C1_CLKSOURCE_SYSCLK );

    /*
     * AHB clocks
     */
    LL_AHB1_GRP1_EnableClock
    (
          LL_AHB1_GRP1_PERIPH_GPIOA
        | LL_AHB1_GRP1_PERIPH_GPIOB
        | LL_AHB1_GRP1_PERIPH_GPIOC
        | LL_AHB1_GRP1_PERIPH_GPIOD
        | LL_AHB1_GRP1_PERIPH_GPIOE
        | LL_AHB1_GRP1_PERIPH_GPIOF
        | LL_AHB1_GRP1_PERIPH_DMA1
    );

    /*
     * APB1 clocks
     */
    LL_APB1_GRP1_EnableClock
    (
            LL_APB1_GRP1_PERIPH_I2C1
          | LL_APB1_GRP1_PERIPH_WWDG
    );

    /*
     * APB2 clocks
     */
    LL_APB2_GRP1_EnableClock
    (
          LL_APB2_GRP1_PERIPH_SYSCFG
        | LL_APB2_GRP1_PERIPH_USART1
        | LL_APB2_GRP1_PERIPH_SPI1
    );

    SysTick_Config( TIMER_1MS );
}


/*----------------------------------------------------------------------------*/

void commonio_wwdg_isr( void )
{
    LL_WWDG_ClearFlag_EWKUP( WWDG );
    commonio_reset_wwdg( );
    io_fault_led_enable( );
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
