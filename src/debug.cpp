////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    debug.cpp
/// @brief   Module containing debugging utilities.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stm32f3xx.h"
#include "core_cm4.h"

#include "usart.hpp"
#include "debug.hpp"

/*------------------------------------------------------------------------------------------------*/
// Constants
/*------------------------------------------------------------------------------------------------*/

static constinit uint32_t CORE_SPEED {48'000'000};

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

/// @brief Setup the CPU cycle counter.
/// 
void debug::stopwatch_initialise(void)
{
    // Enable the cycle counter. 
    SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
    SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
}

/*------------------------------------------------------------------------------------------------*/


/// @brief Start counting CPU cycles.
///
void debug::stopwatch_start(void) {
    WRITE_REG(DWT->CYCCNT, 0);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Stop counting CPU cycles and figure out how many nano-seconds they equate to.
///        Will be accurate assuming less than 89 seconds has passed and core is running
///        at 48MHz.
///
/// @return uint32_t Time elapsed in nano-seconds.
/// 
uint32_t debug::stopwatch_stop(void) {
    const uint32_t stop_time = READ_REG(DWT->CYCCNT);
    return ((stop_time * 1000) / (CORE_SPEED / 1000000));
}

/*------------------------------------------------------------------------------------------------*/
