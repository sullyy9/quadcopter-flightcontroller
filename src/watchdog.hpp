#pragma once
/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 * 
 * @file    watchdog.hpp
 * @brief   header
 * 
 * @date    2022-02-26
 * -------------------------------------------------------------------------------------------------
 */

#include <chrono>

namespace watchdog {
/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

class Watchdog
{
public:
    Watchdog(float timeout_period);
    ~Watchdog();
    
    void update(void);

private:

};

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
}
