/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : comms.c
    Description     : communication functions
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_usart.h"

#include "comms.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

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

/*
 * @brief       setup communication peripherals
 * @param       none
 * @retval      none
 */
void comms_initialise( void )
{
    /*
     * Setup USART1
     */
    LL_USART_InitTypeDef *initialisation_structure  = NULL;
    initialisation_structure->BaudRate              = 115200;
    initialisation_structure->DataWidth             = LL_USART_DATAWIDTH_8B;
    initialisation_structure->HardwareFlowControl   = LL_USART_HWCONTROL_NONE;
    initialisation_structure->OverSampling          = LL_USART_OVERSAMPLING_8;
    initialisation_structure->Parity                = LL_USART_PARITY_NONE;
    initialisation_structure->StopBits              = LL_USART_STOPBITS_1;
    initialisation_structure->TransferDirection     = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init( USART1, initialisation_structure );
    LL_USART_EnableIT_RXNE( USART1 );
    LL_USART_Enable( USART1 );


}

/*----------------------------------------------------------------------------*/

void comms_usart1_isr( void )
{

}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/