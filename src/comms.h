/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : comms.h
    Description     : header file
*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void        comms_initialise( void );
uint16_t    comms_usart1_tx_free( void );
void        comms_usart1_tx_byte( uint8_t data );
void        comms_dma1_channel4_isr( void );
void        comms_usart1_isr( void );

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
