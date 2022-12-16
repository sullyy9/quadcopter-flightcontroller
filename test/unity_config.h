#pragma once

#include "usart.hpp"
#include "debug.hpp"

#define UNITY_OUTPUT_CHAR(a)    debug::printf("char\r\n")
#define UNITY_OUTPUT_START()    debug::printf("unity start\r\n")
#define UNITY_OUTPUT_FLUSH()    while(!usart::tx_free())
#define UNITY_OUTPUT_COMPLETE() while(!usart::tx_free())

#define UNITY_PRINT_EOL { UNITY_OUTPUT_CHAR('\r'); UNITY_OUTPUT_CHAR('\n'); }