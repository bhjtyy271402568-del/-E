#pragma once

#include <stdint.h>
#include "usart.h"

extern float opencv_Vision_data[12];
extern uint8_t Vison_revc_flag;
extern float Vison_Frequency;

void Vision_Uart_Init(UART_HandleTypeDef *_handle);
