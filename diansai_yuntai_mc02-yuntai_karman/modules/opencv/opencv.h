#ifndef OPENCV_H
#define OPENCV_H
#include "bsp_usart.h"
#include "opencv.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "usart.h"
#include "robot_def.h"
#include <string.h>
#include <stdio.h>




void Vision_Uart_Init(UART_HandleTypeDef *_handle);
extern float opencv_Vision_data[12]; // 视觉数据,12个float
extern uint8_t Vison_revc_flag; // 视觉数据接收标志位
extern uint8_t vision_used_flag; // 视觉数据使用标志位,用于判断是否需要使用视觉数据
extern float Vison_Frequency; // 视觉数据频率


#endif // !OPENCV_H
