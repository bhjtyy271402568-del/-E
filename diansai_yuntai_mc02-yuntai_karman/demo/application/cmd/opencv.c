#include "opencv.h"

#include "master_process.h"

float opencv_Vision_data[12] = {0};
uint8_t Vison_revc_flag = 0;
float Vison_Frequency = 0.0f;

void Vision_Uart_Init(UART_HandleTypeDef *_handle)
{
    (void)VisionInit(_handle);
}
