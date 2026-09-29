#include "step_motor.h"
#include "X_V2.h"
#include "usart.h"

#define MOTOR_UHART0 &huart1
#define MOTOR_UHART1 &huart1    




void step_sendcmd(uint8_t addr ,uint8_t *cmd, uint16_t len)
{
    // 发送步进电机控制命令
    if (len > 0 && cmd != NULL)
    {
        if(addr == 1 ||  addr == 2)
        {
            HAL_UART_Transmit_DMA(MOTOR_UHART0, cmd, len );
        }
        else if (addr == 3)
        {
            HAL_UART_Transmit_DMA(MOTOR_UHART1, cmd, len );
        }

    }
}