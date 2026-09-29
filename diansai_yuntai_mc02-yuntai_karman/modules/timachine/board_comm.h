#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include <stdint.h>
#include "string.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"


#define BOARD_COMM_FRAME_SIZE 22u   // 帧大小: 2字节帧头 + 8字节int32 + 8字节float + 4字节校验
#define BOARD_COMM_DATA_SIZE  16u   // 数据大小: 8字节int32 + 8字节float

// 通信数据包结构
typedef struct
{
    uint8_t header[2];          // 帧头 0x55 0xAA
    int32_t int_data[2];        // 2个int32整型值
    float float_data[2];        // 2个float浮点型值
    uint32_t checksum;          // 校验和
} __attribute__((packed)) board_comm_packet_t;

// 板间通信实例
typedef struct
{
    int32_t tx_int_data[2];     // 发送的整型数组
    float tx_float_data[2];     // 发送的浮点型数组
    int32_t rx_int_data[2];     // 接收的整型数组
    float rx_float_data[2];     // 接收的浮点型数组
} board_comm_data_t;

// 函数声明
board_comm_data_t* BoardCommInit(UART_HandleTypeDef *uart_handle);
uint8_t BoardCommIsOnline(void);
void BoardCommSend(int32_t int_data[2], float float_data[2]);
int32_t* BoardCommGetIntData(void);
float* BoardCommGetFloatData(void);

#endif // BOARD_COMM_H