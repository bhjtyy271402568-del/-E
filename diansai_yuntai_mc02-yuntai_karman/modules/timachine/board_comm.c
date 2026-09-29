#include "board_comm.h"


static board_comm_data_t board_comm_data;
static USARTInstance *board_comm_usart_instance;
static DaemonInstance *board_comm_daemon_instance;

// 计算简单校验和
static uint32_t board_comm_checksum(uint8_t *data, uint8_t len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

// 解析接收到的数据包
static void board_comm_decode(const uint8_t *buffer)
{
    board_comm_packet_t *packet = (board_comm_packet_t *)buffer;
    
    // 验证帧头
    if (packet->header[0] != 0x55 || packet->header[1] != 0xAA)
        return;
    
    // 验证校验和（包含整型和浮点型数据）
    uint8_t data_len = sizeof(packet->int_data) + sizeof(packet->float_data);
    uint32_t calc_sum = board_comm_checksum((uint8_t *)packet->int_data, data_len);
    if (calc_sum != packet->checksum)
        return;
    
    // 复制数据
    memcpy(board_comm_data.rx_int_data, packet->int_data, sizeof(packet->int_data));
    memcpy(board_comm_data.rx_float_data, packet->float_data, sizeof(packet->float_data));
}

// 串口接收回调
static void BoardCommRxCallback()
{
    // 暂时注释掉 daemon 相关调用，专注于数据传输
    // DaemonReload(board_comm_daemon_instance); 
    board_comm_decode(board_comm_usart_instance->recv_buff);
}

// 通信丢失回调
static void BoardCommLostCallback(void *id)
{
    memset(board_comm_data.rx_int_data, 0, sizeof(board_comm_data.rx_int_data));
    memset(board_comm_data.rx_float_data, 0, sizeof(board_comm_data.rx_float_data));
    USARTServiceInit(board_comm_usart_instance);
    LOGWARNING("[board_comm] communication lost");
}

// 初始化板间通信
board_comm_data_t* BoardCommInit(UART_HandleTypeDef *uart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = BoardCommRxCallback;
    conf.usart_handle = uart_handle;
    conf.recv_buff_size = BOARD_COMM_FRAME_SIZE;
    board_comm_usart_instance = USARTRegister(&conf);

    // 暂时注释掉 daemon 注册，专注于数据传输
    /*
    // 注册守护进程
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 20,  // 200ms未收到数据视为离线
        .callback = BoardCommLostCallback,
        .owner_id = NULL,
    };
    board_comm_daemon_instance = DaemonRegister(&daemon_conf);
    */

    memset(&board_comm_data, 0, sizeof(board_comm_data));

    return &board_comm_data;
}

// 检查通信状态
uint8_t BoardCommIsOnline(void)
{
    // 暂时返回固定值，专注于数据传输
    return 1;
    // return DaemonIsOnline(board_comm_daemon_instance);
}

// 发送状态数组
void BoardCommSend(int32_t int_data[2], float float_data[2])
{
    board_comm_packet_t packet;
    
    // 设置帧头
    packet.header[0] = 0x55;
    packet.header[1] = 0xAA;
    
    // 复制数据
    memcpy(packet.int_data, int_data, sizeof(packet.int_data));
    memcpy(packet.float_data, float_data, sizeof(packet.float_data));
    memcpy(board_comm_data.tx_int_data, int_data, sizeof(board_comm_data.tx_int_data));
    memcpy(board_comm_data.tx_float_data, float_data, sizeof(board_comm_data.tx_float_data));
    
    // 计算校验和
    uint8_t data_len = sizeof(packet.int_data) + sizeof(packet.float_data);
    packet.checksum = board_comm_checksum((uint8_t *)packet.int_data, data_len);
    
    // 发送数据
    USARTSend(board_comm_usart_instance, (uint8_t *)&packet, sizeof(packet), USART_TRANSFER_IT);
}

// 获取接收到的整型数组
int32_t* BoardCommGetIntData(void)
{
    return board_comm_data.rx_int_data;
}

// 获取接收到的浮点型数组
float* BoardCommGetFloatData(void)
{
    return board_comm_data.rx_float_data;
}