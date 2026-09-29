#include "opencv.h"
#include "daemon.h"
#define FRAME_SOF          0xA5
#define FRAME_PAYLOAD_LEN  (2+2+4*12)  // 52
#define FRAME_FULL_LEN     (1+1+FRAME_PAYLOAD_LEN) //54

float opencv_Vision_data[12];

uint8_t Vison_revc_flag = 0; // 视觉数据接收标志位
uint8_t vision_used_flag = 0; // 视觉数据使用标志位,用于判断是否需要使用视觉数据



float Vison_Frequency; // 视觉数据接收频率,单位Hz
static USARTInstance *cv_usart_instance;
static DaemonInstance *OPENCV_vision_daemon_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    DaemonReload(OPENCV_vision_daemon_instance); // 喂狗,防止离线
    static uint32_t counter;
    float dt = DWT_GetDeltaT(&counter); 
    Vison_Frequency = 1.0f / dt; // 计算频率
    const uint16_t len = cv_usart_instance->recv_buff_size;
    uint16_t flag_register;
    uint8_t *buf = cv_usart_instance->recv_buff;
    if (len != FRAME_FULL_LEN) {
        // 长度不符
        LOGERROR("Vision frame length error");
        return;
    }
    if (buf[0] != FRAME_SOF) {
        // SOF 错误
        LOGERROR("Vision frame SOF error");
        return;
    }
    uint8_t payload_len = buf[1];
    if (payload_len != FRAME_PAYLOAD_LEN) {
        // Payload 长度不符
        LOGERROR("Vision frame payload length error");
        return;
    }
    // 解析命令号和 flags（小端）
    uint16_t cmd_id = buf[2] | (buf[3] << 8);
    uint16_t flags  = buf[4] | (buf[5] << 8);

    // 解析 12 个 float
    for (int i = 0; i < 12; i++) {
        // 从 buf[6 + 4*i] 连续读 4 字节
        memcpy(&opencv_Vision_data[i], buf + 6 + 4*i, sizeof(float));
    }
    // // 解析任务标志位(拟合最接近的int)
    // int last_task_cmd = Vison_task_cmd;
    // Vison_task_cmd = (int)(Vison_data[10] + 0.5f);
    // if (Vison_task_cmd != last_task_cmd) {
    //     Vison_task_changed = 1; // 任务命令变化
    // }

    Vison_revc_flag = 1; // 设置接收标志位
    // vision_used_flag = 1; // 设置视觉数据使用标志位
    //解析暂停标志位
    // run = (int)(Vison_data[11] + 0.5f);

    // 处理...
    // printf("Got frame: cmd=0x%04X flags=0x%04X\n", cmd_id, flags);
    // for (int i = 0; i < 12; i += 2) {
    //     printf("  coord %d: (%.3f, %.3f)\n",
    //            i/2, data[i], data[i+1]);
    // float a = data[0];
    
    // TODO: code to resolve flag_register;
}

void VisionOfflineCallback(void *owner_id)
{
    Vison_revc_flag = 0; // 清除接收标志位
}

void Vision_Uart_Init(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = 54;
    conf.usart_handle = _handle;
    cv_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 10,
    };
    OPENCV_vision_daemon_instance = DaemonRegister(&daemon_conf);
}