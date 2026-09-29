// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "controller.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "vofa.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "opencv.h"

#include "kalman_filter.h"

#include "arm_math.h"

// 添加卡尔曼滤波器实例
static KalmanFilter_t yaw_error_kf;
static KalmanFilter_t pitch_error_kf;
static float filtered_yaw_error = 0.0f;
static float filtered_pitch_error = 0.0f;

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

PIDInstance yaw_tracking_pid;           // 云台yaw跟随pid
PIDInstance pitch_tracking_pid;         // 云台pitch跟随pid
static float yaw_tracking_pid_output;   // 云台yaw跟随pid输出
static float pitch_tracking_pid_output; // 云台pitch跟随pid输出

static float vofa_debug[7];

// #define PITCH_HORIZON_ECD 6800 // 俯仰水平时的电机编码器

// 在RobotCMDInit()中添加卡尔曼滤波器初始化
static void VisionKalmanFilterInit()
{
    // yaw误差卡尔曼滤波器初始化
    Kalman_Filter_Init(&yaw_error_kf, 2, 0, 1); // 状态2维(位置,速度), 控制0维, 观测1维

    static float P_init_yaw[4] = {
        5.0f, 0.0f, // 减小初始不确定性
        0.0f, 5.0f};
    memcpy(yaw_error_kf.P_data, P_init_yaw, sizeof(P_init_yaw));

    // 状态转移矩阵F (dt = 1/200 = 0.005s, 200Hz控制频率)
    static float F_init_yaw[4] = {
        1.0f, 0.005f, // 修正时间步长
        0.0f, 1.0f};
    memcpy(yaw_error_kf.F_data, F_init_yaw, sizeof(F_init_yaw));

    // 过程噪声协方差矩阵Q
    static float Q_init_yaw[4] = {
        0.1f, 0.0f, // 增大位置过程噪声
        0.0f, 1.0f  // 增大速度过程噪声
    };
    memcpy(yaw_error_kf.Q_data, Q_init_yaw, sizeof(Q_init_yaw));

    // 观测矩阵H
    static uint8_t measurement_map_yaw[1] = {1}; // 观测第1个状态(位置)
    static float measurement_degree_yaw[1] = {1.0f};
    static float mat_R_diagonal_yaw[1] = {1.0f};               // 减小观测噪声，增加对观测的信任
    static float state_min_variance_yaw[2] = {0.001f, 0.001f}; // 减小最小方差

    yaw_error_kf.UseAutoAdjustment = 1;
    memcpy(yaw_error_kf.MeasurementMap, measurement_map_yaw, sizeof(measurement_map_yaw));
    memcpy(yaw_error_kf.MeasurementDegree, measurement_degree_yaw, sizeof(measurement_degree_yaw));
    memcpy(yaw_error_kf.MatR_DiagonalElements, mat_R_diagonal_yaw, sizeof(mat_R_diagonal_yaw));
    memcpy(yaw_error_kf.StateMinVariance, state_min_variance_yaw, sizeof(state_min_variance_yaw));

    // pitch误差卡尔曼滤波器初始化
    Kalman_Filter_Init(&pitch_error_kf, 2, 0, 1); // 状态2维(位置,速度), 控制0维, 观测1维
    static float P_init_pitch[4] = {
        5.0f, 0.0f, // 减小初始不确定性
        0.0f, 5.0f};
    memcpy(pitch_error_kf.P_data, P_init_pitch, sizeof(P_init_pitch));
    // 状态转移矩阵F (dt = 1/200 = 0.005s, 200Hz控制频率)
    static float F_init_pitch[4] = {
        1.0f, 0.005f, // 修正时间步长
        0.0f, 1.0f};
    memcpy(pitch_error_kf.F_data, F_init_pitch, sizeof(F_init_pitch));
    // 过程噪声协方差矩阵Q
    static float Q_init_pitch[4] = {
        0.1f, 0.0f, // 增大位置过程噪声
        0.0f, 1.0f  // 增大速度过程噪声
    };
    memcpy(pitch_error_kf.Q_data, Q_init_pitch, sizeof(Q_init_pitch));
    // 观测矩阵H
    static uint8_t measurement_map_pitch[1] = {1}; // 观测第1个状态(位置)
    static float measurement_degree_pitch[1] = {1.0f};
    static float mat_R_diagonal_pitch[1] = {1.0f};               // 减小观测噪声，增加对观测的信任
    static float state_min_variance_pitch[2] = {0.001f, 0.001f}; // 减小最小方差
    pitch_error_kf.UseAutoAdjustment = 1;
    memcpy(pitch_error_kf.MeasurementMap, measurement_map_pitch, sizeof(measurement_map_pitch));
    memcpy(pitch_error_kf.MeasurementDegree, measurement_degree_pitch, sizeof(measurement_degree_pitch));
    memcpy(pitch_error_kf.MatR_DiagonalElements, mat_R_diagonal_pitch, sizeof(mat_R_diagonal_pitch));
    memcpy(pitch_error_kf.StateMinVariance, state_min_variance_pitch, sizeof(state_min_variance_pitch));
}

void RobotCMDInit()
{
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    static PID_Init_Config_s yaw_tracking_pid_config = {
        .Kp = 1.5f,
        .Ki = 3.0f,
        .Kd = 0.07f,
        .IntegralLimit = 100.0f,
        .CoefB = 20.0f,
        .CoefA = 20.0f,
        // .Output_LPF_RC = 0.002f, // 100Hz
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
        .MaxOut = 200.0f,
    };
    static PID_Init_Config_s pitch_tracking_pid_config = {
        .Kp = 1.0f,
        .Ki = 2.0f,
        .Kd = 0.05f,
        .IntegralLimit = 100.0f,
        .CoefB = 10.0f,
        .CoefA = 30.0f,
        // .Output_LPF_RC = 0.002f, // 100Hz
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
        .MaxOut = 200.0f,
    };

    PIDInit(&yaw_tracking_pid, &yaw_tracking_pid_config);
    PIDInit(&pitch_tracking_pid, &pitch_tracking_pid_config);
    VisionKalmanFilterInit();
#ifdef VISION_USE_UART
    Vision_Uart_Init(&huart1);
#else
    Vision_Uart_Init(NULL);
#endif
    robot_state = ROBOT_READY;                // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

void laser_on()
{
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
}

void laser_off()
{
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_RESET);
}
static void AutoSetNoBoardComm()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    laser_off();

    if (Vison_revc_flag && Vison_Frequency >= 10.0f)
    {
        LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
        LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
        LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);

        yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
        pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];

        float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
        float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);

        filtered_yaw_error = yaw_filtered[0];
        filtered_pitch_error = pitch_filtered[0];

        yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
        pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);

        gimbal_cmd_send.yaw -= 0.001f * yaw_tracking_pid_output;
        gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;

        vofa_debug[0] = filtered_yaw_error;
        vofa_debug[1] = filtered_pitch_error;
        vofa_debug[2] = yaw_tracking_pid.Err;
        vofa_debug[3] = yaw_tracking_pid.Pout;
        vofa_debug[4] = yaw_tracking_pid.Iout;
        vofa_debug[5] = yaw_tracking_pid.Dout;
        vofa_debug[6] = yaw_tracking_pid.Output;
        vofa_justfloat_output(vofa_debug, 7, &huart1);
    }

    LIMIT_MIN_MAX(gimbal_cmd_send.pitch, -30.0f, 30.0f);
}

/**
 * @brief Demo 单板模式控制入口
 *
 */
static void AutoSet()
{
    AutoSetNoBoardComm();
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 无遥控模式下仅保留软件急停状态锁定
    if (robot_state == ROBOT_STOP)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        LOGERROR("[CMD] emergency stop!");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    AutoSet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
