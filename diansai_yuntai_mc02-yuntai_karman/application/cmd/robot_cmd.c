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
#include "board_comm.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "opencv.h"
#include "step_motor.h"

#include "kalman_filter.h"

#include "arm_math.h"

#define mapping(x, in_min, in_max, out_min, out_max) \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

// 添加卡尔曼滤波器实例
static KalmanFilter_t yaw_error_kf;
static KalmanFilter_t pitch_error_kf;
static float filtered_yaw_error = 0.0f;
static float filtered_pitch_error = 0.0f;

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define yaw_control_kxmax 0.001f
#define yaw_control_kxmin 0.0005f
#define pitch_control_kxmax 0.001f
#define pitch_control_kxmin 0.0004f

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

static Vision_Recv_s *vision_recv_data;    // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;     // 视觉发送数据
static board_comm_data_t *board_comm_data; // 板间通信数据指针,初始化时返回
static board_comm_data_t board_comm_data_stub;

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

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
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
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
    Vision_Uart_Init(&huart10);
    board_comm_data = &board_comm_data_stub;
    robot_state = ROBOT_READY;                // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

static uint8_t first_circle_flag = 1;
static float circle_progress = 0;   // 画圆进度 100%为一圈
static float circle_radius = 60.0f; // 画圆半径
static float circle_angle = 0.0f;   // 当前圆周角度
static float target_x = 0.0f;       // 圆周上目标点x坐标
static float target_y = 0.0f;       // 圆周上目标点y坐标

void laser_on()
{
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
}

void laser_off()
{
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_RESET);
}
enum YunTaiState_e
{
    YUNT_IDLE = 0, // 云台无任务
    YUNT_NO1,      // 云台题号1准备
    YUNT_NO2_WAITING,
    YUNT_NO2,
    YUNT_NO3_WAITING,
    YUNT_NO3_TURING,
    YUNT_NO3_SEARCHING,
    YUNT_NO3_TRACKING,
    YUNT_NO4_1_WAITING,
    YUNT_NO4_1_TURING,
    YUNT_NO4_1_TRACKING,
    YUNT_NO4_1_TRACKING_RUNNING,
    YUNT_NO4_3_WAITING,
    YUNT_NO4_3_TURING,
    YUNT_NO4_3_TRACKING,
    YUNT_NO4_3_TRACKING_RUNNING,
} YunTaiState = YUNT_IDLE; // 云台状态机

static float yuntai_no3_yawtarget = 0.0f;           // 云台题号3目标yaw角度
static float yuntai_no3_searching_yaw = 0.0f;       // 云台题号3搜索yaw角度
static float yuntai_no3_searching_yaw_max = 60.0f;  // 云台yaw题号3搜索最大yaw角度
static float yuntai_no3_searching_yaw_min = -60.0f; // 云台yaw题号3搜索最小yaw角度
static float yuntai_no3_searching_step = 0.2f;      // 云台题号3搜索步长,每次0.18度
static uint8_t yuntai_no3_searching_dir = 0;        // 云台题号3搜索方向, 0: -, 1:+
static int16_t yuntai_no3_searching_count = 50;     // 3计数器

static uint8_t no4_turning_count = 0;  // 题号4转向计数器
static uint8_t no4_turning_status = 0; // 题号4转向状态
static uint8_t no4_turning_feedforward = 0.05;

static float timestart_timeline = 0.0f; // 题号开始时间

/**
 * @brief 题号状态机
 *
 */
static void AutoSet()
{

    yaw_tracking_pid.Ki = 3.0f;

    switch (board_comm_data->rx_int_data[0]) // 根据板间通信数据的第一个整数值来判断题号
    {
    case 1:
        YunTaiState = YUNT_NO1;
        break;
    case 2:
        if (board_comm_data->rx_int_data[1] == 0)
        {
            YunTaiState = YUNT_NO2_WAITING;
        }
        else
        {
            YunTaiState = YUNT_NO2; // 进入题号2状态
        }
        break;
    case 3:
        if (board_comm_data->rx_int_data[1] == 0)
        {
            YunTaiState = YUNT_NO3_WAITING;
        }
        else if (YunTaiState == YUNT_NO3_WAITING)
        {
            YunTaiState = YUNT_NO3_TURING; // 进入题号3转向状态
        }
        break;
    case 4:
    case 5:
        if (board_comm_data->rx_int_data[1] == 0)
        {
            YunTaiState = YUNT_NO4_1_WAITING; // 进入题号4-1等待状态
        }
        else if (board_comm_data->rx_int_data[1] == 1 && YunTaiState == YUNT_NO4_1_WAITING)
        {
            YunTaiState = YUNT_NO4_1_TURING; // 进入题号4-1转向状态
        }
        else if (board_comm_data->rx_int_data[1] == 2)
        {
            YunTaiState = YUNT_NO4_1_TRACKING_RUNNING;
        }
        break;
    case 6:
        if (board_comm_data->rx_int_data[1] == 0)
        {
            YunTaiState = YUNT_NO4_3_WAITING; // 进入题号4-3等待状态
        }
        else if (board_comm_data->rx_int_data[1] == 1 && YunTaiState == YUNT_NO4_3_WAITING)
        {
            YunTaiState = YUNT_NO4_3_TURING; // 进入题号4-3状态
        }
        else if (board_comm_data->rx_int_data[1] == 2)
        {
            YunTaiState = YUNT_NO4_3_TRACKING_RUNNING;
        }
    default:
        break;
    }

    switch (YunTaiState)
    {
    case YUNT_NO1:
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        laser_off();
        break;
    case YUNT_NO2_WAITING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        timestart_timeline = DWT_GetTimeline_ms(); // 记录开始时间
        laser_off();
        break;
    case YUNT_NO2:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (DWT_GetTimeline_ms() - timestart_timeline >= 1500.0f)
        {
            laser_on();
        }
        else
        {
            if (Vison_revc_flag)
            {
                LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
                LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
                LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
                yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
                float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
                filtered_yaw_error = yaw_filtered[0];
                pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
                float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
                filtered_pitch_error = pitch_filtered[0];
                yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
                pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
                gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;
            }
        }

        break;
    case YUNT_NO3_WAITING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        // 计算0度对应最接近的多圈yaw
        yuntai_no3_yawtarget = roundf(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f) * 360.0f; // 计算最近的多圈yaw
        laser_off();
        timestart_timeline = DWT_GetTimeline_ms(); // 记录开始时间
        break;
    case YUNT_NO3_TURING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        float yaw_diff = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - yuntai_no3_yawtarget; // 计算当前yaw与目标yaw的差值
        float step_size = 0.8f;                                                                  // 90度每秒, 500hz控制频率下每次0.18度
        if (fabsf(yaw_diff) > step_size)                                                         // 如果差值大于180度,则调整到最小角度
        {
            if (yaw_diff > 0)
                gimbal_cmd_send.yaw -= step_size;
            else
                gimbal_cmd_send.yaw += step_size;
        }
        if (fabsf(yaw_diff) <= 5.0f) // 如果差值小于5度,则认为转向完成
        {
            if (Vison_revc_flag)
            {
                YunTaiState = YUNT_NO3_TRACKING; // 进入跟踪状态
            }
            else
            {
                YunTaiState = YUNT_NO3_SEARCHING; // 进入搜索状态
            }
        }
        gimbal_cmd_send.pitch = 0;
        break;
    case YUNT_NO3_TRACKING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (DWT_GetTimeline_ms() - timestart_timeline >= 3500.0f) // 如果超过3.5秒,则认为跟踪完成
        {
            laser_on();
        }
        else
        {
            if (Vison_revc_flag && Vison_Frequency >= 20.0f) // 如果视觉数据可用且频率大于20Hz
            {
                LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
                LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
                LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
                yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
                float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
                filtered_yaw_error = yaw_filtered[0];
                pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
                float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
                filtered_pitch_error = pitch_filtered[0];

                yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
                pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
                gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;
            }
            else
            {
                YunTaiState = YUNT_NO3_SEARCHING; // 如果视觉数据不可用,则进入搜索状态
            }
        }

        break;
    case YUNT_NO3_SEARCHING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (yuntai_no3_searching_dir == 0) // 如果当前是向左搜索
        {
            gimbal_cmd_send.yaw += yuntai_no3_searching_step;                              // 向左搜索
            if (gimbal_cmd_send.yaw > yuntai_no3_searching_yaw_max + yuntai_no3_yawtarget) // 如果超过最小值,则切换方向
            {
                yuntai_no3_searching_dir = 1; // 切换到向右搜索
            }
        }
        else // 当前是向右搜索
        {
            gimbal_cmd_send.yaw -= yuntai_no3_searching_step;                              // 向右搜索
            if (gimbal_cmd_send.yaw < yuntai_no3_searching_yaw_min + yuntai_no3_yawtarget) // 如果超过最大值,则切换方向
            {
                yuntai_no3_searching_dir = 0; // 切换到向左搜索
            }
        }

        gimbal_cmd_send.pitch = 0.0f; // 保持pitch角度为0
        if (Vison_revc_flag && Vison_Frequency >= 20.0f)
        {
            yuntai_no3_searching_count--;
        }
        else
        {
            yuntai_no3_searching_count = 100; // 如果视觉数据可用但频率低于20Hz,则重置计数器
        }
        if (yuntai_no3_searching_count <= 0)
        {
            YunTaiState = YUNT_NO3_TRACKING; // 如果视觉数据可用,则进入跟踪状态
        }
        break;
    case YUNT_NO4_1_WAITING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        // 计算0度对应最接近的多圈yaw
        yuntai_no3_yawtarget = roundf(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f) * 360.0f; // 计算最近的多圈yaw
        no4_turning_count = 0;                                                                            // 题号4转向计数器
        no4_turning_status = 0;                                                                           // 题号4转向状态
        timestart_timeline = DWT_GetTimeline_ms();                                                        // 记录开始时间
        laser_off();
        break;
    case YUNT_NO4_1_TURING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw = yuntai_no3_yawtarget - 45.0f;
        gimbal_cmd_send.pitch = 0.0f; // 保持pitch角度为0
        if (fabsf(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - yuntai_no3_yawtarget + 45.0f) < 5.0f)
        {
            YunTaiState = YUNT_NO4_1_TRACKING; // 进入跟踪状态
        }
        break;
    case YUNT_NO4_1_TRACKING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (Vison_revc_flag)
        {
            LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
            yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
            float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
            filtered_yaw_error = yaw_filtered[0];
            pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
            float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
            filtered_pitch_error = pitch_filtered[0];

            yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
            pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
            gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;
        }
        break;
    case YUNT_NO4_1_TRACKING_RUNNING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (board_comm_data->rx_float_data[1] >= 600.0f)
        {
            no4_turning_status = 1; // 题号4转向状态
            if (board_comm_data->rx_float_data[0] >= 0.0f && board_comm_data->rx_float_data[0] <= 12.5f)
            {
                no4_turning_count = 1;
            }
            else if (board_comm_data->rx_float_data[0] > 12.5f && board_comm_data->rx_float_data[0] <= 37.5f)
            {
                no4_turning_count = 2;
            }
            else if (board_comm_data->rx_float_data[0] > 37.5f && board_comm_data->rx_float_data[0] <= 62.5f)
            {
                no4_turning_count = 3;
            }
            else if (board_comm_data->rx_float_data[0] > 62.5f && board_comm_data->rx_float_data[0] <= 87.5f)
            {
                no4_turning_count = 4;
            }
        }
        else
        {
            no4_turning_status = 0; // 题号4转向状态
        }
        if (Vison_revc_flag)
        {
            LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
            yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
            float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
            filtered_yaw_error = yaw_filtered[0];
            pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
            float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
            filtered_pitch_error = pitch_filtered[0];
            yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
            pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
            gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;
            if (no4_turning_status == 1 && (no4_turning_count >=1 && no4_turning_count <=4)) // 如果题号4转向状态为1且转向计数器为1到4
            {
                yaw_tracking_pid.Kp = 3.0f;                                                                                                             // 关闭yaw比例
                yaw_tracking_pid.Ki = 0.0f;                                                                                                             // 关闭yaw积分
                yaw_tracking_pid.Iout = 0.0f;                                                                                                           // 清零yaw积分输出
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            }
            else
            {
                yaw_tracking_pid.Kp = 2.0f; // 关闭yaw比例
                yaw_tracking_pid.Ki = 4.0f;
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            }
            laser_on();
        }
        break;
    case YUNT_NO4_3_WAITING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
        no4_turning_count = 0;  // 题号4转向计数器
        no4_turning_status = 0; // 题号4转向状态

        // 计算0度对应最接近的多圈yaw
        yuntai_no3_yawtarget = roundf(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f) * 360.0f; // 计算最近的多圈yaw
        timestart_timeline = DWT_GetTimeline_ms();                                                        // 记录开始时间
        laser_off();
        break;
    case YUNT_NO4_3_TURING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_cmd_send.yaw = yuntai_no3_yawtarget - 45.0f;
        gimbal_cmd_send.pitch = 0.0f; // 保持pitch角度为0
        if (fabsf(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - yuntai_no3_yawtarget + 45.0f) < 5.0f)
        {
            YunTaiState = YUNT_NO4_3_TRACKING; // 进入跟踪状态
        }
        break;
    case YUNT_NO4_3_TRACKING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (Vison_revc_flag)
        {
            LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
            yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
            float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
            filtered_yaw_error = yaw_filtered[0];
            pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
            float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
            filtered_pitch_error = pitch_filtered[0];

            yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 60.0f); // 题号4-3目标error为60
            pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
            gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            gimbal_cmd_send.pitch += 0.001f * pitch_tracking_pid_output;
        }
        break;
    case YUNT_NO4_3_TRACKING_RUNNING:
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        if (board_comm_data->rx_float_data[1] >= 600.0f)
        {
            no4_turning_status = 1; // 题号4转向状态
            if (board_comm_data->rx_float_data[0] >= 0.0f && board_comm_data->rx_float_data[0] <= 12.5f)
            {
                no4_turning_count = 1;
            }
            else if (board_comm_data->rx_float_data[0] > 12.5f && board_comm_data->rx_float_data[0] <= 37.5f)
            {
                no4_turning_count = 2;
            }
            else if (board_comm_data->rx_float_data[0] > 37.5f && board_comm_data->rx_float_data[0] <= 62.5f)
            {
                no4_turning_count = 3;
            }
            else if (board_comm_data->rx_float_data[0] > 62.5f && board_comm_data->rx_float_data[0] <= 87.5f)
            {
                no4_turning_count = 4;
            }
        }
        else
        {
            no4_turning_status = 0; // 题号4转向状态
        }
        if (Vison_revc_flag)
        {
            LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[2], 500.0f, 1560.0f);
            yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
            float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
            filtered_yaw_error = yaw_filtered[0];
            pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
            float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
            filtered_pitch_error = pitch_filtered[0];
            // 画圆模式 500hz 16s画完整圆
            circle_progress = board_comm_data->rx_float_data[0] / 100.0f; // 画圆进度,从板间通信数据获取
            if (circle_progress >= 1.0f)
            {
                circle_progress = 1.0f; // 限制在100%
            }
            circle_angle = 2.0f * PI * circle_progress;    // 计算当前圆周角度
            target_x = circle_radius * cosf(circle_angle); // 计算圆周上目标点x坐标
            target_y = circle_radius * sinf(circle_angle); // 计算圆周上目标点y坐标
            yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, target_x);
            pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, target_y);
            gimbal_cmd_send.pitch += pitch_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, pitch_control_kxmax, pitch_control_kxmin); // pitch角度控制
            if (no4_turning_status == 1 && (no4_turning_count == 2 || no4_turning_count == 3))                                                              // 如果题号4转向状态为1且转向计数器为2或3
            {
                yaw_tracking_pid.Kp = 3.0f;                                                                                                             // 关闭yaw比例
                yaw_tracking_pid.Ki = 0.0f;                                                                                                             // 关闭yaw积分
                yaw_tracking_pid.Iout = 0.0f;                                                                                                           // 清零yaw积分输出
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            }
            else
            {
                yaw_tracking_pid.Kp = 1.5f; // 关闭yaw比例
                yaw_tracking_pid.Ki = 3.0f;
                gimbal_cmd_send.yaw -= yaw_tracking_pid_output * mapping(opencv_Vision_data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin); // yaw角度控制
            }
        }
        laser_on();
        break;

    default:
        break;
    }
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
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    AutoSet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
