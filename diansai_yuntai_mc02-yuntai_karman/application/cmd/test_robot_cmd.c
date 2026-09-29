#include "kalman_filter.h"

// 添加卡尔曼滤波器实例
static KalmanFilter_t yaw_error_kf;
static KalmanFilter_t pitch_error_kf;
static float filtered_yaw_error = 0.0f;
static float filtered_pitch_error = 0.0f;

// 在RobotCMDInit()中添加卡尔曼滤波器初始化
static void VisionKalmanFilterInit()
{
    // yaw误差卡尔曼滤波器初始化
    Kalman_Filter_Init(&yaw_error_kf, 2, 0, 1); // 状态2维(位置,速度), 控制0维, 观测1维
    
    // 减小初始协方差矩阵P，增加收敛速度
    static float P_init_yaw[4] = {
        5.0f, 0.0f,    // 减小初始不确定性
        0.0f, 5.0f
    };
    memcpy(yaw_error_kf.P_data, P_init_yaw, sizeof(P_init_yaw));
    
    // 状态转移矩阵F (dt = 1/200 = 0.005s, 200Hz控制频率)
    static float F_init_yaw[4] = {
        1.0f, 0.005f,  // 修正时间步长
        0.0f, 1.0f
    };
    memcpy(yaw_error_kf.F_data, F_init_yaw, sizeof(F_init_yaw));
    
    // 增大过程噪声协方差矩阵Q，允许更快的状态变化
    static float Q_init_yaw[4] = {
        0.1f, 0.0f,    // 增大位置过程噪声
        0.0f, 1.0f     // 增大速度过程噪声
    };
    memcpy(yaw_error_kf.Q_data, Q_init_yaw, sizeof(Q_init_yaw));
    
    // 观测矩阵H
    static uint8_t measurement_map_yaw[1] = {1}; // 观测第1个状态(位置)
    static float measurement_degree_yaw[1] = {1.0f};
    static float mat_R_diagonal_yaw[1] = {1.0f}; // 减小观测噪声，增加对观测的信任
    static float state_min_variance_yaw[2] = {0.001f, 0.001f}; // 减小最小方差
    
    yaw_error_kf.UseAutoAdjustment = 1;
    memcpy(yaw_error_kf.MeasurementMap, measurement_map_yaw, sizeof(measurement_map_yaw));
    memcpy(yaw_error_kf.MeasurementDegree, measurement_degree_yaw, sizeof(measurement_degree_yaw));
    memcpy(yaw_error_kf.MatR_DiagonalElements, mat_R_diagonal_yaw, sizeof(mat_R_diagonal_yaw));
    memcpy(yaw_error_kf.StateMinVariance, state_min_variance_yaw, sizeof(state_min_variance_yaw));
    
    // pitch误差卡尔曼滤波器初始化 - 使用相同的优化参数
    Kalman_Filter_Init(&pitch_error_kf, 2, 0, 1);
    
    static float P_init_pitch[4] = {
        5.0f, 0.0f,
        0.0f, 5.0f
    };
    memcpy(pitch_error_kf.P_data, P_init_pitch, sizeof(P_init_pitch));
    
    static float F_init_pitch[4] = {
        1.0f, 0.005f,
        0.0f, 1.0f
    };
    memcpy(pitch_error_kf.F_data, F_init_pitch, sizeof(F_init_pitch));
    
    static float Q_init_pitch[4] = {
        0.1f, 0.0f,
        0.0f, 1.0f
    };
    memcpy(pitch_error_kf.Q_data, Q_init_pitch, sizeof(Q_init_pitch));
    
    static uint8_t measurement_map_pitch[1] = {1};
    static float measurement_degree_pitch[1] = {1.0f};
    static float mat_R_diagonal_pitch[1] = {1.0f};
    static float state_min_variance_pitch[2] = {0.001f, 0.001f};
    
    pitch_error_kf.UseAutoAdjustment = 1;
    memcpy(pitch_error_kf.MeasurementMap, measurement_map_pitch, sizeof(measurement_map_pitch));
    memcpy(pitch_error_kf.MeasurementDegree, measurement_degree_pitch, sizeof(measurement_degree_pitch));
    memcpy(pitch_error_kf.MatR_DiagonalElements, mat_R_diagonal_pitch, sizeof(mat_R_diagonal_pitch));
    memcpy(pitch_error_kf.StateMinVariance, state_min_variance_pitch, sizeof(state_min_variance_pitch));
}

// 更新RemoteControlSet()函数中的视觉处理部分
static void RemoteControlSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        if (Vison_revc_flag)
        {
            // 限制视觉数据范围
            LIMIT_MIN_MAX(opencv_Vision_data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(opencv_Vision_data[1], -100.0f, 100.0f);
            
            // 更新卡尔曼滤波器观测值
            yaw_error_kf.MeasuredVector[0] = opencv_Vision_data[0];
            pitch_error_kf.MeasuredVector[0] = opencv_Vision_data[1];
        }
        else
        {
            // 无视觉数据时，观测值设为0(表示无效)
            yaw_error_kf.MeasuredVector[0] = 0.0f;
            pitch_error_kf.MeasuredVector[0] = 0.0f;
        }
        
        // 执行卡尔曼滤波更新
        float *yaw_filtered = Kalman_Filter_Update(&yaw_error_kf);
        float *pitch_filtered = Kalman_Filter_Update(&pitch_error_kf);
        
        filtered_yaw_error = yaw_filtered[0];
        filtered_pitch_error = pitch_filtered[0];
        
        // 使用滤波后的误差进行PID控制
        yaw_tracking_pid_output = PIDCalculate(&yaw_tracking_pid, filtered_yaw_error, 0);
        pitch_tracking_pid_output = PIDCalculate(&pitch_tracking_pid, filtered_pitch_error, 0);
        
        gimbal_cmd_send.yaw += 0.001f * yaw_tracking_pid_output;
        gimbal_cmd_send.pitch -= 0.001f * pitch_tracking_pid_output;
        
        // 调试输出
        vofa_debug[0] = filtered_yaw_error;
        vofa_debug[1] = filtered_pitch_error;
        vofa_debug[2] = yaw_tracking_pid.Err;
        vofa_debug[3] = yaw_tracking_pid.dt;
        vofa_debug[4] = yaw_tracking_pid.Iout;
        vofa_debug[5] = yaw_tracking_pid.Dout;
        vofa_debug[6] = yaw_tracking_pid.Output;
        vofa_justfloat_output(vofa_debug, 7, &huart1);
    }
    else
    {
        gimbal_cmd_send.yaw += 0.0025f * (float)rc_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch -= 0.00125f * (float)rc_data[TEMP].rc.rocker_r1;
    }

    if (gimbal_cmd_send.pitch > 30.0f)
        gimbal_cmd_send.pitch = 30.0f;
    else if (gimbal_cmd_send.pitch < -30.0f)
        gimbal_cmd_send.pitch = -30.0f;
}