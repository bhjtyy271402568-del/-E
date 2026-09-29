#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "vofa.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static DJIMotorInstance *yaw_motor;  // 云台yaw电机实例
static DMMotorInstance *pitch_motor; // 云台pitch电机实例

static float yaw_speed_feedforward = 0.0f; // yaw电机的速度前馈
static float yaw_current_feedforward_abs = 500.0f;
static float yaw_current_feedforward = 0.0f; // yaw电机的电流前馈

static float yaw_last_ref = 0.0f; // 上一次yaw电机的设定值,用于计算前馈

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()//云台初始化
{
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);//
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_RESET);
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    // 初始化yaw电机
    Motor_Init_Config_s yaw_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .can_init_config.tx_id = 0x01,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.3,
                .Ki = 0.8,
                .Kd = 0.005,
                .IntegralLimit = 20,
                .CoefA = 15.0f,
                .CoefB = 1.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 1200,
                .Ki = 0,
                .Kd = 3,
                .IntegralLimit = 16000,
                .Output_LPF_RC = 0.001,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut = 16000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &yaw_speed_feedforward, // 速度前馈指针

        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,              // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, // 角度环和速度环
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD, // 开启速度前馈
        },
        .motor_type = GM6020_NEW,
    };
    yaw_motor = DJIMotorInit(&yaw_motor_config);
    // 初始化PITCH电机
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x06,
            .rx_id = 0x16,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0.005,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 10,
                .MaxOut = 10,
            },
            .speed_PID = {
                .Kp = 0.8,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.1,
                .MaxOut = 1,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    pitch_motor = DMMotorInit(&pitch_config);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);

        yaw_speed_feedforward = 0.0f; // 停止时清零前馈
        yaw_last_ref = 0.0f;          // 停止时清零上一次参考值
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);

        yaw_speed_feedforward = DEGREE_2_RAD * (gimbal_cmd_recv.yaw - yaw_last_ref) * 500.0f;
        LIMIT_MIN_MAX(yaw_speed_feedforward, -10.0f, 10.0f); // 限制前馈值
        yaw_last_ref = gimbal_cmd_recv.yaw;                  // 更新上一次的参考值
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);      // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        if (yaw_motor->motor_controller.speed_PID.Ref >= 0.5f)
        {
            yaw_current_feedforward = 500.0f; // 前馈电流
        }
        else if (yaw_motor->motor_controller.speed_PID.Ref <= -0.5f)
        {
            yaw_current_feedforward = -500.0f; // 前馈电流
        }
        else
        {
            yaw_current_feedforward = yaw_current_feedforward_abs * yaw_motor->motor_controller.speed_PID.Ref / 0.5f; // 前馈电流
        }

        break;
    default:
        break;
    }

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}