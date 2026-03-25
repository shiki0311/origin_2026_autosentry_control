/****************************************************************
 * @file: 	Chassis_Task.h
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵舵轮底盘任务，存放底盘的各种控制参数的宏定义
 * @attention:
 ******************************************************************/
#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "struct_typedef.h"
#include "pid.h"

// 量纲转换参数
#define PI 3.141592f
#define M_PER_SEC_TO_RPM 2183.6f // m/s转rpm(注意，与舵轮半径和轮电机减速比有关)公式为 60*MOTOR_REDUCTION_RATIO/（2*pi*WHEEL_RADIUS)
#define DEGREE_TO_RAD 0.0172532f //  pi/180
#define RAD_TO_DEGREE 57.295779f
#define DM6006_ENC_TO_DEGREE 0.021972f //  360/16384
#define GM6020_ENC_TO_DEGREE 0.043945f //  360/8192

// 底盘运动学解算相关参数
#define MOTOR_DISTANCE_TO_CENTER 0.2899f // 舵轮与地面的接触点到车体中心的距离
#define MOTOR_DISTANCE_WIDTH 0.41f // 底盘较短的轮距
#define MOTOR_DISTANCE_LENGTH 0.41f // 底盘较长的轮距
#define WHEEL_RADIUS 0.06f  //舵轮半径
#define MOTOR_REDUCTION_RATIO 13.72f  // 轮电机减速比
#define CHASSIS_FOLLOW_GIMBAL_BACK_ZERO 161.63f  //底盘跟随云台时的后零点，单位：度
#define CHASSIS_FOLLOW_GIMBAL_RIGHT_ZERO 251.63f // 底盘跟随云台时的右零点
#define CHASSIS_FOLLOW_GIMBAL_LEFT_ZERO 71.63f  // 底盘跟随云台时的左零点
#define CHASSIS_FOLLOW_GIMBAL_ZERO 341.63f       // 底盘跟随云台时的零点，同时也是小陀螺模式下底盘vx(前进)正方向

#define STEER_MOROR1_ENC_OFFSET 4705 //1号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR2_ENC_OFFSET 7706 //2号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR3_ENC_OFFSET 807 //3号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR4_ENC_OFFSET 6112 //4号舵电机的编码器偏置（范围0-8191）

//小陀螺相关参数
#define ROTATE_WZ_MAX 12.0  // 小陀螺正向速度,高速，单位：rad/s
#define ROTATE_WZ_HIGH 7.0f // 小陀螺正向速度,中高速，单位：rad/s
#define ROTATE_WZ_MEDIUM -5.0 // 小陀螺反向速度，中速
#define ROTATE_WZ_LOW 3.0f  // 小陀螺正向速度,低速，单位：rad/s
#define ROTATE_WZ_MIN 0.0 // 小陀螺模式，但不转，在导航模式下省功率
#define ROTATE_SAVE_ENERGY 0.3f     // 哨兵未被弹丸击打时将目标小陀螺转速乘以此系数，达到低速小陀螺省功率的效果
#define ROTATE_MOVE_FF_HIGH_SPEED 0.04f // 高速小陀螺模式下的前馈系数
#define ROTATE_MOVE_FF_LOW_SPEED 0.02f  //低速率小陀螺模式下的前馈系数
#define NAV_MAX_SPEED (3.5 * M_PER_SEC_TO_RPM)   // 导航模式下底盘最大速度，单位m/s * (m/s转rpm)

//底盘控制参数（pid,前馈）
#define WHEEL_MOTOR_SPEED_PID_KP 10.5f
#define WHEEL_MOTOR_SPEED_PID_KI 0.002f
#define WHEEL_MOTOR_SPEED_PID_KD 0.0f
#define WHEEL_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define WHEEL_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define STEER_MOTOR_SPEED_PID_KP 80.0f
#define STEER_MOTOR_SPEED_PID_KI 0.02f 
#define STEER_MOTOR_SPEED_PID_KD 0.0f
#define STEER_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define STEER_MOTOR_SPEED_PID_MAX_IOUT 1000.0f
#define STEER_MOTOR_SPEED_FF 3.4f // 舵轮电机速度前馈系数
#define STEER_MOTOR_CURRENT_FF 20.0f

#define STEER_MOTOR_ANGLE_PID_KP 3.0f
#define STEER_MOTOR_ANGLE_PID_KI 0.0002f
#define STEER_MOTOR_ANGLE_PID_KD 0.7f
#define STEER_MOTOR_ANGLE_PID_MAX_OUT 1500.0f
#define STEER_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 100.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0005f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 70.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6500.0f //20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 450.0f

#define BUFFER_ENERGY_PID_KP 2.5f
#define BUFFER_ENERGY_PID_KI 0.02f
#define BUFFER_ENERGY_PID_KD 0.0f
#define BUFFER_ENERGY_PID_MAX_OUT 30.0f
#define BUFFER_ENERGY_PID_MAX_IOUT 30.0f

#define STEER_MOTOR_AGGRESIVE_ANGLE_PID_KP 12.0f //过颠簸特化
#define STEER_MOTOR_AGGRESIVE_ANGLE_PID_KI 0.0002f
#define STEER_MOTOR_AGGRESIVE_ANGLE_PID_KD 1.5f

typedef struct // 上板传下来的导航相关数据
{
    fp32 vx;
    fp32 vy;
    uint8_t chassis_target_mode;
    uint8_t updownhill_state;
    uint8_t health_state;
    uint8_t buffer_energy_remain;
    uint8_t referee_power_limit;
    uint8_t total_ernergy_remain;
    uint8_t at_middle_section;
    uint8_t rmul_first_go_to_middle;
    uint8_t game_start;
} nav_ctrl_t;

extern nav_ctrl_t nav_ctrl;

void Chassis_Task(void const *argument);

#endif
