/****************************************************************
 * @file: 	Chassis_Task.h
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵舵轮底盘任务，存放底盘的各种控制参数的宏定义
 * @attention:
 ******************************************************************/
#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

//底盘运动学解算相关参数
#define MOTOR_DISTANCE_TO_CENTER 0.237f // 舵轮与地面的接触点到车体中心的距离
#define WHEEL_RADIUS 0.06f  //舵轮半径
#define MOTOR_REDUCTION_RATIO 13.72f  // 轮电机减速比
#define CHASSIS_FOLLOW_GIMBAL_BACK_ZERO 268.41f  //底盘跟随云台时的后零点，单位：度
#define CHASSIS_FOLLOW_GIMBAL_RIGHT_ZERO 358.41f // 底盘跟随云台时的右零点
#define CHASSIS_FOLLOW_GIMBAL_LEFT_ZERO 178.41f  // 底盘跟随云台时的左零点
#define CHASSIS_FOLLOW_GIMBAL_ZERO 88.41f       // 底盘跟随云台时的零点，同时也是小陀螺模式下底盘vx(前进)正方向

#define STEER_MOROR1_ENC_OFFSET 6906 //1号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR2_ENC_OFFSET 1467 //2号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR3_ENC_OFFSET 6232 //3号舵电机的编码器偏置（范围0-8191）
#define STEER_MOROR4_ENC_OFFSET 503 //4号舵电机的编码器偏置（范围0-8191）

//小陀螺相关参数
#define ROTATE_WZ_MAX 10.0  // 遥控模式下小陀螺正向速度，单位：rad/s
#define ROTATE_WZ_MIN -5.0 // 遥控模式下小陀螺反向速度
#define ROTATE_SAVE_ENERGY 0.3f     // 哨兵未被弹丸击打时将目标小陀螺转速乘以此系数，达到低速小陀螺省功率的效果
#define ROTATE_MOVE_FF_HIGH_SPEED 0.04f // 高速小陀螺模式下的前馈系数
#define ROTATE_MOVE_FF_LOW_SPEED 0.02f  //低速率小陀螺模式下的前馈系数

//功率控制相关参数
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
#define BUFFER_TOTAL_CURRENT_LIMIT 30000.0f
#define POWER_TOTAL_CURRENT_LIMIT 30000.0f
#define WARNING_POWER_BUFF 60.0f

//底盘控制参数（pid,前馈）
#define WHEEL_MOTOR_SPEED_PID_KP 8.5f
#define WHEEL_MOTOR_SPEED_PID_KI 0.002f
#define WHEEL_MOTOR_SPEED_PID_KD 0.0f
#define WHEEL_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define WHEEL_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define STEER_MOTOR_SPEED_PID_KP 160.0f
#define STEER_MOTOR_SPEED_PID_KI 0.02f 
#define STEER_MOTOR_SPEED_PID_KD 0.0f
#define STEER_MOTOR_SPEED_PID_MAX_OUT 20000.0f
#define STEER_MOTOR_SPEED_PID_MAX_IOUT 1000.0f
#define STEER_MOTOR_SPEED_FF 1.4f // 舵轮电机速度前馈系数

#define STEER_MOTOR_ANGLE_PID_KP 5.0f
#define STEER_MOTOR_ANGLE_PID_KI 0.0002f
#define STEER_MOTOR_ANGLE_PID_KD 1.5f
#define STEER_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define STEER_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 70.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0005f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 12.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6500.0f //20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 450.0f

//自主导航时的参数
#define NAV_SPEED_FAST 800.0f // 导航发过来的速度乘以的系数，非上坡用

//量纲转换参数
#define DM6006_ENC_TO_DEGREE 0.021972f  //  360/16384
#define GM6020_ENC_TO_DEGREE 0.043945f  //  360/8192
#define DEGREE_TO_RAD 0.0172532f  //  pi/180
#define RAD_TO_DEGREE 57.295779f
#define PI 3.141592f

void Chassis_Task(void const *argument);

#endif
