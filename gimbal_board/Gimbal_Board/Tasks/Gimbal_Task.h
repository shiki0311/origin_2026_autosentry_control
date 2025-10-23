/****************************************************************
 * @file: 	Gimbal_Task.h
 * @author: Shiki
 * @date:	2025.10.5
 * @brief:	2026赛季哨兵云台任务,此文件存放云台控制参数的宏定义
 * @attention:
 ******************************************************************/
#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#define DEGREE_TO_RAD 0.0172532f //  pi/180
#define RAD_TO_DEGREE 57.295779f

#define PITCH_ECD_ANGLE_MAX 27280 // 27280
#define PITCH_ECD_ANGLE_MIN 24600 // 24600

/****************************************重力补偿参数和速度环前馈系数*******************************************************/
#define YAW_MOTOR_FF 4.0f
#define PITCH_MOTOR_FF 1.8f
#define PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE (1.3f)  // 用于补偿重力，pitch轴与地面平行时抵消重力所需的力矩
#define PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE (1.5f) // 用于补偿重力，pitch轴与地面不平行时抵消重力所需的偏置力矩系数
/**************************************************************************************************************************/
#define BIG_YAW_MOTOR_SPEED_PID_KP 0.008f
#define BIG_YAW_MOTOR_SPEED_PID_KI 0.00002f // 80.0f
#define BIG_YAW_MOTOR_SPEED_PID_KD 0.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_OUT 12.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT 0.4f

#define BIG_YAW_MOTOR_ANGLE_PID_KP 12.5f
#define BIG_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define BIG_YAW_MOTOR_ANGLE_PID_KD 10.0f
#define BIG_YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define BIG_YAW_MOTOR_ANGLE_PID_MAX_IOUT 50.0f

#define BIG_YAW_MOTOR_AUTO_AIM_PID_KP 25.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_KD 50.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define SMALL_YAW_MOTOR_SPEED_PID_KP 800.0f
#define SMALL_YAW_MOTOR_SPEED_PID_KI 20.0f // 80.0f
#define SMALL_YAW_MOTOR_SPEED_PID_KD 200.0f
#define SMALL_YAW_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define SMALL_YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define SMALL_YAW_MOTOR_ANGLE_PID_KP 5.5f
#define SMALL_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_KD 100.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_IOUT 50.0f

#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KP 15.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KD 50.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_SPEED_PID_KP 5.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.0f
#define PITCH_MOTOR_SPEED_PID_KD 3.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 10.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.4f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 10.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 4.5f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.4f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 4.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

void Gimbal_Task(void const *argument);

#endif
