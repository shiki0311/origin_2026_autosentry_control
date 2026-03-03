/****************************************************************
 * @file: 	Gimbal_Task.h
 * @author: Shiki
 * @date:	2025.10.5
 * @brief:	2026赛季哨兵云台任务,此文件存放云台控制参数的宏定义
 * @attention:
 ******************************************************************/
#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

/****************************************量纲转换*******************************************************/
#define DEGREE_TO_RAD 0.0172532f //  pi/180
#define RAD_TO_DEGREE 57.295779f
#define GM6020_ENC_TO_DEGREE 0.043945f //  360/8192
/****************************************重力补偿参数*******************************************************/
#define PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE (4600) // 用于补偿重力，pitch轴质心与转轴的连线与地面平行时抵消重力所需的力矩，单位是发给6020电机的目标电流
// #define PITCH_CENTROID_OFFSET_ANGLE 50.31f // 补偿pitch的ENC_angle
#define PITCH_CENTROID_OFFSET_ANGLE -10.28313f // 补偿pitch的ENC_angle
/**************************************************PID,前馈系数************************************************************************/
#define BIG_YAW_MOTOR_SPEED_PID_KP 0.1f
#define BIG_YAW_MOTOR_SPEED_PID_KI 0.0002f // 80.0f
#define BIG_YAW_MOTOR_SPEED_PID_KD 0.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_OUT 11.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT 0.4f
#define BIG_YAW_MOTOR_SPEED_FF 4.0f
#define BIG_YAW_MOTOR_CURRENT_FF 0.1f

#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KP 25.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KI 0.002f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KD 12.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_OUT 150.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_IOUT 5.0f

#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KP 15.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KI 0.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KD 25.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_OUT 1200.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_IOUT 100.0f

#define SMALL_YAW_MOTOR_SPEED_PID_KP 180.0f //200
#define SMALL_YAW_MOTOR_SPEED_PID_KI 0.01f 
#define SMALL_YAW_MOTOR_SPEED_PID_KD 5.0f
#define SMALL_YAW_MOTOR_SPEED_PID_MAX_OUT 15000.0f
#define SMALL_YAW_MOTOR_SPEED_PID_MAX_IOUT 3000.0f
#define SMALL_YAW_MOTOR_SPEED_FF 4.0f
#define SMALL_YAW_MOTOR_CURRENT_FF 0.02f

#define SMALL_YAW_MOTOR_ANGLE_PID_KP 10.0f //20
#define SMALL_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_KD 10.0f  //20
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_OUT 2000.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KP 10.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KD 20.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_SPEED_PID_KP 100.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.03f
#define PITCH_MOTOR_SPEED_PID_KD 15.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define PITCH_MOTOR_ANGLE_PID_KP 25.0f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 10.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 2000.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.4f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 4.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f
/****************************************云台零点，限位，电机运动参数*******************************************************/
#define SMALL_YAW_MIDDLE_ENC_ZERO 6474 // 小yaw轴中间位置编码器值，作为大yaw轴跟随小yaw轴时的参考零点

#define SMALL_YAW_NAV_SEEK_ECD_MAX 7400 //导航索敌模式搜索角度最大值,为小yaw电机编码器值
#define SMALL_YAW_NAV_SEEK_ECD_MIN 5500 //导航索敌模式搜索角度最小值，为小yaw电机编码器值减去8192（跟安装位置有关，索敌时电机活动范围越过编码器零点了）
#define SMALL_YAW_NAV_SEEK_STEP 0.2 //导航索敌模式下小yaw轴每次调整的角度步进，单位：度

#define SMALL_YAW_AUTOAIM_ECD_MAX 5561
#define SMALL_YAW_AUTOAIM_ECD_MIN 4365

// #define PITCH_ECD_ANGLE_MAX (3146*GM6020_ENC_TO_DEGREE) //pitch轴电子限位最大角度，用编码器值标定
// #define PITCH_ECD_ANGLE_MIN (1908*GM6020_ENC_TO_DEGREE) //pitch轴电子限位最小角度，用编码器值标定
#define PITCH_ECD_ANGLE_MAX (1800 * GM6020_ENC_TO_DEGREE) // pitch轴电子限位最大角度，用编码器值标定
#define PITCH_ECD_ANGLE_MIN (530 * GM6020_ENC_TO_DEGREE) // pitch轴电子限位最小角度，用编码器值标定

#define PITCH_NAV_SEEK_ENEMY_ANGLE_MAX 15.0f 
#define PITCH_NAV_SEEK_ENEMY_ANGLE_MIN -22.0f 
#define PITCH_NAV_SEEK_ENEMY_STEP 0.15f 
#define PITCH_NAV_SEEK_OUTPOST_ANGLE_MAX 15.0f  
#define PITCH_NAV_SEEK_OUTPOST_ANGLE_MIN -22.0f
#define PITCH_NAV_SEEK_OUTPOST_STEP 0.15f

#define REMOTE_CONTROL_YAW_MAX_SPEED 5.0f  //rad/s
#define REMOTE_CONTROL_PITCH_MAX_SPEED 5.0f  //rad/s

void Gimbal_Task(void const *argument);

#endif
