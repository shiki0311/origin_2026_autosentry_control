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
#define PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE (5300) // 用于补偿重力，pitch轴质心与转轴的连线与地面平行时抵消重力所需的力矩，单位是发给6020电机的目标电流
#define PITCH_CENTROID_OFFSET_ANGLE -55.28313f // 补偿pitch的INS_angle
/**************************************************PID,前馈系数************************************************************************/
#define BIG_YAW_MOTOR_SPEED_PID_KP 0.12f
#define BIG_YAW_MOTOR_SPEED_PID_KI 0.0002f // 80.0f
#define BIG_YAW_MOTOR_SPEED_PID_KD 0.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_OUT 11.0f
#define BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT 0.15f
#define BIG_YAW_MOTOR_SPEED_FF 5.0f //5.0
#define BIG_YAW_MOTOR_CURRENT_FF 0.08f //0.02
#define CHASSIS_FRICTION_COMPENSATE_COEFF 0.12f // 相对于底盘的底盘摩擦补偿

#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KP 25.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KI 0.002f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_KD 12.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_OUT 150.0f
#define BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_IOUT 5.0f

#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KP 20.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KI 0.002f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KD 30.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_OUT 1200.0f
#define BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_IOUT 10.0f

#define BIG_YAW_MOTOR_OMNI_PID_KP 10.0f //全向感知pid
#define BIG_YAW_MOTOR_OMNI_PID_KI 0.002f
#define BIG_YAW_MOTOR_OMNI_PID_KD 30.0f
#define BIG_YAW_MOTOR_OMNI_PID_MAX_OUT 1500.0f
#define BIG_YAW_MOTOR_OMNI_PID_MAX_IOUT 10.0f

#define BIG_YAW_MOTOR_AUTO_AIM_PID_KP 5.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_KD 2.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 120.0f
#define BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KP 200.0f //200
#define SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KI 0.01f 
#define SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KD 5.0f

#define SMALL_YAW_MOTOR_RC_SPEED_PID_KP 100.0f // 遥控器控制的时候p改小，防止p轴震
#define SMALL_YAW_MOTOR_RC_SPEED_PID_KI 0.01f
#define SMALL_YAW_MOTOR_RC_SPEED_PID_KD 15.0f

#define SMALL_YAW_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define SMALL_YAW_MOTOR_SPEED_PID_MAX_IOUT 3000.0f
#define SMALL_YAW_MOTOR_SPEED_FF 20.0f  //8
#define SMALL_YAW_MOTOR_CURRENT_FF 70.0f //40

#define SMALL_YAW_MOTOR_ANGLE_PID_KP 25.0f //20
#define SMALL_YAW_MOTOR_ANGLE_PID_KI 0.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_KD 25.0f  //20
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_OUT 2000.0f
#define SMALL_YAW_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KP 30.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KI 0.002f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_KD 20.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 2000.0f
#define SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 70.0f

#define PITCH_MOTOR_SPEED_PID_KP 190.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.0008f
#define PITCH_MOTOR_SPEED_PID_KD 12.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 100.0f
#define PITCH_MOTOR_SPEED_FF 5.0f
#define PITCH_MOTOR_CURRENT_FF 10.0f

#define PITCH_MOTOR_ANGLE_PID_KP 30.0f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 25.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 2000.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 30.0f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 25.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 2000.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f
/****************************************云台零点，限位，电机运动参数*******************************************************/
#define SMALL_YAW_MIDDLE_ENC_ZERO 3525 // 小yaw轴中间位置编码器值，作为大yaw轴跟随小yaw轴时的参考零点

#define SMALL_YAW_NAV_SEEK_ECD_MAX 4140 //导航索敌模式搜索角度最大值,为小yaw电机编码器值
#define SMALL_YAW_NAV_SEEK_ECD_MIN 2900 //导航索敌模式搜索角度最小值，为小yaw电机编码器值减去8192（跟安装位置有关，索敌时电机活动范围越过编码器零点了）
#define SMALL_YAW_NAV_SEEK_STEP 0.16 //导航索敌模式下小yaw轴每次调整的角度步进，单位：度

#define PITCH_ECD_ANGLE_MAX (7100 * GM6020_ENC_TO_DEGREE) // pitch轴电子限位最大角度，用编码器值标定
#define PITCH_ECD_ANGLE_MIN (6050 * GM6020_ENC_TO_DEGREE) // pitch轴电子限位最小角度，用编码器值标定 

#define PITCH_NAV_SEEK_ENEMY_ANGLE_MAX 4.0f 
#define PITCH_NAV_SEEK_ENEMY_ANGLE_MIN -4.0f 
#define PITCH_NAV_SEEK_ENEMY_STEP 0.12f 
#define PITCH_NAV_SEEK_OUTPOST_ANGLE_MAX 15.0f  
#define PITCH_NAV_SEEK_OUTPOST_ANGLE_MIN -21.0f
#define PITCH_NAV_SEEK_OUTPOST_STEP 0.15f

#define BIG_YAW_AUTOAIM_STOP_RANGE 10.0f // 大yaw轴自瞄死区，单位：度
#define BIG_YAW_AUTOAIM_SLOW_FOLLOW_RANGE 30.0f // 大yaw轴自瞄缓慢跟随范围，单位：度

#define REMOTE_CONTROL_YAW_MAX_SPEED 5.0f  //rad/s
#define REMOTE_CONTROL_PITCH_MAX_SPEED 5.0f  //rad/s

void Gimbal_Task(void const *argument);

#endif
