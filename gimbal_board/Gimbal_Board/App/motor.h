/***************************************************************************************************
 * @file: motor.h
 * @author: Shiki
 * @date: 2025.9.17
 * @brief: 哨兵电机库（上下C板公用）
 * @attention:
 ***************************************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include "struct_typedef.h"
#include "pid.h"

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

typedef enum
{
    MIT = 1,
    VEL_POS = 2,
    VEL = 3
} DM_CTRL_MODE; // 达秒电机控制模式结构体，一般用MIT模式

typedef struct // 大疆电机反馈值结构体
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    fp32 code;

} motor_measure_t;

typedef struct // 底盘轮电机结构体 3508
{
    int8_t spin_direction; // 控制轮电机正反转
    int16_t speed_now; // rpm
    fp32 speed_set;
    int16_t give_current;

    pid_type_def speed_pid;
} chassis_wheel_motor_t;

typedef struct // 底盘舵电机结构体 6020
{
    int16_t speed_now; // rpm
    fp32 speed_set;
    fp32 angle_now; //经过归一化后的舵电机当前位置。单位：度，范围0~180
    fp32 angle_set;
    fp32 angle_set_last;
    int16_t give_current;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
} chassis_steer_motor_t;

typedef struct // 云台小yaw轴，pitch轴的6020结构体
{
    fp32 INS_speed_now;
    fp32 INS_speed_set;
    fp32 INS_speed_set_last;
    fp32 INS_angle_now;
    fp32 INS_angle_set;
    fp32 INS_angle_set_last;
    uint16_t ENC_angle_now;
    int16_t ENC_speed_now;
    int16_t give_current;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
    pid_type_def auto_aim_pid;
} gimbal_motor_t;

typedef struct // 达妙电机结构体,大yaw 6006
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float toq;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    uint16_t target_pos;
    int16_t target_vel;
    float target_current;
    fp32 INS_speed_now;
    fp32 INS_speed_set;
    fp32 INS_speed_set_last;
    fp32 INS_angle_now;
    fp32 INS_angle_set;
    fp32 INS_angle_set_last;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
    pid_type_def auto_aim_pid;

} DM_motor_data_t; 

/*******************dji电机反馈数据结构体***********************/
extern motor_measure_t motor_measure_steer[4];  // 底盘舵电机6020
extern motor_measure_t motor_measure_wheel[4];  // 底盘轮电机3508
extern motor_measure_t motor_measure_small_yaw; // 小yaw电机6020
extern motor_measure_t motor_measure_pitch;     // pitch 6020
extern motor_measure_t motor_measure_shoot[2];  // 摩擦轮电机3508

/***********************电机控制结构体***********************************/
extern chassis_steer_motor_t chassis_steer_motor[4];
extern chassis_wheel_motor_t chassis_wheel_motor[4];
extern gimbal_motor_t gimbal_small_yaw_motor;
extern gimbal_motor_t gimbal_pitch_motor;
extern DM_motor_data_t DM_big_yaw_motor; // 达秒电机的反馈值已经包含在DM_motor_data_t中

#endif