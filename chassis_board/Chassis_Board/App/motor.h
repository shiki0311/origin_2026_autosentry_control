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

#define get_motor_measure_LK(ptr, data)                                 \
    {                                                                   \
        (ptr)->temperature = (int8_t)(data)[1];                         \
        (ptr)->given_current = (int16_t)((data)[2] | ((data)[3] << 8)); \
        (ptr)->speed = (int16_t)((data)[4] | ((data)[5] << 8));         \
        (ptr)->ecd = (uint16_t)((data)[6] | ((data)[7] << 8));          \
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
} motor_measure_t;

typedef struct
{
    int8_t temperature;    // 电机温度
    int16_t given_current; // 转矩电流值
    int16_t speed;         // 电机转速（dps）
    uint16_t ecd;          // 编码器位置值(0-65535)
} LK_motor_measure_t;

typedef struct // 底盘轮电机结构体 3508
{
    int8_t spin_direction; // 控制轮电机正反转
    int16_t speed_now;     // rpm
    fp32 speed_set;
    int16_t give_current;

    pid_type_def speed_pid;
} chassis_wheel_motor_t;

typedef struct // 底盘舵电机结构体 6020
{
    int16_t speed_now; // rpm
    fp32 speed_set;
    fp32 angle_now; // 经过归一化后的舵电机当前位置。单位：度，范围0~180
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
    fp32 ENC_angle_set;
    fp32 ENC_angle_now;
    fp32 ENC_angle_set_last;

    fp32 speed_ff;   // 速度前馈项
    fp32 current_ff; // 电流(力矩）前馈项
    int16_t give_current;

    pid_type_def speed_pid;
    pid_type_def angle_pid;
    pid_type_def auto_aim_pid;
} gimbal_motor_t;

typedef struct // 大yaw 达妙6006结构体
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

    fp32 target_current;
    fp32 INS_speed_now;
    fp32 INS_speed_set;
    fp32 INS_speed_set_last;
    fp32 INS_angle_now;
    fp32 INS_angle_set;
    fp32 INS_angle_set_last;

    fp32 speed_ff;   // 速度前馈项
    fp32 current_ff; // 电流(力矩）前馈项

    pid_type_def speed_pid;
    pid_type_def nav_angle_pid;
    pid_type_def follow_small_yaw_pid;
    pid_type_def auto_aim_pid;
    pid_type_def omni_pid; // 全向感知pid

} DM_motor_data_t;

typedef struct // 摩擦轮电机结构体 3508
{
    int16_t speed_now; // rpm
    fp32 speed_set;
    int16_t give_current;
    int16_t given_current;

    pid_type_def speed_pid;
} fric_motor_t;

typedef struct
{
    int8_t temperature;
    fp32 speed_now; // rpm
    fp32 speed_set;
    fp32 angle_now; // degree
    fp32 angle_set;
    int16_t give_current;
    int16_t given_current;

    fp32 set_base[10];    // 10个拨弹盘固定角度
    uint8_t now_aim_pose; // 目标拨弹盘位置,为set_base数组的索引
    uint8_t now_pose;     // 当前拨弹盘位置,为set_base数组的索引

    pid_type_def speed_pid;
    pid_type_def angle_pid;
} dial_motor_t;
/*******************dji电机反馈数据结构体***********************/
extern motor_measure_t motor_measure_steer[4];  // 底盘舵电机6020
extern motor_measure_t motor_measure_wheel[4];  // 底盘轮电机3508
extern motor_measure_t motor_measure_small_yaw; // 小yaw电机6020
extern motor_measure_t motor_measure_pitch;     // pitch 6020
extern motor_measure_t motor_measure_fric[2];   // 摩擦轮电机3508
extern LK_motor_measure_t motor_measure_dial;   // 拨弹盘电机
/***********************电机控制结构体***********************************/
extern chassis_steer_motor_t chassis_steer_motor[4];
extern chassis_wheel_motor_t chassis_wheel_motor[4];
extern gimbal_motor_t gimbal_small_yaw_motor;
extern gimbal_motor_t gimbal_pitch_motor;
extern DM_motor_data_t DM_big_yaw_motor; // 达秒电机的反馈值已经包含在DM_motor_data_t中
extern fric_motor_t fric_motor[2];
extern dial_motor_t LK_dial_motor;

#endif