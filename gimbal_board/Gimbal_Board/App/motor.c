/***************************************************************************************************
 * @file: motor.c
 * @author: Shiki
 * @date: 2025.9.17
 * @brief: 哨兵电机库（上下C板公用）
 * @attention:
 ***************************************************************************************************/

 #include "motor.h"

/*******************dji电机反馈数据结构体***********************/
motor_measure_t motor_measure_steer[4] = {0};  // 底盘舵电机6020
motor_measure_t motor_measure_wheel[4] = {0};  // 底盘轮电机3508
motor_measure_t motor_measure_small_yaw = {0}; // 小yaw电机6020
motor_measure_t motor_measure_pitch = {0};     // pitch 6020
motor_measure_t motor_measure_shoot[2] = {0};  // 摩擦轮电机3508

/***********************电机控制结构体***********************************/
chassis_steer_motor_t chassis_steer_motor[4] = {0};
chassis_wheel_motor_t chassis_wheel_motor[4] = {0};
gimbal_motor_t gimbal_small_yaw_motor = {0};
gimbal_motor_t gimbal_pitch_motor = {0};
DM_motor_data_t DM_big_yaw_motor = {0}; // 达秒电机的反馈值已经包含在DM_motor_data_t中