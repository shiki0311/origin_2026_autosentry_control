/*****************************************************************************************************************************
 * @file: Chassis_Power_Limitor.h
 * @author: Shiki
 * @date: 2025.12.19
 * @brief:	起源哨兵2026赛季底盘功率限制模块头文件
 *****************************************************************************************************************************/

#ifndef CHASSIS_POWER_LIMITOR_H
#define CHASSIS_POWER_LIMITOR_H

#include "motor.h"
/*****************************************************************************************************************************
 电机输入功率公式：P_in = T*w + K1*w^2 + K2*T^2 + c
 其中T为电机输出的转矩，w为电机转速，K1为电机转速产生的功率损耗系数,K2为电机转速产生的功率损耗系数,c为电机力矩平方产生的功率损耗系数
 T与力矩电流控制值（give_current）成正比关系,w可以通过电机反馈的速度值得到，因此电机输入功率公式可以写成：
 P_in = K_P*motor.give_current*motor.speed_now + K_W*motor.speed_now^2 + K_T*(motor.give_current)^2 + STATIC_POWER

 注意：T*w描述的是电机实际输出的机械功率，对于直驱电机来说就是输出转矩乘以转速再乘个量纲变换的系数，但对于减速电机来说，转矩和转速在转子端和输出轴端的值不同，
 输出轴力矩等于转子力矩乘以减速比，输出角速度等于转子角速度除以减速比。
 以3508电机为例，电机反馈电流映射到的是输出轴端转矩(在减速箱为原装的情况下），而电机反馈的速度是转子端的速度，进行功率建模的时候要将转矩和转速都换算到同一端，本算法换算到转子端。
 *****************************************************************************************************************************/
/* 3508电机功率模型系数 */
#define M3508_K_P 1.99688994e-6f  // 通过计算得出，非拟合，计算公式：rpm转为rad/s的量纲变换系数(约为1/9.55)*give_current到转子转矩的转换系数((（20/16384)/原装减速箱减速比）*转矩常数)
#define M3508_K_W 1.453e-7f       // 通过拟合曲线得到
#define M3508_K_T 1.23e-7f        // 通过拟合曲线得到
#define M3508_STATIC_POWER 1.581f // 通过拟合曲线得到
/* 6020电机功率模型系数 */
#define GM6020_K_P 1.42074e-6f    // 通过计算得出，非拟合，计算公式：rpm转为rad/s的量纲变换系数(约为1/9.55)*give_current到转子转矩的转换系数((3/16384）*转矩常数)
#define GM6020_K_W 8.9596e-7f     // 通过拟合曲线得到
#define GM6020_K_T 3.0609e-7f     // 通过拟合曲线得到
#define GM6020_STATIC_POWER 0.69f // 通过拟合曲线得到

#define MOTOR_NUM 4   // 每组电机数量，轮电机和舵电机都是4

typedef struct // 电机组功率模型结构体
{
    /* 电机功率模型参数,定值 */
    const float k_p;
    const float k_w;
    const float k_t;
    const float p_static;

    float weight[MOTOR_NUM];        // 电机权重
    float alpha[MOTOR_NUM];         // 电机目标电流缩放的幅度值，范围为（MIN_CMD_CURRENT/abs(speed_pid.give_current)）~ MAX_CMD_CURRENT/abs(speed_pid.give_current))
    float min_weight;              // 电机组最低权重
    float max_weight;              // 电机组最高权重

    /*功率控制后(如果需要功率控制的话）的电机输入功率为:
    K_P * (alpha * motor.give_current) * motor.speed_now + K_W * motor.speed_now ^ 2 + K_T * (alpha * motor.give_current) ^ 2 + STATIC_POWER
    可以简化为 quadratic_coeff * alpha^2 + linear_coeff * alpha + constant ，目的是减小mcu计算量*/
    float quadratic_coeff[MOTOR_NUM]; // alpha的二次项系数
    float linear_coeff[MOTOR_NUM];    // alpha的一次项系数
    float constant[MOTOR_NUM];        // 常数项
} power_model_t;

typedef enum { 
    NORMAL_WEIGHT_ALLOCATE, // 正常权重分配方式
    PASS_BUMPY_ALLOCATE  // 颠簸路面权重分配方式
}weight_allocate_mode_t;

typedef struct // 功率控制结构体
{
    float chassis_power_predicted; // 还未经过功率控制的底盘预测功率
    float chassis_power_processed; // 功率控制后的底盘预测功率,调试用
    unsigned char iter_num;        // 二分法迭代次数,调试用
    float final_lambda;            // 最终拉格朗日乘子，调试用
    weight_allocate_mode_t weight_allocate_mode; // 权重分配模式

    power_model_t wheel_motors; // 轮电机组功率模型
    power_model_t steer_motors; // 舵电机组功率模型
} power_limitor_t;

void Chassis_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], chassis_steer_motor_t steer_motor[4], float P_max, weight_allocate_mode_t weight_allocate_mode);

#endif