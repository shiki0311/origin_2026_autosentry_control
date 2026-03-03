/*****************************************************************************************************************************
 * @file: Chassis_Power_Limitor.c
 * @author: Shiki
 * @date: 2025.12.19
 * @brief:	起源哨兵2026赛季底盘功率限制模块源文件
 *
 功率控制模块工作流程：计算底盘当前预测功率->判断是否超过功率上限->若超过则进行拉格朗日乘子法求解->根据求解结果调整电机目标电流
*****************************************************************************************************************************/
#include "Chassis_Power_Limitor.h"
#include "user_common_lib.h"
#include "detect_task.h"
/*****************************************************************************************************************************
/* 功率控制器参数 */
#define MAX_CMD_CURRENT 16384.0f        // 经过功率控制后的最大控制电流
#define MIN_CMD_CURRENT -16384.0f       // 经过功率控制后的最小控制电流
#define LAMBDA_INITIAL_UPEER_BOUND 0.05f // 拉格朗日乘子初始上限
#define LAMBDA_INITIAL_LOWER_BOUND 0.0f // 拉格朗日乘子初始下限
#define LAMBDA_UPPER_BOUND_MAX_ITER 10  // 寻找拉格朗日乘子上限时最大迭代次数
#define LAMBDA_UPPER_BOUND_STEP 10      // 寻找拉格朗日乘子上限时的迭代步长
#define LAMBDA_MAX_ITER 30              // 二分法寻找lambda时的最大迭代次数
#define POWER_TOLERANCE 0.8f            // 功率误差容限，单位W
/*****************************************************************************************************************************/
// 函数声明
void Chassis_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], chassis_steer_motor_t steer_motor[4], float P_max, weight_allocate_mode_t weight_allocate_mode);
static void Lagrange_Solve_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], chassis_steer_motor_t steer_motor[4], float P_max, weight_allocate_mode_t weight_allocate_mode);
static float Calculate_Initial_Power(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4]);
static void Calculate_All_Alpha_Coefficients(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4]);
static void Calculate_Alpha(power_limitor_t *power_limiter, float lambda, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4]);
static float Calculate_Power_With_Alpha(power_limitor_t *power_limiter);
static void Allocate_Motor_Weight(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4], weight_allocate_mode_t weight_allocate_mode);

float lambda;     // debug用
int iter;         // debug用
float power_iter; // debug用
/**
 * @description:功率控制模块对底盘任务的接口，在chassis_task中调用此函数即可，此函数会将发送给电机的目标电流调整到功率控制后的值
 * @return {*}
 * @param {chassis_wheel_motor_t} wheel_motor 轮电机结构体数组指针
 * @param {chassis_steer_motor_t} steer_motor 舵电机结构体数组指针
 * @param {float} P_max 功率上限
 */
void Chassis_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], chassis_steer_motor_t steer_motor[4], float P_max, weight_allocate_mode_t weight_allocate_mode)
{
    power_limiter->chassis_power_predicted = Calculate_Initial_Power(power_limiter, wheel_motor, steer_motor);

    if (power_limiter->chassis_power_predicted <= P_max)
        return; // 预测功率没有达到上限，直接返回
    else
    {
        // 因为目标函数的变量是电流缩放系数，所以需要避免目标电流控制值为0的情况，matlab实测把0改成1或者-1对功率几乎无影响，这里较保守选择与转速同号，让修改后的功率比原来大
        for (int i = 0; i < 4; i++)
        {
            if (wheel_motor[i].give_current == 0)
            {
                wheel_motor[i].give_current = (wheel_motor[i].speed_now > 0) ? 1.0f : -1.0f;
            }
            if (steer_motor[i].give_current == 0)
            {
                steer_motor[i].give_current = (steer_motor[i].speed_now > 0) ? 1.0f : -1.0f;
            }
        }

        Calculate_All_Alpha_Coefficients(power_limiter, wheel_motor, steer_motor);
        Lagrange_Solve_Power_Control(power_limiter, wheel_motor, steer_motor, P_max, weight_allocate_mode);

        // 根据求解结果调整电机目标电流
        for (int i = 0; i < 4; i++)
        {
            wheel_motor[i].give_current = (int16_t)(wheel_motor[i].give_current * power_limiter->wheel_motors.alpha[i]);
            steer_motor[i].give_current = (int16_t)(steer_motor[i].give_current * power_limiter->steer_motors.alpha[i]);
        }
    }
}

/**
 * @description:舵轮功率控制本质是一个带不等式约束的凸二次优化问题，因此可以用拉格朗日乘子法求解，满足KKT条件的解即为全局最优解
 *              此函数具体求解过程:先找到满足功率限制的λ上限和下限，然后在该区间内用二分法寻找合适的λ值，然后用λ解出每个电机的电流缩放倍数alpha，
 *              使得计算出的功率等于功率上限。
 *              如果寻找λ上限时迭代到最大次数依然无法满足功率限制，说明功率限制十分苛刻，每个电机取到让功率最小的控制电流仍然无法满足功率限制，
 *              则直接输出最后一次迭代得到的alpha，并且跳过后续的二分搜索λ。
 * @return {*}
 * @param {chassis_wheel_motor_t} wheel_motor 轮电机结构体数组指针
 * @param {chassis_steer_motor_t} steer_motor 舵电机结构体数组指针
 * @param {float} P_max 功率上限
 */
static void Lagrange_Solve_Power_Control(power_limitor_t *power_limiter, chassis_wheel_motor_t wheel_motor[4], chassis_steer_motor_t steer_motor[4], float P_max, weight_allocate_mode_t weight_allocate_mode)
{
    float lambda_lower_bound = LAMBDA_INITIAL_LOWER_BOUND;
    float lambda_upper_bound = LAMBDA_INITIAL_UPEER_BOUND;

    // 计算电机权重
    Allocate_Motor_Weight(power_limiter, wheel_motor, steer_motor, weight_allocate_mode);
    // 寻找λ的上限,如果初始上限不满足功率约束就顺便调整下限，只要功率约束不是太紧一般初始上限就能满足要求
    for (int i = 0; i < LAMBDA_UPPER_BOUND_MAX_ITER; i++)
    {
        Calculate_Alpha(power_limiter, lambda_upper_bound, wheel_motor, steer_motor);
        float power = Calculate_Power_With_Alpha(power_limiter);
        if (power > P_max)
        {
            if (i == LAMBDA_UPPER_BOUND_MAX_ITER - 1)
            {
                // 说明功率限制过于苛刻，无法找到拉姆达（拉姆达取无穷大也无法满足条件），直接输出当前结果
                power_limiter->chassis_power_processed = power;
                power_limiter->final_lambda = lambda_upper_bound;
                return;
            }
            else
            {
                lambda_lower_bound = lambda_upper_bound;
                lambda_upper_bound *= LAMBDA_UPPER_BOUND_STEP;
            }
        }
        else
        {
            break;
        }
    }
    // 用二分法寻找合适的λ
    for (iter = 0; iter < LAMBDA_MAX_ITER; iter++)
    {
        lambda = (lambda_lower_bound + lambda_upper_bound) / 2.0f;
        Calculate_Alpha(power_limiter, lambda, wheel_motor, steer_motor);
        power_iter = Calculate_Power_With_Alpha(power_limiter);

        if (power_iter < P_max && (P_max - power_iter) < POWER_TOLERANCE)
        {
            power_limiter->final_lambda = lambda;
						power_limiter->iter_num = iter + 1;
						power_limiter->chassis_power_processed = power_iter;
            return;
        }
        else if (power_iter > P_max)
        {
            lambda_lower_bound = lambda;
        }
        else
        {
            lambda_upper_bound = lambda;
        }
    }
    power_limiter->final_lambda = lambda;
    power_limiter->iter_num = iter + 1;
    power_limiter->chassis_power_processed = power_iter;
}
/**
 * @description: 计算还未经过功率控制的底盘的预测功率，用于判断是否需要进行功率限制
 * @return {*} 预测的底盘功率值，单位：W
 * @param {chassis_wheel_motor_t} wheel_motor 轮电机结构体数组指针
 * @param {chassis_steer_motor_t} steer_motor 舵电机结构体数组指针
 */
static float Calculate_Initial_Power(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4])
{
    float initial_total_power = 0;
    float initial_wheel_power = 0;
    float initial_steer_power = 0;

    for (int i = 0; i < 4; i++)
    {
        initial_wheel_power = power_limiter->wheel_motors.k_p * wheel_motor[i].give_current * wheel_motor[i].speed_now +
                              power_limiter->wheel_motors.k_w * wheel_motor[i].speed_now * wheel_motor[i].speed_now +
                              power_limiter->wheel_motors.k_t * wheel_motor[i].give_current * wheel_motor[i].give_current +
                              power_limiter->wheel_motors.p_static;

        if (initial_wheel_power < 0 || toe_is_error(WHEEL_MOTOR_1_TOE + i)) // 如果功率小于0或者对应电机通讯丢失，认为该电机功率为0
            initial_wheel_power = 0;

        initial_steer_power = power_limiter->steer_motors.k_p * steer_motor[i].give_current * steer_motor[i].speed_now +
                              power_limiter->steer_motors.k_w * steer_motor[i].speed_now * steer_motor[i].speed_now +
                              power_limiter->steer_motors.k_t * steer_motor[i].give_current * steer_motor[i].give_current +
                              power_limiter->steer_motors.p_static;

        if (initial_steer_power < 0 || toe_is_error(STEER_MOTOR_1_TOE + i)) // 如果功率小于0或者对应电机通讯丢失，认为该电机功率为0
            initial_steer_power = 0;

        initial_total_power += (initial_wheel_power + initial_steer_power);
    }
    return initial_total_power;
}

/**
 * @description: 计算所有电机功率表达式中的α系数（功率表达式见power_model_t定义中的注释）,包括二次项系数，一次项系数和常数项，用于减小计算量
 * @return {*}
 * @param {chassis_wheel_motor_t} wheel_motor
 * @param {chassis_steer_motor_t} steer_motor
 */
static void Calculate_All_Alpha_Coefficients(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.quadratic_coeff[i] = power_limiter->wheel_motors.k_t * wheel_motor[i].give_current * wheel_motor[i].give_current;
        power_limiter->wheel_motors.linear_coeff[i] = power_limiter->wheel_motors.k_p * wheel_motor[i].speed_now * wheel_motor[i].give_current;
        power_limiter->wheel_motors.constant[i] = power_limiter->wheel_motors.k_w * wheel_motor[i].speed_now * wheel_motor[i].speed_now + power_limiter->wheel_motors.p_static;

        power_limiter->steer_motors.quadratic_coeff[i] = power_limiter->steer_motors.k_t * steer_motor[i].give_current * steer_motor[i].give_current;
        power_limiter->steer_motors.linear_coeff[i] = power_limiter->steer_motors.k_p * steer_motor[i].speed_now * steer_motor[i].give_current;
        power_limiter->steer_motors.constant[i] = power_limiter->steer_motors.k_w * steer_motor[i].speed_now * steer_motor[i].speed_now + power_limiter->steer_motors.p_static;
    }
}

/**
 * @description: 计算给定拉格朗日乘子下底盘八个电机的α系数
 * @return {*}
 * @param {float} lambda
 * @param {chassis_wheel_motor_t} wheel_motor
 * @param {chassis_steer_motor_t} steer_motor
 */
static void Calculate_Alpha(power_limitor_t *power_limiter, float lambda, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        power_limiter->wheel_motors.alpha[i] = limit((2.0f * power_limiter->wheel_motors.weight[i] - lambda * power_limiter->wheel_motors.linear_coeff[i]) /
                                                         (2.0f * power_limiter->wheel_motors.weight[i] + 2.0f * power_limiter->wheel_motors.quadratic_coeff[i] * lambda),
                                                     MIN_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current),
                                                     MAX_CMD_CURRENT / my_fabsf((float)wheel_motor[i].give_current));

        power_limiter->steer_motors.alpha[i] = limit((2.0f * power_limiter->steer_motors.weight[i] - lambda * power_limiter->steer_motors.linear_coeff[i]) /
                                                         (2.0f * power_limiter->steer_motors.weight[i] + 2.0f * power_limiter->steer_motors.quadratic_coeff[i] * lambda),
                                                     MIN_CMD_CURRENT / my_fabsf((float)steer_motor[i].give_current),
                                                     MAX_CMD_CURRENT / my_fabsf((float)steer_motor[i].give_current));
    }
}
/**
 * @description: 计算给定拉格朗日乘子下底盘八个电机的总功率
 * @return {*} 预测的底盘功率值，单位：W
 * @param {chassis_wheel_motor_t} wheel_motor 轮电机结构体数组指针
 * @param {chassis_steer_motor_t} steer_motor 舵电机结构体数组指针
 */
static float Calculate_Power_With_Alpha(power_limitor_t *power_limiter)
{
    float alpha_total_power = 0;
    float alpha_wheel_power = 0;
    float alpha_steer_power = 0;

    for (int i = 0; i < 4; i++)
    {
        alpha_wheel_power = power_limiter->wheel_motors.quadratic_coeff[i] * power_limiter->wheel_motors.alpha[i] * power_limiter->wheel_motors.alpha[i] +
                            power_limiter->wheel_motors.linear_coeff[i] * power_limiter->wheel_motors.alpha[i] +
                            power_limiter->wheel_motors.constant[i];

        if (alpha_wheel_power < 0 || toe_is_error(WHEEL_MOTOR_1_TOE + i)) // 如果功率小于0或者对应电机通讯丢失，认为该电机功率为0
            alpha_wheel_power = 0;

        alpha_steer_power = power_limiter->steer_motors.quadratic_coeff[i] * power_limiter->steer_motors.alpha[i] * power_limiter->steer_motors.alpha[i] +
                            power_limiter->steer_motors.linear_coeff[i] * power_limiter->steer_motors.alpha[i] +
                            power_limiter->steer_motors.constant[i];

        if (alpha_steer_power < 0 || toe_is_error(STEER_MOTOR_1_TOE + i)) // 如果功率小于0或者对应电机通讯丢失，认为该电机功率为0
            alpha_steer_power = 0;

        alpha_total_power += (alpha_wheel_power + alpha_steer_power);
    }
    return alpha_total_power;
}

/**
 * @description: 根据当前电机状态动态分配电机权重
 * @return {*}
 * @param {chassis_wheel_motor_t} wheel_motor
 * @param {chassis_steer_motor_t} steer_motor
 */
static void Allocate_Motor_Weight(power_limitor_t *power_limiter, const chassis_wheel_motor_t wheel_motor[4], const chassis_steer_motor_t steer_motor[4], weight_allocate_mode_t weight_allocate_mode)
{
    switch (weight_allocate_mode)
    {
    case NORMAL_WEIGHT_ALLOCATE:
    {
        power_limiter->steer_motors.min_weight = 1.0f;
        power_limiter->steer_motors.max_weight = 10.0f;
        power_limiter->wheel_motors.min_weight = 0.5f;
        power_limiter->wheel_motors.max_weight = 5.0f;

        #define STEER_WEIGHT_CURRENT_GAIN (power_limiter->steer_motors.max_weight / 16384.0f) // 舵电机电流给舵电机带来的权重增益系数
        #define STEER_WEIGHT_SPEED_GAIN (power_limiter->steer_motors.max_weight / 400.0f)              // 舵电机转速给舵电机带来的权重增益系数
        #define WHEEL_WEIGHT_STEER_ANGLE_ERROR_GAIN (power_limiter->wheel_motors.max_weight / 90.0f)          // 舵角误差给轮电机带来的权重增益系数

        break;
    }
    case PASS_BUMPY_ALLOCATE:
    {
        power_limiter->steer_motors.min_weight = 5.0f;
        power_limiter->steer_motors.max_weight = 30.0f;
        power_limiter->wheel_motors.min_weight = 0.1f;
        power_limiter->wheel_motors.max_weight = 5.0f;

        #define STEER_WEIGHT_CURRENT_GAIN (2.0f * power_limiter->steer_motors.max_weight / 16384.0f)        // 舵电机电流给舵电机带来的权重增益系数
        #define STEER_WEIGHT_SPEED_GAIN (0.5f * power_limiter->steer_motors.max_weight / 400.0f)            // 舵电机转速给舵电机带来的权重增益系数
        #define WHEEL_WEIGHT_STEER_ANGLE_ERROR_GAIN (0.5f * power_limiter->wheel_motors.max_weight / 90.0f) // 舵角误差给轮电机带来的权重增益系数

        break;
    }
    default:
        break;
    }
    for (int i = 0; i < 4; i++)
    {
        power_limiter->steer_motors.weight[i] = limit(steer_motor[i].give_current * STEER_WEIGHT_CURRENT_GAIN + steer_motor[i].speed_now * STEER_WEIGHT_SPEED_GAIN, power_limiter->steer_motors.min_weight, power_limiter->steer_motors.max_weight);

        float steer_angle_error = my_fabsf(steer_motor[i].angle_set - steer_motor[i].angle_now);
        power_limiter->wheel_motors.weight[i] = limit((90.0f - steer_angle_error) * WHEEL_WEIGHT_STEER_ANGLE_ERROR_GAIN, power_limiter->wheel_motors.min_weight, power_limiter->wheel_motors.max_weight);
    }
}
