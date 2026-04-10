/****************************************************************
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *
 *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *            佛祖保佑     永不宕机     永无BUG
 ******************************************************************************
 * @file    user_common_lib.c
 * @author  Shiki
 * @version V1.0.0
 * @date    2025.6.18
 * @brief   shiki の 常用函数库
 ******************************************************************************/
#include "user_common_lib.h"

/**
 * @brief 将输入角度值限制在[-180, 180]度区间内
 * @param in 输入角度值（单位：度），允许任意浮点数值
 * @return float 调整后的等效角度值，保证在[-180, 180]区间内
 */
float Limit_To_180(float in)
{
    while (in < -180 || in > 180)
    {
        if (in < -180)
            in = in + 360;
        else if (in > 180)
            in = in - 360;
    }
    return in;
}

/**
 * @brief 实现一个斜坡控制算法，用于在参考值和设定值之间平滑过渡。
 *        该函数的目的是通过限制变化率（加速度），避免突变。
 * @param ref 当前位置或状态的参考值。
 * @param set 要达到的目标位置或状态。
 * @param accel 加速度限制，控制变化的速度。
 * @return 应用斜坡控制后得到的调整值，确保平滑过渡。
 */
float ramp_control(float ref, float set, float accel)
{
    float ramp = limit(accel, 0, 1) * (set - ref);
    return ref + ramp;
}

/**
 * @brief 将给定的数据值限制在指定的最小值和最大值范围内
 * @param data 待限制的数据值
 * @param min 数据值的下限
 * @param max 数据值的上限
 * @return 返回限制后的数据值
 */
float limit(float data, float min, float max)
{
    if (data >= max)
        return max;
    if (data <= min)
        return min;
    return data;
}

/**
 * 将无符号整数转换为浮点数
 *
 * @param x_int 整数输入，代表一个无符号整数值
 * @param x_min 转换范围的最小值
 * @param x_max 转换范围的最大值
 * @param bits 整数x_int的位数
 * @return 转换后的浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * 将浮点数转换为无符号整数。
 *
 * @param x 待转换的浮点数。
 * @param x_min x的最小可能值，定义了输入范围的下界。
 * @param x_max x的最大可能值，定义了输入范围的上界。
 * @param bits 转换结果所用的位数，决定了输出整数的范围。
 * @return 转换后的无符号整数。
 *
 * 注意：函数内部首先计算输入范围的跨度（span）和起始点（offset），然后应用线性映射公式进行转换。
 * 这种映射方式确保了输入浮点数的相对差异在转换到整数后得以保持。
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    x = limit(x, x_min, x_max);
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 判断符号位
int my_sign(float value)
{
    if (value >= 0.0f)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief 自定义浮点数绝对值函数(高效实现)
 * @param x 需要计算绝对值的浮点数
 * @return x的绝对值
 */
float my_fabsf(float x)
{
    union
    {
        float f;
        unsigned int u;
    } converter;

    converter.f = x;
    // 清除符号位(第31位)
    converter.u &= 0x7FFFFFFF;
    return converter.f;
}

/**
 * @description: 将整数转换为指定进制的字符串
 * @return 转换后的字符串
 * @param {int} num
 * @param {char} *str
 * @param {int} radix
 */
char *itoa(int num, char *str, int radix)
{
    char index[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned unum;
    int i = 0, j, k;

    if (radix == 10 && num < 0)
    {
        unum = (unsigned)-num;
        str[i++] = '-';
    }
    else
        unum = (unsigned)num;

    do
    {
        str[i++] = index[unum % (unsigned)radix];
        unum /= radix;

    } while (unum);

    str[i] = '\0';

    if (str[0] == '-')
        k = 1;
    else
        k = 0;

    char temp;
    for (j = k; j <= (i - 1) / 2; j++)
    {
        temp = str[j];
        str[j] = str[i - 1 + k - j];
        str[i - 1 + k - j] = temp;
    }

    return str;
}
