/****************************************************************
 * @file: 	Gimbal_Task.c
 * @author: Shiki
 * @date:	2025.10.5
 * @brief:	2026赛季哨兵云台任务
 * @attention:
 ******************************************************************/
#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can_gimbal.h"
#include "arm_math.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "referee.h"
#include "motor.h"
#include "Vofa_send.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "Shoot_Task.h"

/*************************云台模式枚举体****************************/
typedef enum
{
    AUTOAIM,               // 自瞄模式
    GIMBAL_REMOTE_CONTROL, // 遥控器控制模式
    NAV,                   // 导航自主巡逻模式
    GIMBAL_SAFE            // 失能模式
} gimbal_mode_t;

/*************************云台电机被控变量枚举体****************************/
typedef enum
{
    POSITION,
    SPEED
} gimbal_motor_control_mode_t; // 用于判断在当前云台模式下电机的控制模式，控速度还是位置

/*******************************云台电机类型结构体************************************/
typedef enum
{
    PITCH_MOTOR,    // GM6020
    BIG_YAW_MOTOR,  // DM6006
    SMALL_YAW_MOTOR // GM6020
} gimbal_motor_id_t;

/*******************************pitch导航模式下自主巡航模式************************************/
typedef enum
{
    HIT_ROBOT,  // 击打机器人
    HIT_OUTPOST // 击打前哨站
} pitch_updown_mode_t;

/*******************************pitch导航模式下自主巡航参数结构体************************************/
typedef struct
{
    float min_angle; // 当前pitch巡航模式下的最小角度
    float max_angle; // 当前pitch巡航模式下的最大角度
    float step;      // 当前pitch巡航模式下每次的步进角度
} PitchSwingParams;

/**************************云台控制模式表，在不同的云台模式下设置对应云台控制逻辑***************************/
void gimbal_autoaim_handler(void); // 云台模式处理函数声明，使用函数名给函数指针赋值之前，该函数必须已经被声明
void gimbal_remote_control_handler(void);
void gimbal_nav_handler(void);
void gimbal_safe_handler(void);
struct
{
    gimbal_mode_t mode;    // 底盘模式
    void (*handler)(void); // 不同云台模式对应的处理函数
} gimbal_commands[4] = {
    {AUTOAIM, gimbal_autoaim_handler},
    {GIMBAL_REMOTE_CONTROL, gimbal_remote_control_handler},
    {NAV, gimbal_nav_handler},
    {GIMBAL_SAFE, gimbal_safe_handler}};
/*********************************************************全局变量及常量区***************************************************************/
gimbal_mode_t gimbal_mode = GIMBAL_SAFE; // 设置成全局变量便于调试观察
float big_yaw_angle_err = 0;             // 仅用于调试时候观测大yaw角度偏差，不参与云台控制
float pitch_angle_err = 0;               // 仅用于调试时候观测pitch角度偏差，不参与云台控制

/**
 * @description: 让大yaw每次旋转都在一百八十度以内（最短路径）
 * @return 大yaw目标角度
 * @param {float} 目标角度
 * @param {float} 当前角度
 */
static float Find_Big_Yaw_Min_Angle(float target, float current) // 只有大yaw轴需要，pitch活动范围不会超过180度
{
    float err = target - current;
    if (err > 180)
        target -= 360;
    else if (err < -180)
        target += 360;
    return target;
}

/**
 * @description: 初始化云台pid
 * @return {*}
 */
void Gimbal_Motor_Pid_Init(void)
{
    const static fp32 big_yaw_motor_speed_pid[3] = {BIG_YAW_MOTOR_SPEED_PID_KP, BIG_YAW_MOTOR_SPEED_PID_KI, BIG_YAW_MOTOR_SPEED_PID_KD};
    const static fp32 big_yaw_motor_angle_pid[3] = {BIG_YAW_MOTOR_ANGLE_PID_KP, BIG_YAW_MOTOR_ANGLE_PID_KI, BIG_YAW_MOTOR_ANGLE_PID_KD};
    const static fp32 big_yaw_motor_auto_aim_pid[3] = {BIG_YAW_MOTOR_AUTO_AIM_PID_KP, BIG_YAW_MOTOR_AUTO_AIM_PID_KI, BIG_YAW_MOTOR_AUTO_AIM_PID_KD};

    const static fp32 small_yaw_motor_speed_pid[3] = {SMALL_YAW_MOTOR_SPEED_PID_KP, SMALL_YAW_MOTOR_SPEED_PID_KI, SMALL_YAW_MOTOR_SPEED_PID_KD};
    const static fp32 small_yaw_motor_angle_pid[3] = {SMALL_YAW_MOTOR_ANGLE_PID_KP, SMALL_YAW_MOTOR_ANGLE_PID_KI, SMALL_YAW_MOTOR_ANGLE_PID_KD};
    const static fp32 small_yaw_motor_auto_aim_pid[3] = {SMALL_YAW_MOTOR_AUTO_AIM_PID_KP, SMALL_YAW_MOTOR_AUTO_AIM_PID_KI, SMALL_YAW_MOTOR_AUTO_AIM_PID_KD};

    const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
    const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
    const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};

    PID_init(&DM_big_yaw_motor.speed_pid, PID_POSITION, big_yaw_motor_speed_pid, BIG_YAW_MOTOR_SPEED_PID_MAX_OUT, BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.angle_pid, PID_POSITION, big_yaw_motor_angle_pid, BIG_YAW_MOTOR_ANGLE_PID_MAX_OUT, BIG_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.auto_aim_pid, PID_POSITION, big_yaw_motor_auto_aim_pid, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);

    PID_init(&gimbal_small_yaw_motor.speed_pid, PID_POSITION, big_yaw_motor_speed_pid, BIG_YAW_MOTOR_SPEED_PID_MAX_OUT, BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_small_yaw_motor.angle_pid, PID_POSITION, big_yaw_motor_angle_pid, BIG_YAW_MOTOR_ANGLE_PID_MAX_OUT, BIG_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_small_yaw_motor.auto_aim_pid, PID_POSITION, big_yaw_motor_auto_aim_pid, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
}

/**
 * @description: 更新电机的速度，位置信息
 * @return {*}
 */
void Gimbal_Motor_Data_Update(void)
{
    DM_big_yaw_motor.INS_angle_set_last = DM_big_yaw_motor.INS_angle_set;
    DM_big_yaw_motor.INS_speed_set_last = DM_big_yaw_motor.INS_speed_set;
    DM_big_yaw_motor.INS_speed_now = bmi088_real_data.gyro[2] * RAD_TO_DEGREE; // 单位度每秒
    DM_big_yaw_motor.INS_angle_now = INS_angle_deg[0];

    big_yaw_angle_err = DM_big_yaw_motor.INS_angle_set - DM_big_yaw_motor.INS_angle_now;
}

static gimbal_mode_t Gimbal_Mode_Update()
{
    bool_t check_autoaim = (AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0); // 是否满足自瞄模式，下面以此类推
    bool_t check_rc_ctrl = (rc_ctrl.rc.s[1] == RC_SW_MID);
    bool_t check_nav = (rc_ctrl.rc.s[1] == RC_SW_UP);
    bool_t check_safe = ((rc_ctrl.rc.s[1] == RC_SW_DOWN) || toe_is_error(DBUS_TOE));

    if (check_safe)
    {
        return GIMBAL_SAFE; // 失能模式的优先级最高，需要优先判断
    }
    else if (check_autoaim)
    {
        // return AUTOAIM; // 自瞄的优先级第二高
        return GIMBAL_SAFE; // 失能模式的优先级最高，需要优先判断
    }
    else if (check_nav)
    {
        // return NAV;
        return GIMBAL_SAFE; // 失能模式的优先级最高，需要优先判断
    }
    else if (check_rc_ctrl)
    {
        return GIMBAL_REMOTE_CONTROL; // 遥控的优先级最低
    }
    else
        return GIMBAL_SAFE;
}

/**
 * @description: 检查自瞄目标是否丢失，若丢失则yaw电机原地停两秒防止敌人再次出现，复活赛弃用，联盟赛可开启
 * @return none
 */
void Check_Yaw_LostTarget_Wait()
{
    static uint32_t zero_speed_start_time = 0;
    static uint8_t zero_speed_flag = 0;
    static float auto_aim_yaw_last = 0;
    if (AutoAim_Data_Receive.yaw_aim == 0 && auto_aim_yaw_last != 0)
    {
        zero_speed_start_time = xTaskGetTickCount();
        zero_speed_flag = 1;
    }
    if (zero_speed_flag && (xTaskGetTickCount() - zero_speed_start_time <= pdMS_TO_TICKS(2000)))
    {
        DM_big_yaw_motor.INS_speed_set = 0;
    }
    else
    {
        zero_speed_flag = 0;
    }
    auto_aim_yaw_last = AutoAim_Data_Receive.yaw_aim;
}

/**
 * @description: pitch轴重力补偿，解算出的目标电流值叠加在最后speed pid输出的目标电流上
 * @return pitch轴电机重力补偿的电流值
 */
float Pitch_Gravity_Compensation(void)
{
    return PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE * arm_sin_f32(gimbal_pitch_motor.INS_angle_now / 57.3) + PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE;
}

/**
 * @description: 检查pitch当前有无越过电子限位，不用陀螺仪角度检查是因为陀螺仪会零漂，用电机自身编码器检查更准确
 * @return 无
 */
void Check_Pitch_Electronic_Limit(gimbal_motor_control_mode_t mode)
{
}
/**
 * @description: 用于计算导航模式下pitch轴上下摆动巡航的目标角度
 * @return pitch目标角度
 */
float Pitch_Updown(void)
{
    static float auto_pitch_watch = 0;
    static uint8_t updown_switch_flag = 0;
    uint8_t speed_state = AutoAim_Data_Receive.pitch_speed ? HIT_OUTPOST : HIT_ROBOT;
    const PitchSwingParams swing_params[2] = {{-10.0f, 25.0f, 0.08f}, {-24.0f, -15.0f, 0.04f}};

    if (AutoAim_Data_Receive.yaw_rotate_flag == 0 && yaw_rotate_flag_last != 0) // 保证大回环后先pitch先往下动，防止瞄不到离自己近的车
    {
        updown_switch_flag = 0;
        auto_pitch_watch = gimbal_pitch_motor.INS_angle_now;
    }

    if (updown_switch_flag == 0)
    {
        auto_pitch_watch += swing_params[speed_state].step;
        if (auto_pitch_watch >= swing_params[speed_state].max_angle)
        {
            updown_switch_flag = 1;
            auto_pitch_watch = swing_params[speed_state].max_angle;
        }
    }
    else
    {
        auto_pitch_watch -= swing_params[speed_state].step;
        if (auto_pitch_watch <= swing_params[speed_state].min_angle)
        {
            updown_switch_flag = 0;
            auto_pitch_watch = swing_params[speed_state].min_angle;
        }
    }
    return auto_pitch_watch;
}

/**
 * @description: 根据当前电机的控制模式（控制位置或者速度）选择不同pid计算逻辑最终算出电机目标电流,把pid指针作为参数传入是因为位置模式下会传入不同的pid
 * @return 无
 */
void Calculate_Gimbal_Motor_Target_Current(pid_type_def *gimbal_motor_pid, gimbal_motor_control_mode_t mode, gimbal_motor_id_t motor_id)
{
    switch (motor_id)
    {
    case PITCH_MOTOR: // 6020
        if (mode == POSITION)
        {
        }
        else if (mode == SPEED)
        {
        }
        break;
    case BIG_YAW_MOTOR: // 6006
        if (mode == POSITION)
        {
            PID_calc(gimbal_motor_pid, DM_big_yaw_motor.INS_angle_now, DM_big_yaw_motor.INS_angle_set);
            DM_big_yaw_motor.INS_speed_set = gimbal_motor_pid->out;
            PID_calc(&DM_big_yaw_motor.speed_pid, DM_big_yaw_motor.INS_speed_now, DM_big_yaw_motor.INS_speed_set);
            DM_big_yaw_motor.target_current = DM_big_yaw_motor.speed_pid.out;
        }
        else if (mode == SPEED)
        {
            PID_calc(gimbal_motor_pid, DM_big_yaw_motor.INS_speed_now, DM_big_yaw_motor.INS_speed_set);
            DM_big_yaw_motor.target_current = gimbal_motor_pid->out;
        }
        break;

    default:
        break;
    }
}
/**
 * @description: 失能模式下的控制函数，直接对云台电机电流置零
 * @return 无
 */
void gimbal_safe_handler(void)
{
    DM_big_yaw_motor.target_current = 0;
}

/**
 * @description: 自瞄模式下的控制函数，两个轴电机都是位置控制
 * @return 无
 */
void gimbal_autoaim_handler(void)
{
    gimbal_motor_control_mode_t yaw_mode = POSITION, pitch_mode = POSITION;
}

/**
 * @description: 导航模式下的控制函数，pitch轴电机是位置控制，yaw轴电机是速度控制
 * @return 无
 */
void gimbal_nav_handler(void)
{
    gimbal_motor_control_mode_t yaw_mode = SPEED, pitch_mode = POSITION;
}

/**
 * @description: 遥控模式下的控制函数，速度模式和位置模式根据不同条件切换
 * @return 无
 */
void gimbal_remote_control_handler(void)
{
    static gimbal_motor_control_mode_t yaw_mode, yaw_mode_last, pitch_mode, pitch_mode_last;
    yaw_mode_last = yaw_mode;
    pitch_mode_last = pitch_mode;
    yaw_mode = (abs(rc_ctrl.rc.ch[0]) > 10) ? SPEED : POSITION;
    pitch_mode = (abs(rc_ctrl.rc.ch[1]) > 5) ? SPEED : POSITION;

    if (yaw_mode == SPEED)
    {
        DM_big_yaw_motor.INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * 5.0f * RAD_TO_DEGREE;
        Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.speed_pid, SPEED, BIG_YAW_MOTOR);
    }
    else if (yaw_mode == POSITION)
    {
        if (yaw_mode_last == SPEED)
        {
            DM_big_yaw_motor.INS_angle_set = DM_big_yaw_motor.INS_angle_now;
        }
        DM_big_yaw_motor.INS_angle_set = Find_Big_Yaw_Min_Angle(DM_big_yaw_motor.INS_angle_set, DM_big_yaw_motor.INS_angle_now);
        Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.angle_pid, POSITION, BIG_YAW_MOTOR);
    }
}

/**
 * @brief  根据不同云台模式执行对应云台控制函数，每个控制函数最后会解算出当前pitch轴电机和yaw轴电机的目标电流
 *         注意:1.除了gimbal_safe_handler外每个控制函数都会先选择yaw和pitch的被控变量（位置或者速度），并设定被控变量目标值，然后调用Calculate_Gimbal_Target_Current函数通过pid计算出当前云台电机的目标电流
 *              2.gimbal_safe_handler会直接将云台的yaw轴和pitch轴的电流设置为0
 * @return 无
 */
void choose_gimbal_handler(gimbal_mode_t mode)
{
    int size = sizeof(gimbal_commands) / sizeof(gimbal_commands[0]);
    for (int i = 0; i < size; i++) // 寻找匹配当前模式的控制函数
    {
        if (gimbal_commands[i].mode == mode)
        {
            gimbal_commands[i].handler();
            return;
        }
    }
}
void Check_DM_Auto_Enable()
{
    static uint8_t enable_send_count = 0;
    static uint8_t gimbal_output_last = 0;
    if (Game_Robot_State.power_management_gimbal_output && !gimbal_output_last)
    {
        enable_send_count = 5;
    }
    gimbal_output_last = Game_Robot_State.power_management_gimbal_output;

    while (enable_send_count > 0)
    {
        enable_DM(BIG_YAW_DM6006_TransID, MIT);
        enable_send_count--;
    }
}

void Gimbal_Task(void const *argument)
{
    Gimbal_Motor_Pid_Init();
    vTaskDelay(200);

    gimbal_mode = GIMBAL_SAFE;
    static uint8_t cnt = 1;
    while (1)
    {
        // Check_DM_Auto_Enable();
        Gimbal_Motor_Data_Update();
        gimbal_mode = Gimbal_Mode_Update();
        choose_gimbal_handler(gimbal_mode);

        if (cnt % CAN_TX_DIV5 == 0)
        {
            Allocate_Can_Msg(0,0,0,0, CAN_RC_TO_CHASSIS_SECOND_CMD);
            Allocate_Can_Msg(rc_ctrl.rc.s[1] << 8 | (!toe_is_error(DBUS_TOE)), rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3], rc_ctrl.rc.ch[4], CAN_RC_TO_CHASSIS_FIRST_CMD);
        }

        Ctrl_DM_Motor(0, 0, 0, 0, DM_big_yaw_motor.target_current);
        // Allocate_Can_Buffer(0, shoot_m2006[0].target_current, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current, CAN_SHOOT_CMD);

        cnt == 120 ? cnt = 1 : cnt++; // div等于2,3,4,5的最小公倍数时重置
        // Vofa_Send_Data4((float)DM_pitch_motor_data.INS_angle_set,(float)DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
        vTaskDelay(2);
    }
}
