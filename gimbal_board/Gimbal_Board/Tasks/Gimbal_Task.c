/****************************************************************
 * @file: 	Gimbal_Task.c
 * @author: Shiki
 * @date:	2025.10.5
 * @brief:	2026赛季哨兵云台任务
 * @attention:
 ******************************************************************/
#include "Gimbal_Task.h"
#include "main.h"
#include "cmsis_os.h"

#if USE_EKF == 1
#include "INS_Task_ekf.h"
#else
#include "INS_Task_Mahony.h"
#endif

#include "remote_control.h"
#include "bsp_can_gimbal.h"
#include "bsp_DMIMU.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "referee.h"
#include "motor.h"
#include "Vofa_send.h"
#include "detect_task.h"
#include "user_common_lib.h"

/*************************云台模式枚举体****************************/
typedef enum
{
    NAV_PASS_BUMPY,        // 导航过颠簸路段模式
    AUTOAIM,               // 自瞄模式
    GIMBAL_REMOTE_CONTROL, // 遥控器控制模式
    NAV_SEEK_ENEMY,        // 导航寻敌模式
    GIMBAL_SAFE            // 失能模式
} gimbal_mode_t;
/*************************云台电机被控变量枚举体****************************/
typedef enum
{
    SPEED,
    POSITION_ENC,              // 位置控制模式，使用电机自身编码器作为位置反馈
    POSITION_INS               // 位置控制模式，使用INS_task解算出的角度作为位置反馈
} gimbal_motor_control_mode_t; // 用于判断在当前云台模式下电机的控制模式，控速度还是位置
/*******************************云台电机类型结构体************************************/
typedef enum
{
    PITCH_MOTOR,    // GM6020
    BIG_YAW_MOTOR,  // DM6006
    SMALL_YAW_MOTOR // GM6020
} gimbal_motor_type_t;
/*******************************pitch导航模式下自主巡航枚举体************************************/
typedef enum
{
    HIT_ROBOT,  // 击打机器人
    HIT_OUTPOST // 击打前哨站
} pitch_updown_mode_t;
/*******************************导航模式下自主pitch,小yaw巡航参数结构体************************************/
typedef struct
{
    float min_angle; // 巡航模式下的最小角度
    float max_angle; // 巡航模式下的最大角度
    float step;      // 巡航模式下每次的步进角度
} SwingParams;
/*******************************云台控制结构体************************************/
typedef struct
{
    gimbal_mode_t gimbal_mode;
    gimbal_mode_t gimbal_mode_last;

    gimbal_motor_control_mode_t big_yaw_mode;   // 大yaw电机控制模式
    gimbal_motor_control_mode_t small_yaw_mode; // 小yaw电机控制模式
    gimbal_motor_control_mode_t pitch_mode;     // pitch电机控制模式

    float big_yaw_angle_err;     // 仅用于调试时候观测大yaw角度偏差，不参与云台控制
    float small_yaw_angle_err;   // 仅用于调试时候观测小yaw角度偏差，不参与云台控制
    float pitch_angle_err;       // 仅用于调试时候观测pitch角度偏差，不参与云台控制
} gimbal_control_t;
gimbal_control_t gimbal_control = {
    .gimbal_mode = GIMBAL_SAFE,
    .gimbal_mode_last = GIMBAL_SAFE};
/**************************云台控制模式表，在不同的云台模式下设置对应云台控制逻辑***************************/
// 云台模式处理函数声明，使用函数名给函数指针赋值之前，该函数必须已经被声明
static void gimbal_nav_pass_bumpy_handler(void);
static void gimbal_autoaim_handler(void);
static void gimbal_remote_control_handler(void);
static void gimbal_nav_seek_enemy_handler(void);
static void gimbal_safe_handler(void);

typedef void (*gimbal_handler)(void); // 不同云台模式对应的处理函数
gimbal_handler gimbal_commands[] = {
    [NAV_PASS_BUMPY] = gimbal_nav_pass_bumpy_handler,
    [AUTOAIM] = gimbal_autoaim_handler,
    [GIMBAL_REMOTE_CONTROL] = gimbal_remote_control_handler,
    [NAV_SEEK_ENEMY] = gimbal_nav_seek_enemy_handler,
    [GIMBAL_SAFE] = gimbal_safe_handler};

// 其余函数声明
static void Gimbal_Motor_Control_Init(void);
static void Gimbal_Data_Update(void);
static gimbal_mode_t Gimbal_Mode_Update(void);
static float Find_Yaw_Min_Angle(float target, float current);
static void Check_Big_Yaw_DM_Auto_Enable();
static bool_t Check_Big_Yaw_LostTarget_Wait(gimbal_mode_t last_mode);
static fp32 Set_Big_Yaw_Seek_Enemy_Angle();
static fp32 Set_Small_Yaw_Seek_Enemy_Angle();
static fp32 Pitch_Gravity_Compensation(float pitch_angle_now);
void Check_Pitch_Angle_Limit(gimbal_motor_control_mode_t mode);
static float Set_Pitch_Seek_Enemy_Angle(void);
static void Calculate_Gimbal_Motor_Target_Current(pid_type_def *gimbal_motor_pid, gimbal_motor_control_mode_t mode, gimbal_motor_type_t motor_type, fp32 now, fp32 set);
static void Call_Gimbal_Mode_Handler(gimbal_mode_t mode);
void Gimbal_Task(void const *argument);

/**
 * @description: 初始化云台pid
 * @return {*}
 */
static void Gimbal_Motor_Control_Init(void)
{
    const static fp32 big_yaw_motor_speed_pid[3] = {BIG_YAW_MOTOR_SPEED_PID_KP, BIG_YAW_MOTOR_SPEED_PID_KI, BIG_YAW_MOTOR_SPEED_PID_KD};
    const static fp32 big_yaw_motor_nav_angle_pid[3] = {BIG_YAW_MOTOR_NAV_ANGLE_PID_KP, BIG_YAW_MOTOR_NAV_ANGLE_PID_KI, BIG_YAW_MOTOR_NAV_ANGLE_PID_KD};
    const static fp32 big_yaw_motor_follow_small_yaw_pid[3] = {BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KP, BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KI, BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_KD};
    const static fp32 big_yaw_motor_auto_aim_pid[3] = {BIG_YAW_MOTOR_AUTO_AIM_PID_KP, BIG_YAW_MOTOR_AUTO_AIM_PID_KI, BIG_YAW_MOTOR_AUTO_AIM_PID_KD};
    const static fp32 big_yaw_motor_omni_pid[3] = {BIG_YAW_MOTOR_OMNI_PID_KP, BIG_YAW_MOTOR_OMNI_PID_KI, BIG_YAW_MOTOR_OMNI_PID_KD};

    const static fp32 small_yaw_motor_speed_pid[3] = {SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KP, SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KI, SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KD};
    const static fp32 small_yaw_motor_angle_pid[3] = {SMALL_YAW_MOTOR_ANGLE_PID_KP, SMALL_YAW_MOTOR_ANGLE_PID_KI, SMALL_YAW_MOTOR_ANGLE_PID_KD};
    const static fp32 small_yaw_motor_auto_aim_pid[3] = {SMALL_YAW_MOTOR_AUTO_AIM_PID_KP, SMALL_YAW_MOTOR_AUTO_AIM_PID_KI, SMALL_YAW_MOTOR_AUTO_AIM_PID_KD};

    const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
    const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
    const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};

    PID_init(&DM_big_yaw_motor.speed_pid, PID_POSITION, big_yaw_motor_speed_pid, BIG_YAW_MOTOR_SPEED_PID_MAX_OUT, BIG_YAW_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.nav_angle_pid, PID_POSITION, big_yaw_motor_nav_angle_pid, BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_OUT, BIG_YAW_MOTOR_NAV_ANGLE_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.follow_small_yaw_pid, PID_POSITION, big_yaw_motor_follow_small_yaw_pid, BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_OUT, BIG_YAW_MOTOR_FOLLOW_SMALL_YAW_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.auto_aim_pid, PID_POSITION, big_yaw_motor_auto_aim_pid, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, BIG_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
    PID_init(&DM_big_yaw_motor.omni_pid, PID_POSITION, big_yaw_motor_omni_pid, BIG_YAW_MOTOR_OMNI_PID_MAX_OUT, BIG_YAW_MOTOR_OMNI_PID_MAX_IOUT);
    DM_big_yaw_motor.speed_ff = BIG_YAW_MOTOR_SPEED_FF;
    DM_big_yaw_motor.current_ff = BIG_YAW_MOTOR_CURRENT_FF;

    PID_init(&gimbal_small_yaw_motor.speed_pid, PID_POSITION, small_yaw_motor_speed_pid, SMALL_YAW_MOTOR_SPEED_PID_MAX_OUT, SMALL_YAW_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_small_yaw_motor.angle_pid, PID_POSITION, small_yaw_motor_angle_pid, SMALL_YAW_MOTOR_ANGLE_PID_MAX_OUT, SMALL_YAW_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_small_yaw_motor.auto_aim_pid, PID_POSITION, small_yaw_motor_auto_aim_pid, SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, SMALL_YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
    gimbal_small_yaw_motor.speed_ff = SMALL_YAW_MOTOR_SPEED_FF;
    gimbal_small_yaw_motor.current_ff = SMALL_YAW_MOTOR_CURRENT_FF;

    PID_init(&gimbal_pitch_motor.speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_MOTOR_SPEED_PID_MAX_OUT, PITCH_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_pitch_motor.angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_MOTOR_ANGLE_PID_MAX_OUT, PITCH_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_pitch_motor.auto_aim_pid, PID_POSITION, pitch_motor_auto_aim_pid, PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT, PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);
    gimbal_pitch_motor.speed_ff = PITCH_MOTOR_SPEED_FF;
    gimbal_pitch_motor.current_ff = PITCH_MOTOR_CURRENT_FF;
}

/**
 * @description: 更新电机的速度，位置信息和云台控制相关的变量
 * @return {*}
 */
static void Gimbal_Data_Update(void)
{
    gimbal_small_yaw_motor.INS_angle_set_last = gimbal_small_yaw_motor.INS_angle_set;
    gimbal_small_yaw_motor.ENC_angle_set_last = gimbal_small_yaw_motor.ENC_angle_set;
    gimbal_small_yaw_motor.INS_speed_set_last = gimbal_small_yaw_motor.INS_speed_set;

    gimbal_small_yaw_motor.INS_speed_now = (-arm_sin_f32(INS.Pitch * DEGREE_TO_RAD) * INS.Gyro[AXIS_X] + arm_cos_f32(INS.Pitch * DEGREE_TO_RAD) * INS.Gyro[AXIS_Z]) * RAD_TO_DEGREE; // 单位度每秒
    gimbal_small_yaw_motor.INS_angle_now = INS.Yaw;

    // 处理小yaw电机位置编码器值的跳变问题
    if (SMALL_YAW_MIDDLE_ENC_ZERO < 1500 && motor_measure_small_yaw.ecd > 6692)
        gimbal_small_yaw_motor.ENC_angle_now = (motor_measure_small_yaw.ecd - 8192) * GM6020_ENC_TO_DEGREE;
    else if (SMALL_YAW_MIDDLE_ENC_ZERO > 6692 && motor_measure_small_yaw.ecd < 1500)
        gimbal_small_yaw_motor.ENC_angle_now = (motor_measure_small_yaw.ecd + 8192) * GM6020_ENC_TO_DEGREE;
    else
        gimbal_small_yaw_motor.ENC_angle_now = motor_measure_small_yaw.ecd * GM6020_ENC_TO_DEGREE;

    DM_big_yaw_motor.INS_angle_set_last = DM_big_yaw_motor.INS_angle_set;
    DM_big_yaw_motor.INS_speed_set_last = DM_big_yaw_motor.INS_speed_set;
    DM_big_yaw_motor.INS_speed_now = imu.gyro[2] * RAD_TO_DEGREE;
    DM_big_yaw_motor.INS_angle_now = gimbal_small_yaw_motor.INS_angle_now + (SMALL_YAW_MIDDLE_ENC_ZERO * GM6020_ENC_TO_DEGREE - gimbal_small_yaw_motor.ENC_angle_now);

    gimbal_pitch_motor.INS_angle_set_last = gimbal_pitch_motor.INS_angle_set;
    gimbal_pitch_motor.ENC_angle_set_last = gimbal_pitch_motor.ENC_angle_set;
    gimbal_pitch_motor.INS_speed_set_last = gimbal_pitch_motor.INS_speed_set;
    gimbal_pitch_motor.INS_speed_now = -INS.Gyro[AXIS_Y] * RAD_TO_DEGREE; // 单位度每秒
    gimbal_pitch_motor.INS_angle_now = -INS.Pitch;

    // 处理pitch电机位置编码器值的跳变问题
    if (PITCH_ECD_ANGLE_MAX < 1300 * GM6020_ENC_TO_DEGREE && motor_measure_pitch.ecd > 6800 * GM6020_ENC_TO_DEGREE)
        gimbal_pitch_motor.ENC_angle_now = (motor_measure_pitch.ecd - 8192) * GM6020_ENC_TO_DEGREE;
    else if (PITCH_ECD_ANGLE_MIN > 6800 * GM6020_ENC_TO_DEGREE && motor_measure_pitch.ecd < 1300 * GM6020_ENC_TO_DEGREE)
        gimbal_pitch_motor.ENC_angle_now = (motor_measure_pitch.ecd + 8192) * GM6020_ENC_TO_DEGREE;
    else
        gimbal_pitch_motor.ENC_angle_now = motor_measure_pitch.ecd * GM6020_ENC_TO_DEGREE;

    gimbal_control.gimbal_mode_last = gimbal_control.gimbal_mode;
}

/**
 * @description: 更新云台模式，为选择对应的处理函数做准备
 * @return {*}
 */
static gimbal_mode_t Gimbal_Mode_Update()
{
    bool_t check_nav_pass_bumpy = ((rc_ctrl.rc.s[1] == RC_SW_UP) && (NUC_Data_Receive.pass_bumpy_mode == 1));
    bool_t check_autoaim = (NUC_Data_Receive.small_yaw_aim != 0 || NUC_Data_Receive.pitch_aim != 0); // 是否满足自瞄模式，下面以此类推
    bool_t check_rc_ctrl = (rc_ctrl.rc.s[1] == RC_SW_MID);
    bool_t check_nav_seek_enemy = (rc_ctrl.rc.s[1] == RC_SW_UP);
    bool_t check_safe = ((rc_ctrl.rc.s[1] == RC_SW_DOWN) || toe_is_error(DBUS_TOE) || toe_is_error(DM_IMU_TOE) || toe_is_error(BOARD_ACCEL_TOE) || toe_is_error(BOARD_GYRO_TOE));

    if (check_safe)
    {
        return GIMBAL_SAFE; // 失能模式的优先级最高，需要优先判断
    }
    else if (check_nav_pass_bumpy)
    {
        return NAV_PASS_BUMPY;
    }
    else if (check_autoaim)
    {
        return AUTOAIM; // 自瞄的优先级第三高
    }
    else if (check_nav_seek_enemy)
    {
        return NAV_SEEK_ENEMY;
    }
    else if (check_rc_ctrl)
    {
        return GIMBAL_REMOTE_CONTROL; // 遥控的优先级最低
    }
    else
        return GIMBAL_SAFE;
}

/**
 * @description: 让yaw每次旋转都在一百八十度以内（最短路径）
 * @return yaw目标角度
 * @param {float} 目标角度
 * @param {float} 当前角度
 */
static float Find_Yaw_Min_Angle(float target, float current) // 只有yaw轴需要，pitch活动范围不会超过180度
{

    while (my_fabsf(target - current) > 180.1f)
    {
        target += ((target - current > 0) ? -360 : 360);
    }
    return target;
}

/**
 * @description: 检查大yaw电机是否处于失能状态，如果是，那么使能大yaw轴达妙电机
 * @return {*}
 */
static void Check_Big_Yaw_DM_Auto_Enable()
{
    if (!DM_big_yaw_motor.state)
    {
        uint8_t enable_send_count = 5;
        while (enable_send_count > 0)
        {
            enable_DM(BIG_YAW_DM6006_TransID, MIT);
            vTaskDelay(1);
            enable_send_count--;
        }
    }
}

/**
 * @description: 检查自瞄目标是否丢失，若丢失则大yaw电机原地停两秒防止敌人再次出现，复活赛弃用，联盟赛可开启
 * @return none
 */
static bool_t Check_Big_Yaw_LostTarget_Wait(gimbal_mode_t last_mode)
{
    static uint32_t start_time = 0;
    static bool_t need_wait = FALSE; // 作为函数返回值

    if (last_mode == AUTOAIM)
    {
        need_wait = TRUE;
        start_time = xTaskGetTickCount();
    }

    if (need_wait == TRUE)
    {
        // 检查是否已等待满1.5秒
        if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(1500))
        {
            need_wait = FALSE;
        }
    }

    return need_wait;
}

/**
 * @description: 用于计算导航索敌模式下大yaw轴旋转巡航的目标角度,每隔两秒往一个方向转九十度
 * @return {*}
 */
static fp32 Set_Big_Yaw_Seek_Enemy_Angle()
{
    static uint32_t seek_wait_start = 0;
    static volatile uint8_t seek_wait_flag = 0;
    static float big_yaw_angle_set = 0;

    if (seek_wait_flag == 0)
    {
        big_yaw_angle_set = DM_big_yaw_motor.INS_angle_now + 90.0f;
        seek_wait_start = xTaskGetTickCount();
        seek_wait_flag = 1;
    }
    else if (xTaskGetTickCount() - seek_wait_start >= pdMS_TO_TICKS(2000))
    {
        seek_wait_flag = 0;
    }

    return big_yaw_angle_set;
}

/**
 * @description: 用于计算导航索敌模式下小yaw轴左右摆动巡航的目标角度
 * @return {*}
 */
static fp32 Set_Small_Yaw_Seek_Enemy_Angle()
{
    static float auto_small_yaw_watch = SMALL_YAW_MIDDLE_ENC_ZERO * GM6020_ENC_TO_DEGREE;
    static uint8_t swing_switch_flag = 0;
    const SwingParams swing_params = {SMALL_YAW_NAV_SEEK_ECD_MIN * GM6020_ENC_TO_DEGREE,
                                      SMALL_YAW_NAV_SEEK_ECD_MAX * GM6020_ENC_TO_DEGREE,
                                      SMALL_YAW_NAV_SEEK_STEP};
    if (swing_switch_flag == 0)
    {
        auto_small_yaw_watch += swing_params.step;
        if (auto_small_yaw_watch >= swing_params.max_angle)
        {
            swing_switch_flag = 1;
            auto_small_yaw_watch = swing_params.max_angle;
        }
    }
    else
    {
        auto_small_yaw_watch -= swing_params.step;
        if (auto_small_yaw_watch <= swing_params.min_angle)
        {
            swing_switch_flag = 0;
            auto_small_yaw_watch = swing_params.min_angle;
        }
    }
    return auto_small_yaw_watch;
}

/**
 * @description: pitch轴重力补偿，解算出的目标电流值叠加在最后speed pid输出的目标电流上
 * @return pitch轴电机重力补偿的电流值
 */
static fp32 Pitch_Gravity_Compensation(float pitch_INS_angle_now)
{
    return PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE * arm_cos_f32((pitch_INS_angle_now - PITCH_CENTROID_OFFSET_ANGLE) * DEGREE_TO_RAD);
}

/**
 * @description: 检查pitch当前有无越过电子限位，不用IMU解算出的角度检查是因为角度会有偏差，用电机自身编码器检查更准确
 * @return 无
 */
void Check_Pitch_Angle_Limit(gimbal_motor_control_mode_t mode)
{
    if ((gimbal_pitch_motor.ENC_angle_now < PITCH_ECD_ANGLE_MIN && gimbal_pitch_motor.INS_speed_set < 0) || (gimbal_pitch_motor.ENC_angle_now > PITCH_ECD_ANGLE_MAX && gimbal_pitch_motor.INS_speed_set > 0))
    {
        if (mode == POSITION_INS)
            gimbal_pitch_motor.INS_angle_set = gimbal_pitch_motor.INS_angle_now; // 位置模式下一越过电子限位，pitch轴目标角度就设置为当前角度
        else if (mode == POSITION_ENC)
            gimbal_pitch_motor.ENC_angle_set = gimbal_pitch_motor.ENC_angle_now; // 位置模式下一越过电子限位，pitch轴目标角度就设置为当前角度
        else if (mode == SPEED)
            gimbal_pitch_motor.INS_speed_set = 0; // 速度模式下一越过电子限位，pitch轴目标速度就设置为0
    }
}
/**
 * @description: 用于计算导航索敌模式下pitch轴上下摆动巡航的目标角度
 * @return pitch目标角度
 */
static float Set_Pitch_Seek_Enemy_Angle(void)
{
    static float auto_pitch_watch = 0;
    static uint8_t updown_switch_flag = 0;
    const SwingParams swing_params[] = {[HIT_ROBOT] = {PITCH_NAV_SEEK_ENEMY_ANGLE_MIN, PITCH_NAV_SEEK_ENEMY_ANGLE_MAX, PITCH_NAV_SEEK_ENEMY_STEP},
                                        [HIT_OUTPOST] = {PITCH_NAV_SEEK_OUTPOST_ANGLE_MIN, PITCH_NAV_SEEK_OUTPOST_ANGLE_MAX, PITCH_NAV_SEEK_OUTPOST_STEP}};

    pitch_updown_mode_t seek_mode = (NUC_Data_Receive.pitch_mode == 0) ? HIT_ROBOT : HIT_OUTPOST; // 0打机器人其他打前哨站

    if (updown_switch_flag == 0)
    {
        auto_pitch_watch += swing_params[seek_mode].step;
        if (auto_pitch_watch >= swing_params[seek_mode].max_angle)
        {
            updown_switch_flag = 1;
            auto_pitch_watch = swing_params[seek_mode].max_angle;
        }
    }
    else
    {
        auto_pitch_watch -= swing_params[seek_mode].step;
        if (auto_pitch_watch <= swing_params[seek_mode].min_angle)
        {
            updown_switch_flag = 0;
            auto_pitch_watch = swing_params[seek_mode].min_angle;
        }
    }
    return auto_pitch_watch;
}

/**
 * @description: 根据当前电机的控制模式（控制位置或者速度）选择不同pid计算逻辑最终算出电机目标电流,把pid指针作为参数传入是因为位置模式下会传入不同的pid
 * @return 无
 */
static void Calculate_Gimbal_Motor_Target_Current(pid_type_def *gimbal_motor_pid, gimbal_motor_control_mode_t mode, gimbal_motor_type_t motor_type, fp32 now, fp32 set)
{
    switch (motor_type)
    {
    case PITCH_MOTOR:
    case SMALL_YAW_MOTOR:
    {
        gimbal_motor_t *motor = (motor_type == PITCH_MOTOR) ? &gimbal_pitch_motor : &gimbal_small_yaw_motor;
        
        if (mode == POSITION_INS || mode == POSITION_ENC)
        {
            PID_calc(gimbal_motor_pid, now, set);
            
            float angle_diff = (mode == POSITION_INS) ? 
                (motor->INS_angle_set - motor->INS_angle_set_last) : 
                (motor->ENC_angle_set - motor->ENC_angle_set_last);
                
            motor->INS_speed_set = gimbal_motor_pid->out + motor->speed_ff * angle_diff;
            
            PID_calc(&motor->speed_pid, motor->INS_speed_now, motor->INS_speed_set);
            motor->give_current = motor->speed_pid.out;
        }
        else // SPEED
        {
            PID_calc(gimbal_motor_pid, now, set);
            motor->give_current = gimbal_motor_pid->out;
        }

        // 应用电流前馈
        motor->give_current += motor->current_ff * (motor->INS_speed_set - motor->INS_speed_set_last);

        // 应用重力补偿 (仅Pitch轴)
        if (motor_type == PITCH_MOTOR)
        {
            motor->give_current += Pitch_Gravity_Compensation(motor->INS_angle_now);
        }
        break;
    }
    case BIG_YAW_MOTOR: // 6006
    {
        if (mode == POSITION_INS || mode == POSITION_ENC)
        {
            PID_calc(gimbal_motor_pid, now, set);
            
            // Big Yaw 只有 INS 角度设定
            float angle_diff = DM_big_yaw_motor.INS_angle_set - DM_big_yaw_motor.INS_angle_set_last;
            
            DM_big_yaw_motor.INS_speed_set = gimbal_motor_pid->out + DM_big_yaw_motor.speed_ff * angle_diff;
            PID_calc(&DM_big_yaw_motor.speed_pid, DM_big_yaw_motor.INS_speed_now, DM_big_yaw_motor.INS_speed_set);
            DM_big_yaw_motor.target_current = DM_big_yaw_motor.speed_pid.out;
        }
        else // SPEED
        {
            PID_calc(gimbal_motor_pid, now, set);
            DM_big_yaw_motor.target_current = gimbal_motor_pid->out;
        }    
        // 应用电流前馈
        DM_big_yaw_motor.target_current += DM_big_yaw_motor.current_ff * (DM_big_yaw_motor.INS_speed_set - DM_big_yaw_motor.INS_speed_set_last);
        break;
    }
    default:
        break;
    }
}
/**
 * @description: 失能模式下的控制函数，直接对云台电机电流置零,同时清零pid的iout
 * @return 无
 */
static void gimbal_safe_handler(void)
{
    gimbal_small_yaw_motor.give_current = 0;
    DM_big_yaw_motor.target_current = 0;
    gimbal_pitch_motor.give_current = 0;

    PID_clear(&gimbal_small_yaw_motor.speed_pid);
    PID_clear(&gimbal_small_yaw_motor.angle_pid);
    PID_clear(&gimbal_small_yaw_motor.auto_aim_pid);

    PID_clear(&DM_big_yaw_motor.speed_pid);
    PID_clear(&DM_big_yaw_motor.nav_angle_pid);
    PID_clear(&DM_big_yaw_motor.follow_small_yaw_pid);
    PID_clear(&DM_big_yaw_motor.auto_aim_pid);

    PID_clear(&gimbal_pitch_motor.speed_pid);
    PID_clear(&gimbal_pitch_motor.angle_pid);
    PID_clear(&gimbal_pitch_motor.auto_aim_pid);
}

/**
 * @description: 过颠簸路段模式下的控制函数，三个轴电机都是位置控制,小yaw目标位置为上位机发下来的导航角度，大yaw跟随小yaw
 * @return 无
 */
static void gimbal_nav_pass_bumpy_handler(void)
{
    gimbal_control.big_yaw_mode = POSITION_ENC, gimbal_control.small_yaw_mode = POSITION_INS, gimbal_control.pitch_mode = POSITION_INS;

    gimbal_small_yaw_motor.speed_pid.Kp = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KP;
    gimbal_small_yaw_motor.speed_pid.Ki = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KI;
    gimbal_small_yaw_motor.speed_pid.Kd = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KD;
    gimbal_small_yaw_motor.INS_angle_set = Find_Yaw_Min_Angle(NUC_Data_Receive.pass_bumpy_yaw_angle, gimbal_small_yaw_motor.INS_angle_now);
    Calculate_Gimbal_Motor_Target_Current(&gimbal_small_yaw_motor.angle_pid, POSITION_INS, SMALL_YAW_MOTOR, gimbal_small_yaw_motor.INS_angle_now, gimbal_small_yaw_motor.INS_angle_set);

    fp32 big_yaw_follow_angle = SMALL_YAW_MIDDLE_ENC_ZERO * GM6020_ENC_TO_DEGREE - gimbal_small_yaw_motor.ENC_angle_now;
    Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.follow_small_yaw_pid, POSITION_ENC, BIG_YAW_MOTOR, big_yaw_follow_angle, 0);

    gimbal_pitch_motor.INS_angle_set = 0.0f;
    Check_Pitch_Angle_Limit(POSITION_INS);
    Calculate_Gimbal_Motor_Target_Current(&gimbal_pitch_motor.angle_pid, POSITION_INS, PITCH_MOTOR, gimbal_pitch_motor.INS_angle_now, gimbal_pitch_motor.INS_angle_set);

    gimbal_control.small_yaw_angle_err = gimbal_small_yaw_motor.INS_angle_set - gimbal_small_yaw_motor.INS_angle_now;
    gimbal_control.big_yaw_angle_err = big_yaw_follow_angle;
    gimbal_control.pitch_angle_err = gimbal_pitch_motor.INS_angle_set - gimbal_pitch_motor.INS_angle_now;
}
/**
 * @description: 自瞄模式下的控制函数，三个轴电机都是位置控制
 * @return 无
 */
static void gimbal_autoaim_handler(void)
{
    gimbal_control.big_yaw_mode = POSITION_INS, gimbal_control.small_yaw_mode = POSITION_INS, gimbal_control.pitch_mode = POSITION_INS;

    if (gimbal_control.gimbal_mode_last != AUTOAIM)
    {
        PID_clear(&gimbal_small_yaw_motor.auto_aim_pid);
        PID_clear(&DM_big_yaw_motor.auto_aim_pid);
        PID_clear(&gimbal_pitch_motor.auto_aim_pid);
    }

    // 小yaw控制逻辑
    gimbal_small_yaw_motor.speed_pid.Kp = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KP;
    gimbal_small_yaw_motor.speed_pid.Ki = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KI;
    gimbal_small_yaw_motor.speed_pid.Kd = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KD;
    gimbal_small_yaw_motor.INS_angle_set = Find_Yaw_Min_Angle(NUC_Data_Receive.small_yaw_aim > 0 ? NUC_Data_Receive.small_yaw_aim - 180 : NUC_Data_Receive.small_yaw_aim + 180, gimbal_small_yaw_motor.INS_angle_now);
    Calculate_Gimbal_Motor_Target_Current(&gimbal_small_yaw_motor.auto_aim_pid, POSITION_INS, SMALL_YAW_MOTOR, gimbal_small_yaw_motor.INS_angle_now, gimbal_small_yaw_motor.INS_angle_set);

    // 大yaw控制逻辑
    float big_yaw_autoaim_target_angle = Find_Yaw_Min_Angle(NUC_Data_Receive.big_yaw_aim > 0 ? NUC_Data_Receive.big_yaw_aim - 180 : NUC_Data_Receive.big_yaw_aim + 180, DM_big_yaw_motor.INS_angle_now);
    float big_yaw_autoaim_error = big_yaw_autoaim_target_angle - DM_big_yaw_motor.INS_angle_now;
    static uint8_t big_yaw_lock_flag = 0; //全向感知快速转头

    if (my_fabsf(big_yaw_autoaim_error) > BIG_YAW_AUTOAIM_SLOW_FOLLOW_RANGE)
    {
        big_yaw_lock_flag = 1;
    }

    if (big_yaw_lock_flag)
    {
        // 标志置位期间，持续调用nav_angle_pid快速转头
        DM_big_yaw_motor.INS_angle_set = big_yaw_autoaim_target_angle;
        Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.omni_pid, POSITION_INS, BIG_YAW_MOTOR, DM_big_yaw_motor.INS_angle_now, big_yaw_autoaim_target_angle);

        // 直到误差小于3度，清除标志，恢复正常流程
        if (my_fabsf(big_yaw_autoaim_error) < 3.0f)
        {
            big_yaw_lock_flag = 0;
        }
    }
    else
    {
        if (my_fabsf(big_yaw_autoaim_error) < BIG_YAW_AUTOAIM_STOP_RANGE)
        {
            DM_big_yaw_motor.INS_angle_set = DM_big_yaw_motor.INS_angle_now; // 当小yaw接近中心，大yaw不需要转动
            Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.nav_angle_pid, POSITION_INS, BIG_YAW_MOTOR, DM_big_yaw_motor.INS_angle_now, DM_big_yaw_motor.INS_angle_set);
        }
        else if (my_fabsf(big_yaw_autoaim_error) < BIG_YAW_AUTOAIM_SLOW_FOLLOW_RANGE)
        {
            DM_big_yaw_motor.INS_angle_set = ramp_control(DM_big_yaw_motor.INS_angle_now, big_yaw_autoaim_target_angle, 0.3f);
            Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.auto_aim_pid, POSITION_INS, BIG_YAW_MOTOR, DM_big_yaw_motor.INS_angle_now, DM_big_yaw_motor.INS_angle_set);
        }
    }

    // pitch控制逻辑
    gimbal_pitch_motor.INS_angle_set = NUC_Data_Receive.pitch_aim;
    Check_Pitch_Angle_Limit(POSITION_INS);
    Calculate_Gimbal_Motor_Target_Current(&gimbal_pitch_motor.auto_aim_pid, POSITION_INS, PITCH_MOTOR, gimbal_pitch_motor.INS_angle_now, gimbal_pitch_motor.INS_angle_set);

    gimbal_control.small_yaw_angle_err = gimbal_small_yaw_motor.INS_angle_set - gimbal_small_yaw_motor.INS_angle_now;
    gimbal_control.big_yaw_angle_err = DM_big_yaw_motor.INS_angle_set - DM_big_yaw_motor.INS_angle_now;
    gimbal_control.pitch_angle_err = gimbal_pitch_motor.INS_angle_set - gimbal_pitch_motor.INS_angle_now;
}

/**
 * @description: 导航索敌模式下的控制函数，三个轴电机都是位置控制
 * @return 无
 */
static void gimbal_nav_seek_enemy_handler(void)
{
    gimbal_control.big_yaw_mode = POSITION_INS, gimbal_control.small_yaw_mode = POSITION_ENC, gimbal_control.pitch_mode = POSITION_INS;

    // 小yaw控制逻辑
    gimbal_small_yaw_motor.speed_pid.Kp = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KP;
    gimbal_small_yaw_motor.speed_pid.Ki = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KI;
    gimbal_small_yaw_motor.speed_pid.Kd = SMALL_YAW_MOTOR_NORMAL_SPEED_PID_KD;
    gimbal_small_yaw_motor.ENC_angle_set = Set_Small_Yaw_Seek_Enemy_Angle();
    Calculate_Gimbal_Motor_Target_Current(&gimbal_small_yaw_motor.angle_pid, POSITION_ENC, SMALL_YAW_MOTOR, gimbal_small_yaw_motor.ENC_angle_now, gimbal_small_yaw_motor.ENC_angle_set);

    // 大yaw控制逻辑
    if (Check_Big_Yaw_LostTarget_Wait(gimbal_control.gimbal_mode_last) == TRUE) // 自瞄刚丢失目标
    {
        DM_big_yaw_motor.INS_angle_set = DM_big_yaw_motor.INS_angle_now; 
    }
    else // 正常索敌
    {
        DM_big_yaw_motor.INS_angle_set = Set_Big_Yaw_Seek_Enemy_Angle();
    }
    DM_big_yaw_motor.INS_angle_set = Find_Yaw_Min_Angle(DM_big_yaw_motor.INS_angle_set, DM_big_yaw_motor.INS_angle_now); // 让电机选择最近的路径进行转动
    Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.nav_angle_pid, POSITION_INS, BIG_YAW_MOTOR, DM_big_yaw_motor.INS_angle_now, DM_big_yaw_motor.INS_angle_set);

    // pitch控制逻辑
    gimbal_pitch_motor.INS_angle_set = Set_Pitch_Seek_Enemy_Angle();
    Check_Pitch_Angle_Limit(POSITION_INS);
    Calculate_Gimbal_Motor_Target_Current(&gimbal_pitch_motor.angle_pid, POSITION_INS, PITCH_MOTOR, gimbal_pitch_motor.INS_angle_now, gimbal_pitch_motor.INS_angle_set);

    gimbal_control.small_yaw_angle_err = gimbal_small_yaw_motor.ENC_angle_set - gimbal_small_yaw_motor.ENC_angle_now;
    gimbal_control.big_yaw_angle_err = DM_big_yaw_motor.INS_angle_set - DM_big_yaw_motor.INS_angle_now;
    gimbal_control.pitch_angle_err = gimbal_pitch_motor.INS_angle_set - gimbal_pitch_motor.INS_angle_now;
}

/**
 * @description: 遥控模式下的控制函数，速度模式和位置模式根据不同条件切换
 * @return 无
 */
static void gimbal_remote_control_handler(void)
{
    gimbal_motor_control_mode_t small_yaw_mode_last, pitch_mode_last;
    small_yaw_mode_last = gimbal_control.small_yaw_mode;
    pitch_mode_last = gimbal_control.pitch_mode;

    gimbal_control.small_yaw_mode = (my_fabsf(rc_ctrl.rc.ch[0]) > 10) ? SPEED : POSITION_INS;
    gimbal_control.pitch_mode = (my_fabsf(rc_ctrl.rc.ch[1]) > 10) ? SPEED : POSITION_INS;
    gimbal_control.big_yaw_mode = POSITION_ENC;

    /*********小yaw轴电机控制逻辑********/
    gimbal_small_yaw_motor.speed_pid.Kp = SMALL_YAW_MOTOR_RC_SPEED_PID_KP;
    gimbal_small_yaw_motor.speed_pid.Ki = SMALL_YAW_MOTOR_RC_SPEED_PID_KI;
    gimbal_small_yaw_motor.speed_pid.Kd = SMALL_YAW_MOTOR_RC_SPEED_PID_KD;
    if (gimbal_control.small_yaw_mode == SPEED)
    {
        gimbal_small_yaw_motor.INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * REMOTE_CONTROL_YAW_MAX_SPEED * RAD_TO_DEGREE;
        Calculate_Gimbal_Motor_Target_Current(&gimbal_small_yaw_motor.speed_pid, SPEED, SMALL_YAW_MOTOR, gimbal_small_yaw_motor.INS_speed_now, gimbal_small_yaw_motor.INS_speed_set);
    }
    else if (gimbal_control.small_yaw_mode == POSITION_INS)
    {
        if (small_yaw_mode_last == SPEED || gimbal_control.gimbal_mode_last != GIMBAL_REMOTE_CONTROL)
        {
            gimbal_small_yaw_motor.INS_angle_set = gimbal_small_yaw_motor.INS_angle_now;
        }
        gimbal_small_yaw_motor.INS_angle_set = Find_Yaw_Min_Angle(gimbal_small_yaw_motor.INS_angle_set, gimbal_small_yaw_motor.INS_angle_now);
        Calculate_Gimbal_Motor_Target_Current(&gimbal_small_yaw_motor.angle_pid, POSITION_INS, SMALL_YAW_MOTOR, gimbal_small_yaw_motor.INS_angle_now, gimbal_small_yaw_motor.INS_angle_set);
    }
    /*********大yaw轴电机控制逻辑********/
    fp32 big_yaw_follow_angle = SMALL_YAW_MIDDLE_ENC_ZERO * GM6020_ENC_TO_DEGREE - gimbal_small_yaw_motor.ENC_angle_now;
    Calculate_Gimbal_Motor_Target_Current(&DM_big_yaw_motor.follow_small_yaw_pid, POSITION_ENC, BIG_YAW_MOTOR, big_yaw_follow_angle, 0);

    /*********pitch轴电机控制逻辑********/
    if (gimbal_control.pitch_mode == SPEED)
    {
        gimbal_pitch_motor.INS_speed_set = -(float)rc_ctrl.rc.ch[1] / 660.0f * REMOTE_CONTROL_PITCH_MAX_SPEED * RAD_TO_DEGREE;
        Check_Pitch_Angle_Limit(SPEED);
        Calculate_Gimbal_Motor_Target_Current(&gimbal_pitch_motor.speed_pid, SPEED, PITCH_MOTOR, gimbal_pitch_motor.INS_speed_now, gimbal_pitch_motor.INS_speed_set);
    }
    else if (gimbal_control.pitch_mode == POSITION_INS)
    {
        if (pitch_mode_last == SPEED || gimbal_control.gimbal_mode_last != GIMBAL_REMOTE_CONTROL)
        {
            gimbal_pitch_motor.INS_angle_set = gimbal_pitch_motor.INS_angle_now;
        }
        Check_Pitch_Angle_Limit(POSITION_INS);
        Calculate_Gimbal_Motor_Target_Current(&gimbal_pitch_motor.angle_pid, POSITION_INS, PITCH_MOTOR, gimbal_pitch_motor.INS_angle_now, gimbal_pitch_motor.INS_angle_set);
    }

    gimbal_control.small_yaw_angle_err = gimbal_small_yaw_motor.INS_angle_set - gimbal_small_yaw_motor.INS_angle_now;
    gimbal_control.big_yaw_angle_err = big_yaw_follow_angle;
    gimbal_control.pitch_angle_err = gimbal_pitch_motor.INS_angle_set - gimbal_pitch_motor.INS_angle_now;
}

/**
 * @brief  根据不同云台模式执行对应云台控制函数，每个控制函数最后会解算出当前pitch轴电机和yaw轴电机的目标电流
 *         注意:1.除了gimbal_safe_handler外每个控制函数都会先选择yaw和pitch的被控变量（位置或者速度），并设定被控变量目标值，然后调用Calculate_Gimbal_Target_Current函数通过pid计算出当前云台电机的目标电流
 *              2.gimbal_safe_handler会直接将云台的yaw轴和pitch轴的电流设置为0
 * @return 无
 */
static void Call_Gimbal_Mode_Handler(gimbal_mode_t mode)
{
    gimbal_commands[mode]();
}

void Gimbal_Task(void const *argument)
{
    // 等待INS_Task完成第一次while(1)循环再启动gimbal_task，否则云台在非失能模式下会失控
    if (xSemaphoreTake(ins_init_done_semaphore, portMAX_DELAY) != pdPASS)
    {
        Error_Handler();
    }

    Gimbal_Motor_Control_Init();

    while (1)
    {
        static uint8_t cnt = 1;

        Check_Big_Yaw_DM_Auto_Enable();
        Gimbal_Data_Update();
        gimbal_control.gimbal_mode = Gimbal_Mode_Update();
        Call_Gimbal_Mode_Handler(gimbal_control.gimbal_mode);

//             Ctrl_DM_Motor(0, 0, 0, 0, 0);
       Ctrl_DM_Motor(0, 0, 0, 0, DM_big_yaw_motor.target_current);

        Allocate_Can_Msg(gimbal_small_yaw_motor.give_current, gimbal_pitch_motor.give_current, 0, 0, CAN_SMALL_YAW_AND_PITCH_CMD);
//	    Allocate_Can_Msg(500, gimbal_pitch_motor.give_current, 0, 0, CAN_SMALL_YAW_AND_PITCH_CMD);
        //		Allocate_Can_Msg(0, 0, 0, 0, CAN_SMALL_YAW_AND_PITCH_CMD);

        // Vofa_Send_Data4(arm_cos_f32((gimbal_pitch_motor.INS_angle_now - PITCH_CENTROID_OFFSET_ANGLE) * DEGREE_TO_RAD), gimbal_pitch_motor.give_current, motor_measure_pitch.given_current, 0);

        cnt == 120 ? cnt = 1 : cnt++; // div等于2,3,4,5的最小公倍数时重置
        vTaskDelay(2);
    }
}
