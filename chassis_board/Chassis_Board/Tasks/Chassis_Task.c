/****************************************************************
 * @file: 	Chassis_Task.c
 * @author: Shiki
 * @date:	2025.9.26
 * @brief:	2026赛季哨兵舵轮底盘任务
 * @attention:
 ******************************************************************/
#include "Chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can_chassis.h"
#include "bsp_cap.h"
#include "arm_math.h"
#include "motor.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "INS_Task.h"
#include "Vofa_send.h"
#include "Can_Send_Task.h"
/*************************底盘模式枚举体****************************/
typedef enum
{
	FOLLOW_GIMBAL, // 底盘跟随云台移动模式
	ROTATE,		   // 小陀螺移动模式
	CHASSIS_SAFE   // 失能模式
} chassis_mode_t;
/*******************************************************************/

/************************底盘控制模式表，在不同的底盘模式下设置对应底盘控制逻辑***************************/
void chassis_follow_gimbal_handler(void); // 底盘模式处理函数声明，使用函数名给函数指针赋值之前，该函数必须已经被声明
void chassis_rotate_handler(void);
void chassis_safe_handler(void);

struct
{
	chassis_mode_t mode;   // 底盘模式
	void (*handler)(void); // 不同底盘模式对应的处理函数
} chassis_commands[] = {
	{FOLLOW_GIMBAL, chassis_follow_gimbal_handler},
	{ROTATE, chassis_rotate_handler},
	{CHASSIS_SAFE, chassis_safe_handler}};
/*******************************************************************/
typedef struct // 底盘坐标系目标速度结构体
{
	fp32 vx;
	fp32 vy;
	fp32 wz;
} chassis_speed_t;

typedef struct // 底盘控制参数结构体
{
	chassis_speed_t chassis_target_speed;

	fp32 chassis_follow_gimbal_angle; // 底盘跟随云台模式下云台大yaw轴与底盘当前零点的夹角。单位：度
	fp32 chassis_gimbal_angle_rad;	  // 底盘默认零点（底盘yaw轴正方向）与云台大yaw轴的夹角，用于将云台坐标系下的vx,vy转换到底盘坐标系下。单位：弧度
	fp32 chassis_power_limit;
	fp32 init_chassis_power;
	fp32 current_wz;
	// bool_t chassis_follow_gimbal_zerochange;

	pid_type_def chassis_follow_gimbal_pid;
} chassis_control_t;

/************************全局变量及常量区*****************************/
chassis_control_t chassis_control = {0};
chassis_speed_t chassis_target_speed = {0};
chassis_mode_t chassis_mode = CHASSIS_SAFE; // 因为debug所以开成全局了
fp32 temp_move_angle_set[4] = {0};
bool_t is_stop, is_stop_last = TRUE; // is_stop_last在定义时一定要初始化为TRUE,保证在进入非失能模式的时候先初始化轮电机旋转方向
/********************************其余底盘函数声明***********************************/
static void Chassis_Motor_Pid_Init(void);
static void Chassis_Data_Update(void);
static chassis_mode_t Chassis_Mode_Update();
static fp32 Find_Chassis_Follow_Gimbal_ZERO(fp32 current_yaw_angle);
static fp32 Find_Steer_Min_Angle(fp32 target_angle, fp32 current_angle);
static void Set_Chassis_VxVy(fp32 yaw_nearest_zero_rad, fp32 *chassis_vx, fp32 *chassis_vy);
static fp32 Set_FollowGimbal_Wz(fp32 follow_gimbal_angle, fp32 *wz);
static fp32 Set_Rotate_Wz(fp32 *wz);
static void Find_Chassis_Mode_Handler(chassis_mode_t mode);
static void Chassis_Vector_To_Steer_Angle(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, chassis_mode_t mode);
static void Chassis_Vector_To_Wheel_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, chassis_mode_t mode);
static void Chassis_Motor_Current_Set(chassis_mode_t mode);
void Chassis_Task(void const *argument);

/**
 * @description: 初始化底盘pid
 * @return {*}
 */
static void Chassis_Motor_Pid_Init(void)
{
	const static fp32 wheel_motor_speed_pid[3] = {WHEEL_MOTOR_SPEED_PID_KP, WHEEL_MOTOR_SPEED_PID_KI, WHEEL_MOTOR_SPEED_PID_KD};
	const static fp32 steer_motor_speed_pid[3] = {STEER_MOTOR_SPEED_PID_KP, STEER_MOTOR_SPEED_PID_KI, STEER_MOTOR_SPEED_PID_KD};
	const static fp32 steer_motor_angle_pid[3] = {STEER_MOTOR_ANGLE_PID_KP, STEER_MOTOR_ANGLE_PID_KI, STEER_MOTOR_ANGLE_PID_KD};
	const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_init(&chassis_wheel_motor[i].speed_pid, PID_POSITION, wheel_motor_speed_pid, WHEEL_MOTOR_SPEED_PID_MAX_OUT, WHEEL_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init(&chassis_steer_motor[i].speed_pid, PID_POSITION, steer_motor_speed_pid, STEER_MOTOR_SPEED_PID_MAX_OUT, STEER_MOTOR_SPEED_PID_MAX_IOUT);
		PID_init(&chassis_steer_motor[i].angle_pid, PID_POSITION, steer_motor_angle_pid, STEER_MOTOR_ANGLE_PID_MAX_OUT, STEER_MOTOR_ANGLE_PID_MAX_IOUT);
	}
	PID_init(&chassis_control.chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
}

/**
 * @description: 1.更新底盘当前角速度用于补偿陀螺状态下的底盘跟随云台夹角 2.根据电机反馈值更新电机的速度，位置信息，并且在此函数内完成大yaw轴达秒电机位置值的归一化.
 * @return {*}
 */
static void Chassis_Data_Update(void)
{
	chassis_control.current_wz = bmi088_real_data.gyro[2] * RAD_TO_DEGREE;
	/********************更新大yaw电机位置信息********************/
	while (DM_big_yaw_motor.p_int >= 16384)
	{
		// 将DM6006（大yaw电机）角度归一化到0-360度
		DM_big_yaw_motor.p_int -= 16384;
	}
	DM_big_yaw_motor.pos = DM_big_yaw_motor.p_int * DM6006_ENC_TO_DEGREE;

	/********************更新舵电机位置速度信息和轮电机速度信息********************/
	const static uint16_t steer_motor_ENC_offset[4] = {STEER_MOROR1_ENC_OFFSET, STEER_MOROR2_ENC_OFFSET, STEER_MOROR3_ENC_OFFSET, STEER_MOROR4_ENC_OFFSET};
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].speed_now = motor_measure_wheel[i].speed_rpm;
		chassis_steer_motor[i].speed_now = motor_measure_steer[i].speed_rpm;
		// 将舵电机GM6020的角度归一化到0-360度，舵轮转一圈每个位置对应唯一一个角度,并保证每个舵轮朝向相同时舵电机角度也相同，加8192是考虑motor_measure_steer[i].ecd - steer_motor_ENC_offset[i]<0的情况
		chassis_steer_motor[i].angle_now = ((motor_measure_steer[i].ecd - steer_motor_ENC_offset[i] + 8192) % 8192) * GM6020_ENC_TO_DEGREE;
		chassis_steer_motor[i].angle_set_last = chassis_steer_motor[i].angle_set;
	}
}

/**
 * @description: 更新底盘模式
 * @return 底盘当前模式
 */
static chassis_mode_t Chassis_Mode_Update()
{
	bool_t rc_ctrl_follow_gimbal = ((chassis_rc_ctrl.s[1] == RC_SW_MID) && (chassis_rc_ctrl.ch[4] < 500) && (chassis_rc_ctrl.ch[4] > -500)); // 是否满足遥控器控制时底盘跟随云台模式，下面以此类推
	bool_t rc_ctrl_rotate = ((chassis_rc_ctrl.s[1] == RC_SW_MID) && !rc_ctrl_follow_gimbal);
	bool_t rc_ctrl_safe = ((chassis_rc_ctrl.s[1] == RC_SW_DOWN) || toe_is_error(RC_FIRST_TOE));
	// bool_t nav_follow_gimbal = ((AutoAim_Data_Receive.rotate == 0) && (chassis_rc_ctrl.s[1] == RC_SW_UP)); // 是否满足导航模式下底盘跟随云台模式，下面以此类推
	// bool_t nav_rotate = ((AutoAim_Data_Receive.rotate != 0) && (chassis_rc_ctrl.s[1] == RC_SW_UP));
	// bool_t nav_safe = ((Game_Status.game_progress != 4) && (chassis_rc_ctrl.s[1] == RC_SW_UP));

	if (rc_ctrl_safe)
	{
		return CHASSIS_SAFE; // 失能模式的优先级最高，需要优先判断
	}
	else if (rc_ctrl_rotate)
	{
		return ROTATE;
	}
	else if (rc_ctrl_follow_gimbal)
	{
		return FOLLOW_GIMBAL;
	}
	else
		return CHASSIS_SAFE;
}

/**
 * @description: 从其他模式切换到底盘跟随云台时调用，寻找距离大yaw最近的底盘零点
 * @return 距离大yaw最近的底盘零点
 * @param {fp32} current_yaw_angle 大yaw当前角度（单位:度）
 */
static fp32 Find_Chassis_Follow_Gimbal_ZERO(fp32 current_yaw_angle)
{
	const static fp32 zero_arr[4] = {
		CHASSIS_FOLLOW_GIMBAL_BACK_ZERO,
		CHASSIS_FOLLOW_GIMBAL_RIGHT_ZERO,
		CHASSIS_FOLLOW_GIMBAL_LEFT_ZERO,
		CHASSIS_FOLLOW_GIMBAL_ZERO};

	for (uint8_t i = 0; i < sizeof(zero_arr) / sizeof(fp32); i++)
	{
		if (my_fabsf(current_yaw_angle - zero_arr[i]) >= 315.0f || my_fabsf(current_yaw_angle - zero_arr[i]) <= 45.0f)
			return zero_arr[i];
	}
}

/**
 * @description:根据初步算出的舵电机目标角度和当前角度计算出使舵电机旋转路径最短的最总目标角度
 * @return {*}
 * @param {fp32} *target_angle 目标角度
 * @param {fp32} current_angle 当前角度
 */
static fp32 Find_Steer_Min_Angle(fp32 target_angle, fp32 current_angle)
{

	while (my_fabsf(target_angle - current_angle) > 90.0f)
	{
		if (target_angle - current_angle > 90.0f)
			target_angle -= 180.0f;
		else if (target_angle - current_angle < -90.0f)
			target_angle += 180.0f;
	}
	return target_angle;
}
/**
 * @brief  设置云台坐标系的xy轴目标速度并将其变换到底盘坐标系下，在小陀螺模式和底盘跟随云台模式下通用
 */
static void Set_Chassis_VxVy(fp32 yaw_nearest_zero_rad, fp32 *chassis_vx, fp32 *chassis_vy)
{
	static fp32 gimbal_vx, gimbal_vy; // 加static修饰，不然每次调用ramp函数gimbal_vx, gimbal_vy都会归零
	fp32 sin_yaw = arm_sin_f32(yaw_nearest_zero_rad);
	fp32 cos_yaw = arm_cos_f32(yaw_nearest_zero_rad);
	// todo
	if (chassis_rc_ctrl.s[1] == RC_SW_MID) // 遥控器控制模式
	{
		gimbal_vx = ramp_control(gimbal_vx, chassis_rc_ctrl.ch[3] * 10, 0.1f);
		gimbal_vy = ramp_control(gimbal_vy, -chassis_rc_ctrl.ch[2] * 10, 0.1f);
	}
	else // 导航模式（进入Set_FollowGimbal_VxVy函数时不是遥控器控制模式就是导航模式，所以不用再判断一次是否为导航模式）
	{
		gimbal_vx = 0;
		gimbal_vy = 0;
		// vx = -ramp_control(vx, (float)AutoAim_Data_Receive.vy * NAV_SPEED_FAST, 0.9f);
		// vy = ramp_control(vy,  (float)AutoAim_Data_Receive.vx * NAV_SPEED_FAST, 0.9f);
	}
	*chassis_vx = cos_yaw * gimbal_vx + sin_yaw * gimbal_vy;
	*chassis_vy = sin_yaw * gimbal_vx - cos_yaw * gimbal_vy;
}

/**
 * @brief  设置底盘跟随云台时的底盘角速度，只在chassis_follow_gimbal_handler中调用
 */
static fp32 Set_FollowGimbal_Wz(fp32 follow_gimbal_angle, fp32 *wz)
{
	PID_calc(&chassis_control.chassis_follow_gimbal_pid, follow_gimbal_angle, 0);
	*wz = -chassis_control.chassis_follow_gimbal_pid.out;
	return *wz;
}

/**
 * @brief  设置小陀螺时的底盘角速度，只在chassis_rotate_handler中调用
 */
static fp32 Set_Rotate_Wz(fp32 *wz)
{
	const uint16_t RADS_TO_RPM = (uint16_t)((MOTOR_DISTANCE_TO_CENTER * 60 / (2 * PI * WHEEL_RADIUS)) * MOTOR_REDUCTION_RATIO);
	if (chassis_rc_ctrl.s[1] == RC_SW_MID && chassis_rc_ctrl.ch[4] <= -500)
		*wz = ramp_control(*wz, ROTATE_WZ_MAX * RADS_TO_RPM, 0.2f);
	else if (chassis_rc_ctrl.s[1] == RC_SW_MID && chassis_rc_ctrl.ch[4] >= 500)
		*wz = ramp_control(*wz, ROTATE_WZ_MIN * RADS_TO_RPM, 0.8f);
	else
		*wz = 0;
	// else // 导航模式下的小陀螺角速度设置
	// {
	// 	if (AutoAim_Data_Receive.uphill_flag == 2)
	// 		*wz = 0;

	// 	else if (health_state == HEALTH_HURT)
	// 	{
	// 		*wz = -(float)AutoAim_Data_Receive.rotate;
	// 	}
	// 	else
	// 	{
	// 		*wz = -(float)AutoAim_Data_Receive.rotate * ROTATE_WEAK;
	// 	}
	// }
	return *wz;
}

/**
 * @brief  跟随云台模式下的控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 */
static void chassis_follow_gimbal_handler(void)
{
	static fp32 chassis_follow_gimbal_zero_actual = CHASSIS_FOLLOW_GIMBAL_ZERO;

	chassis_follow_gimbal_zero_actual = Find_Chassis_Follow_Gimbal_ZERO(DM_big_yaw_motor.pos);
	chassis_control.chassis_follow_gimbal_angle = Limit_To_180(chassis_follow_gimbal_zero_actual - DM_big_yaw_motor.pos);
	chassis_control.chassis_gimbal_angle_rad = Limit_To_180(CHASSIS_FOLLOW_GIMBAL_ZERO - DM_big_yaw_motor.pos) * DEGREE_TO_RAD;

	Set_FollowGimbal_Wz(chassis_control.chassis_follow_gimbal_angle, &chassis_target_speed.wz);
	Set_Chassis_VxVy(chassis_control.chassis_gimbal_angle_rad, &chassis_target_speed.vx, &chassis_target_speed.vy);
}

/**
 * @brief  小陀螺模式下的控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 */
static void chassis_rotate_handler(void)
{
	fp32 rotate_ff = chassis_target_speed.wz > 13000 ? ROTATE_MOVE_FF_HIGH_SPEED : ROTATE_MOVE_FF_LOW_SPEED;
	chassis_control.chassis_follow_gimbal_angle = Limit_To_180(CHASSIS_FOLLOW_GIMBAL_ZERO - DM_big_yaw_motor.pos + rotate_ff * chassis_control.current_wz);
	chassis_control.chassis_gimbal_angle_rad = chassis_control.chassis_follow_gimbal_angle * DEGREE_TO_RAD; // 小陀螺模式下默认零点（即底盘坐标系x轴正方向）距离yaw轴的弧度差

	Set_Rotate_Wz(&chassis_target_speed.wz);
	Set_Chassis_VxVy(chassis_control.chassis_gimbal_angle_rad, &chassis_target_speed.vx, &chassis_target_speed.vy);
}

/**
 * @brief  失能模式下的控制函数，对底盘电机电流置零,默认底盘进入停止状态
 */
static void chassis_safe_handler(void)
{
	is_stop = TRUE, is_stop_last = TRUE;
	chassis_control.chassis_follow_gimbal_pid.Iout = 0; // 清零iout，防止积分饱和
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_steer_motor[i].give_current = 0;
		chassis_wheel_motor[i].give_current = 0;

		chassis_steer_motor[i].speed_pid.Iout = 0; // 清零iout，防止积分饱和
		chassis_steer_motor[i].angle_pid.Iout = 0;
		chassis_wheel_motor[i].speed_pid.Iout = 0;
	}
}

/**
 * @brief  根据不同底盘模式执行对应底盘控制函数，在控制函数内将云台坐标系下的目标速度转化到底盘坐标系下
 *         注意：1.在小陀螺模式和底盘跟随模式下，底盘xy轴目标速度的赋值逻辑相同，wz角速度目标值赋值逻辑不同
 * 				 2.失能模式下，直接对八个舵轮电机电流值赋0，在task的while(1)中后续调用Chassis_Vector_To_Steer_Angle函数，Chassis_Vector_To_Wheel_Speed函数，Chassis_Motor_Current_Set函数时会直接返回
 *         综上：在小陀螺模式和底盘跟随模式下计算xy轴目标速度时调用同一函数 Set_Chassis_VxVy（），计算目标角速度时根据底盘当前模式调用不同函数
 */
static void Find_Chassis_Mode_Handler(chassis_mode_t mode)
{
	uint8_t size = sizeof(chassis_commands) / sizeof(chassis_commands[0]);
	for (int i = 0; i < size; i++) // 寻找匹配当前模式的控制函数
	{
		if (chassis_commands[i].mode == mode)
		{
			chassis_commands[i].handler();
			return;
		}
	}
}
/**
 * @description: 通过底盘坐标系下的目标速度解算出速度四个舵电机的目标角度,并判断轮电机是正转还是反转
 * @return {*}
 * @param {fp32} vx_set 底盘坐标系下x轴目标速度
 * @param {fp32} vy_set 底盘坐标系下y轴目标速度
 * @param {fp32} wz_set 底盘坐标系下z轴目标角速度
 * @param {chassis_mode_t} mode
 */
static void Chassis_Vector_To_Steer_Angle(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, chassis_mode_t mode)
{
	if (mode == CHASSIS_SAFE)
	{
		return;
	}

	is_stop = ((my_fabsf(vx_set) <= 10.0f && my_fabsf(vy_set) <= 10.0f && my_fabsf(wz_set) <= 300.0f) ? TRUE : FALSE); // 判断当前是否停车

	if (is_stop)
	{
		static uint32_t stop_start_time;
		static uint8_t stop_flag;

		if (!is_stop_last) // 如果刚进入停车状态
		{
			stop_flag = 1;
			stop_start_time = xTaskGetTickCount();
		}

		if (stop_flag && (xTaskGetTickCount() - stop_start_time <= pdMS_TO_TICKS(1000))) // 如果刚进入停车状态不到一秒
		{
			for (int i = 0; i < 4; i++)
				chassis_steer_motor[i].angle_set = chassis_steer_motor[i].angle_now; // 将舵电机目标角度设置为当前角度,防止急停后直接进入自锁模式翻车
		}
		else
		{
			fp32 temp_stop_angle_set[4];
			stop_flag = 0;
			for (int i = 0; i < 4; i++) // 自锁模式，相邻舵轮之间角度差为90度
			{
				temp_stop_angle_set[i] = 45.0f + i * 90.0f;
				// 选择路程最短的方向旋转，保证舵轮每次旋转的角度小于等于90度
				chassis_steer_motor[i].angle_set = Find_Steer_Min_Angle(temp_stop_angle_set[i], chassis_steer_motor[i].angle_now);
			}
		}
	}
	else
	{
		const fp32 rad2deg = 180.0f / PI;
		static fp32 last_diff[4] = {0.0f}; // 用于记录每个舵电机上一时刻temp_move_angle_set和最终的angel_set的差
		fp32 vx_linear = wz_set * 0.707107f;
		fp32 vy_linear = wz_set * 0.707107f;
		fp32 target_angle_rad[4];

		arm_atan2_f32((vy_set + vy_linear), (vx_set + vx_linear), &target_angle_rad[0]);
		arm_atan2_f32((vy_set + vy_linear), (vx_set - vx_linear), &target_angle_rad[1]);
		arm_atan2_f32((vy_set - vy_linear), (vx_set - vx_linear), &target_angle_rad[2]);
		arm_atan2_f32((vy_set - vy_linear), (vx_set + vx_linear), &target_angle_rad[3]);

		for (int i = 0; i < 4; i++)
		{
			fp32 current_diff;

			temp_move_angle_set[i] = target_angle_rad[i] * rad2deg; // 舵电机在运动学下的目标角度
			// 先将舵电机目标角度跨度限制到0~360度
			if (temp_move_angle_set[i] < 0.0f)
				temp_move_angle_set[i] += 360.0f;
			// 选择路程最短的方向旋转，保证舵轮每次旋转的角度小于等于90度
			chassis_steer_motor[i].angle_set = Find_Steer_Min_Angle(temp_move_angle_set[i], chassis_steer_motor[i].angle_now); // angle_set单位：度

			current_diff = my_fabsf(temp_move_angle_set[i] - chassis_steer_motor[i].angle_set); // 只有0，180，360三种情况
			// 车子刚启动时，通过舵电机目标位置判断每个轮电机的旋转方向
			if (is_stop_last)
			{
				int8_t spin_direction_init[4];
				fp32 absolute_angle_error = my_fabsf(chassis_steer_motor[i].angle_now - temp_move_angle_set[i]);
				spin_direction_init[i] = (absolute_angle_error > 90.0f && absolute_angle_error < 270.0f) ? -1 : 1;
				chassis_wheel_motor[i].spin_direction = spin_direction_init[i];
			}
			else // 车子启动后，通过舵电机目标位置判断每个轮电机的旋转方向
			{
				if (my_fabsf(my_fabsf(current_diff - last_diff[i]) - 180.0f) < 0.1f) // 舵电机抄一次180度近路轮电机就需要反转一次,同时注意浮点数计算带来的误差，需要模糊判断
				{
					chassis_wheel_motor[i].spin_direction = -chassis_wheel_motor[i].spin_direction;
				}
			}
			last_diff[i] = current_diff;
		}
	}
	is_stop_last = is_stop;
}

/**
 * @description: 通过底盘坐标系下的目标速度解算出速度四个轮电机的目标速度
 * @return {*}
 * @param {fp32} vx_set 底盘坐标系下x轴目标速度
 * @param {fp32} vy_set 底盘坐标系下y轴目标速度
 * @param {fp32} wz_set 底盘坐标系下z轴目标角速度
 * @param {chassis_mode_t} mode
 */
static void Chassis_Vector_To_Wheel_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, chassis_mode_t mode)
{
	if (mode == CHASSIS_SAFE)
	{
		return;
	}
	fp32 vx_linear = wz_set * 0.707107f;
	fp32 vy_linear = wz_set * 0.707107f;
	fp32 wheel_speed[4];

	arm_sqrt_f32((vy_set + vy_linear) * (vy_set + vy_linear) + (vx_set + vx_linear) * (vx_set + vx_linear), &wheel_speed[0]);
	arm_sqrt_f32((vy_set + vy_linear) * (vy_set + vy_linear) + (vx_set - vx_linear) * (vx_set - vx_linear), &wheel_speed[1]);
	arm_sqrt_f32((vy_set - vy_linear) * (vy_set - vy_linear) + (vx_set - vx_linear) * (vx_set - vx_linear), &wheel_speed[2]);
	arm_sqrt_f32((vy_set - vy_linear) * (vy_set - vy_linear) + (vx_set + vx_linear) * (vx_set + vx_linear), &wheel_speed[3]);

	for (int i = 0; i < 4; i++)
	{
		chassis_wheel_motor[i].speed_set = wheel_speed[i] * chassis_wheel_motor[i].spin_direction; // speed_set单位：rpm
	}
}
/**
 * @brief  通过pid计算出舵电机和轮电机的目标电流
 */
static void Chassis_Motor_Current_Set(chassis_mode_t mode)
{
	if (mode == CHASSIS_SAFE)
		return;

	for (uint8_t i = 0; i < 4; i++)
	{
		PID_calc(&chassis_steer_motor[i].angle_pid, chassis_steer_motor[i].angle_now, chassis_steer_motor[i].angle_set);
		chassis_steer_motor[i].speed_set = chassis_steer_motor[i].angle_pid.out + STEER_MOTOR_SPEED_FF * (chassis_steer_motor[i].angle_set - chassis_steer_motor[i].angle_set_last);
		PID_calc(&chassis_steer_motor[i].speed_pid, chassis_steer_motor[i].speed_now, chassis_steer_motor[i].speed_set);
		chassis_steer_motor[i].give_current = (int16_t)chassis_steer_motor[i].speed_pid.out;

		PID_calc(&chassis_wheel_motor[i].speed_pid, chassis_wheel_motor[i].speed_now, chassis_wheel_motor[i].speed_set);
		chassis_wheel_motor[i].give_current = (int16_t)chassis_wheel_motor[i].speed_pid.out;
	}
}

void Chassis_Task(void const *argument)
{
	Chassis_Motor_Pid_Init();

	vTaskDelay(200);

	chassis_mode = CHASSIS_SAFE;
	while (1)
	{
		chassis_mode = Chassis_Mode_Update();
		Chassis_Data_Update();

		Find_Chassis_Mode_Handler(chassis_mode);
		Chassis_Vector_To_Steer_Angle(chassis_target_speed.vx, chassis_target_speed.vy, chassis_target_speed.wz, chassis_mode);
		Chassis_Vector_To_Wheel_Speed(chassis_target_speed.vx, chassis_target_speed.vy, chassis_target_speed.wz, chassis_mode);
		Chassis_Motor_Current_Set(chassis_mode);

		Allocate_Can_Msg(chassis_steer_motor[0].give_current, chassis_steer_motor[1].give_current, chassis_steer_motor[2].give_current, chassis_steer_motor[3].give_current, CAN_STEER_GM6020_CMD);
		Allocate_Can_Msg(chassis_wheel_motor[0].give_current, chassis_wheel_motor[1].give_current, chassis_wheel_motor[2].give_current, chassis_wheel_motor[3].give_current, CAN_WHEEL_M3508_CMD);
		Vofa_Send_Data4((int16_t)(chassis_steer_motor[0].angle_now), chassis_steer_motor[0].angle_set, chassis_wheel_motor[0].speed_now, chassis_wheel_motor[0].speed_set);
		vTaskDelay(2);
	}
}
