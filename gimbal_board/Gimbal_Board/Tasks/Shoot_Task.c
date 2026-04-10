/****************************************************************
 * @file: 	Gimbal_Task.c
 * @author: Shiki
 * @date:	2025.10.5
 * @brief:	2026赛季哨兵发射机构任务
 * @attention:
 ******************************************************************/
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can_gimbal.h"
#include "remote_control.h"
#include "referee.h"
#include "math.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "Vofa_send.h"
#include "user_common_lib.h"
#include "motor.h"
#include "detect_task.h"

typedef struct
{
	bool_t need_limit_heat;				   // 是否需要进行热量保护
	bool_t fric_ready;					   // 摩擦轮是否达到目标转速
	bool_t fric_start;					   // 是否让摩擦轮开始转动
	bool_t dial_over_temperatue;		   // 拨盘电机是否过温
	int16_t fric_target_rpm;			   // 摩擦轮目标转速
	uint16_t cooling_limit_cnt;			   // 热量保护计数
	float current_heat_without_referee; // 不依赖裁判系统的当前热量
} shoot_control_t;

typedef enum									//热量计算状态机
	{
		SHOOT_STEP_READY = 0, 					// 准备发射状态
		SHOOT_STEP_SUSPECT,	  					// 发射嫌疑状态
		SHOOT_STEP_WAIT_CURRENT_FALLING_DOWN, 	//等待电流恢复状态
		SHOOT_STEP_DONE		  					// 发射完成状态
	} shoot_step_e;

shoot_step_e shoot_state = SHOOT_STEP_READY;

shoot_control_t shoot_control = {
	.need_limit_heat = FALSE,
	.fric_ready = FALSE,
	.fric_start = FALSE,
	.dial_over_temperatue = FALSE,
	.fric_target_rpm = 5950,
	.cooling_limit_cnt = 0,
	.current_heat_without_referee = 0};
/*****************************************************************根据裁判系统发射数据进行弹速闭环********************************************************************************/
#define DEBUG_MODE 0 // 日常调试1，比赛前改0

#if HAVE_REFEREE_SYSTEM
#define USE_REFEREE_BULLET_SPEED_LOOP 1 // 1:对裁判系统传回的弹速闭环，外环控弹速（因为裁判系统传回的弹速数据是发射一发子弹传一次，频率不固定，所以只能用状态机控制，不用pid)，内环控摩擦轮3508转速
#endif									// 0:仅对3508摩擦轮速度闭环，适用没有裁判系统的情况

#if USE_REFEREE_BULLET_SPEED_LOOP == 1
// 弹速控制参数
#define TARGET_BULLET_SPEED 23.0f // 目标弹速
#define MIN_ADJUST_STEP 10		  // 弹速很接近目标弹速时每次调整的转速步长
#define MIDDLE_ADJUST_STEP 20	  // 弹速较为接近目标弹速时每次调整的转速步长
#define MAX_ADJUST_STEP 40		  // 弹速距离目标弹速偏差较大时每次调整的转速步长
#define MIN_RPM 5000			  // 摩擦轮最小转速
#define MAX_RPM 7000			  // 摩擦轮最大转速

uint8_t shoot_new_bullet = 0; // 检测发射机构是否发射新子弹

/**
 * @description:根据裁判系统弹速数据判断是否有子弹发射，发射则shoot_new_bullet = 1,只有在USE_REFEREE_BULLET_SPEED_LOOP为1时会被编译
 * @return 无
 */
void Shoot_Bullet_Update()
{
	static fp32 last_bullet_speed;
	if (Shoot_Data.bullet_speed != last_bullet_speed)
	{
		shoot_new_bullet = 1;
	}
	else
	{
		shoot_new_bullet = 0;
	}
	last_bullet_speed = Shoot_Data.bullet_speed;
}
/**
 * @description:选择弹速闭环时的改变摩擦轮转速的步长
 * @return 改变摩擦轮转速的步长
 */
uint8_t Choose_Bullet_Speed_Adjust_Step(float diff)
{
	if (diff > 0.3f)
	{
		if (diff <= 1.0f)
		{
			return MIN_ADJUST_STEP;
		}
		else if (diff <= 2.0f)
		{
			return MIDDLE_ADJUST_STEP;
		}
		else
		{
			return MAX_ADJUST_STEP;
		}
	}
	return 0;
}
/**
 * @description:根据裁判系统反馈弹速调整摩擦轮目标转速
 * @return 无
 */
void Fric_Motor_Speed_Control(void)
{
	static uint8_t bullet_count = 0;	  // 连续弹速计数，发射三发视为一轮
	static float bullet_speed_sum = 0.0f; // 弹速总和（用于计算平均值）
	if (shoot_new_bullet && Game_Robot_State.power_management_shooter_output == 0x01)
	{
		bullet_speed_sum += Shoot_Data.bullet_speed;
		bullet_count++;

		// 每3发子弹进行一次调速
		if (bullet_count >= 3)
		{
			float avg_speed = bullet_speed_sum / bullet_count;
			// 调速逻辑
			float diff = TARGET_BULLET_SPEED - avg_speed;
			uint8_t step = Choose_Bullet_Speed_Adjust_Step(my_fabsf(diff));
			shoot_control.fric_target_rpm += my_sign(diff) * step;
			// 对目标转速进行约束
			shoot_control.fric_target_rpm = limit(shoot_control.fric_target_rpm, (float)MIN_RPM, (float)MAX_RPM);
			// 重置计数
			bullet_speed_sum = 0.0f;
			bullet_count = 0;
		}
	}
}
#endif
/********************************************************************************************************************************************************************************/
void Shoot_Motor_Pid_Init(void)
{
	const static fp32 dial_motor_speed_pid[3] = {DIAL_MOTOR_SPEED_PID_KP, DIAL_MOTOR_SPEED_PID_KI, DIAL_MOTOR_SPEED_PID_KD};
	const static fp32 dial_motor_angle_pid[3] = {DIAL_MOTOR_ANGLE_PID_KP, DIAL_MOTOR_ANGLE_PID_KI, DIAL_MOTOR_ANGLE_PID_KD};

	PID_init(&LK_dial_motor.speed_pid, PID_POSITION, dial_motor_speed_pid, DIAL_MOTOR_SPEED_PID_MAX_OUT, DIAL_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&LK_dial_motor.angle_pid, PID_POSITION, dial_motor_angle_pid, DIAL_MOTOR_ANGLE_PID_MAX_OUT, DIAL_MOTOR_ANGLE_PID_MAX_IOUT);

	const static fp32 fric_motor_speed_pid[3] = {FRIC_MOTOR_SPEED_PID_KP, FRIC_MOTOR_SPEED_PID_KI, FRIC_MOTOR_SPEED_PID_KD};

	PID_init(&fric_motor[0].speed_pid, PID_POSITION, fric_motor_speed_pid, FRIC_MOTOR_SPEED_PID_MAX_OUT, FRIC_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&fric_motor[1].speed_pid, PID_POSITION, fric_motor_speed_pid, FRIC_MOTOR_SPEED_PID_MAX_OUT, FRIC_MOTOR_SPEED_PID_MAX_IOUT);
}

/**
 * @description:更新波蛋盘2006电机和两个摩擦轮3508电机的数据
 * @return 无
 */
void Shoot_Motor_Data_Update(void)
{
	LK_dial_motor.speed_now = motor_measure_dial.speed / 6.0f;
	LK_dial_motor.angle_now = motor_measure_dial.ecd / 65535.0f * 360.0f;
	LK_dial_motor.given_current = motor_measure_dial.given_current;
	LK_dial_motor.temperature = motor_measure_dial.temperature;

	for (int i = 0; i < 2; i++)
	{
		fric_motor[i].speed_now = motor_measure_fric[i].speed_rpm;
		fric_motor[i].given_current = motor_measure_fric[i].given_current;
	}
}

/**
 * @description:更新shoot_control.fric_start,shoot_control.fric_start为1时摩擦轮开始转动
 * @return 无
 */
void Fric_Start_Update()
{
	bool_t start_fric;

#if HAVE_REFEREE_SYSTEM
	start_fric = ((rc_ctrl.rc.s[1] == RC_SW_UP) || ((rc_ctrl.rc.s[1] == RC_SW_MID) && (rc_ctrl.rc.s[0] != RC_SW_DOWN))) && Game_Robot_State.power_management_shooter_output == 0x01;
#else
	start_fric = ((rc_ctrl.rc.s[1] == RC_SW_UP) || ((rc_ctrl.rc.s[1] == RC_SW_MID) && (rc_ctrl.rc.s[0] != RC_SW_DOWN)));
#endif

	if (start_fric)
	{
		shoot_control.fric_start = TRUE;
	}
	else
	{
		shoot_control.fric_start = FALSE;
	}
}
/**
 * @description:更新shoot_control.fric_ready,shoot_control.fric_ready为1时才能转波蛋盘，目的是确保在摩擦轮转速达到后才发弹
 * @return 无
 */
void Fric_Ready_Update(void)
{
	if ((fric_motor[0].speed_now < -(shoot_control.fric_target_rpm - 300)) && (fric_motor[1].speed_now > shoot_control.fric_target_rpm - 300))
	{
		shoot_control.fric_ready = TRUE;
	}
	else
		shoot_control.fric_ready = FALSE;
}

void Fric_Motor_Current_Control(void)
{
	if (shoot_control.fric_start && !toe_is_error(DBUS_TOE))
	{
		fric_motor[0].speed_set = -shoot_control.fric_target_rpm;
		fric_motor[1].speed_set = shoot_control.fric_target_rpm;
	}
	else
	{
		fric_motor[0].speed_set = 0;
		fric_motor[1].speed_set = 0;
	}

	PID_calc(&fric_motor[0].speed_pid, fric_motor[0].speed_now, fric_motor[0].speed_set);
	PID_calc(&fric_motor[1].speed_pid, fric_motor[1].speed_now, fric_motor[1].speed_set);

	fric_motor[0].give_current = fric_motor[0].speed_pid.out; // - dif_pid.out;
	fric_motor[1].give_current = fric_motor[1].speed_pid.out; // + dif_pid.out;
}

/**
 * @description:自主计算当前热量
 * @return 当前热量
 */
float Get_Current_Heat(void)
{
	static float current_heat = 0.0f;
	static uint16_t last_robot_hp = 0;			  // 上一次记录的机器人血量,用于判断是否刚刚复活
	static uint32_t shoot_suspect_start_time = 0; // 记录进入嫌疑状态的系统时间


	// 获取摩擦轮反馈电流绝对值
	float current_0 = -(float)fric_motor[0].given_current;
	float current_1 = (float)fric_motor[1].given_current;
	// 定义判断阈值
	const float CURRENT_MAX_THRESHOLD = 5500.0f;	// 摩擦轮反馈电流绝对值最大阈值 (可根据实际情况调整)
	const float CURRENT_MIN_THRESHOLD = 1500.0f;    // 摩擦轮反馈电流绝对值最小阈值 (可根据实际情况调整)
	const float CURRENT_FELL_DOWN = 1000.0f;		//摩擦轮反馈电流下沿
	const uint32_t SUSPECT_TIME_THRESHOLD = 5; // 嫌疑状态持续时间阈值(ms)，超过此时间则判定为发射完成
	const uint32_t CHECK_FAILED_TIME = 50;		
	// 判断是否刚刚复活
	bool_t is_robot_reborn = (Game_Robot_State.current_HP != 0 && last_robot_hp == 0);
	last_robot_hp = Game_Robot_State.current_HP;

	if (is_robot_reborn)
	{
		current_heat = 0.0f;
		shoot_state = SHOOT_STEP_READY;
	}
	else
	{
		// 状态机处理
		switch (shoot_state)
		{
		case SHOOT_STEP_READY:
			// 如果摩擦轮已准备好（转速达到目标转速附近），且电流突然超过阈值，认为子弹可能进入摩擦轮
			if (shoot_control.fric_ready && current_0 < CURRENT_MAX_THRESHOLD  && current_0 > CURRENT_MIN_THRESHOLD )
			{
				shoot_state = SHOOT_STEP_SUSPECT;
				shoot_suspect_start_time = xTaskGetTickCount(); // 记录嫌疑开始时间
			}
			break;

		case SHOOT_STEP_SUSPECT:
			// 如果电流依然大于阈值
			if (current_0 < CURRENT_MAX_THRESHOLD  && current_0 > CURRENT_MIN_THRESHOLD )
			{
				// 如果持续时间超过设定阈值，则认为射出一发子弹
				if ((xTaskGetTickCount() - shoot_suspect_start_time) > SUSPECT_TIME_THRESHOLD )
				{
					shoot_state = SHOOT_STEP_WAIT_CURRENT_FALLING_DOWN;
				}
			}
			else
				shoot_state = SHOOT_STEP_READY;
			break;

		case SHOOT_STEP_WAIT_CURRENT_FALLING_DOWN:
			//如果电流回到下沿		
			if( current_0 < CURRENT_FELL_DOWN )
			{
				shoot_state = SHOOT_STEP_DONE;
			}
			else if( (xTaskGetTickCount() - shoot_suspect_start_time) > CHECK_FAILED_TIME )
			{
				shoot_state = SHOOT_STEP_READY;
			}
			break;


		case SHOOT_STEP_DONE:
			// 确认为发射完成，更新热量
			current_heat += 10.0f;
			shoot_state = SHOOT_STEP_READY;
			break;

		default:
			shoot_state = SHOOT_STEP_READY;
			break;
		}
	}

	// 更新并记录时间戳，计算时间间隔 dt (秒)
	uint32_t current_time = xTaskGetTickCount();
	static uint32_t last_time = 0;
	if (last_time == 0) {
		last_time = current_time;
	}
	float dt = (current_time - last_time) / 1000.0f;
	last_time = current_time;

	// 按冷却速率更新热量
	current_heat -= (float)Game_Robot_State.shooter_barrel_cooling_value * dt;
	
	// 限制最低热量为 0
	current_heat = (current_heat < 0.0f ? 0.0f : current_heat);
	
	

	return current_heat;
}
/**
 * @description:设置波蛋盘目标转速，在Dial_Motor_Control()中调用
 * @return 波蛋盘目标转速
 */
void Dial_Speed_Set(fp32 *dial_speed)
{
#if HAVE_REFEREE_SYSTEM
	if ((rc_ctrl.rc.s[1] == RC_SW_UP || (NUC_Data_Receive.small_yaw_aim != 0) || (rc_ctrl.rc.s[1] == RC_SW_MID && rc_ctrl.rc.s[0] == RC_SW_UP)) && Game_Robot_State.power_management_shooter_output == 0x01) // 判断是否要进行热量保护,快超热量了就把拨弹盘目标速度定为0一段时间
	{
		if ((Power_Heat_Data.shooter_17mm_barrel_heat >= (Game_Robot_State.shooter_barrel_heat_limit - 60)))  
		{
			shoot_control.need_limit_heat = 1;
		}
		if (shoot_control.need_limit_heat)
		{
			shoot_control.cooling_limit_cnt++;
			if (shoot_control.cooling_limit_cnt >= 250)
			{
				shoot_control.cooling_limit_cnt = 0;
				shoot_control.need_limit_heat = 0;
			}

			*dial_speed = 0;
			return;
		}
	}
#endif

	bool_t remote_control_shoot = (rc_ctrl.rc.s[0] == RC_SW_UP && rc_ctrl.rc.s[1] == RC_SW_MID);

#if DEBUG_MODE == 0
	bool_t autoaim_shoot = (rc_ctrl.rc.s[1] == RC_SW_UP && NUC_Data_Receive.fire_or_not == 1 && Game_Status.game_progress == 4);
#else
	bool_t autoaim_shoot = (rc_ctrl.rc.s[1] == RC_SW_UP && NUC_Data_Receive.fire_or_not == 1);
#endif

	if (shoot_control.fric_ready && (remote_control_shoot || autoaim_shoot))
	{
		*dial_speed = (float)DIAL_SPEED_HIGH;
	}
	else
		*dial_speed = 0;
}

void Dial_Motor_Control(void)
{
	if (LK_dial_motor.temperature > 80)
		shoot_control.dial_over_temperatue = TRUE;

	if (shoot_control.dial_over_temperatue && LK_dial_motor.temperature < 60)
		shoot_control.dial_over_temperatue = FALSE;

#if HAVE_REFEREE_SYSTEM
	bool_t disable_dial_motor = (rc_ctrl.rc.s[1] == RC_SW_DOWN || toe_is_error(DBUS_TOE) || shoot_control.dial_over_temperatue || Game_Robot_State.power_management_shooter_output == 0x00);
#else
	bool_t disable_dial_motor = (rc_ctrl.rc.s[1] == RC_SW_DOWN || toe_is_error(DBUS_TOE) || shoot_control.dial_over_temperatue);
#endif

	if (disable_dial_motor) // 失能状态直接给波蛋盘电机目标电流置零
	{
		LK_dial_motor.give_current = 0;
		PID_clear(&LK_dial_motor.speed_pid);
		PID_clear(&LK_dial_motor.angle_pid);
		return;
	}

	static uint8_t dial_back_flag = 0; // 退弹保护flag
	static uint32_t dial_stop_cnt = 0;
	static uint16_t back_start_time = 0;

	if (dial_back_flag) // 如果需要退弹保护，波蛋盘反转一段时间
	{
		if (xTaskGetTickCount() - back_start_time > 300) // 如果超出时间阈值，就退出退弹保护
		{
			dial_back_flag = 0;
			dial_stop_cnt = 0;
			PID_clear(&LK_dial_motor.speed_pid);
			PID_clear(&LK_dial_motor.angle_pid);
		}
		else
			return; // 时间阈值没到，target_current继续保持5000或-5000
	}
	if (abs(LK_dial_motor.give_current) > (0.85 * LK_dial_motor.speed_pid.Kp * DIAL_SPEED_HIGH) && fabs(LK_dial_motor.speed_now) < 20 && !toe_is_error(DIAL_MOTOR_TOE) && shoot_control.fric_ready) // 波蛋盘电机给定电流较大但是转速很小，说明拨弹盘卡住了
	{
		dial_stop_cnt++;
		if (dial_stop_cnt > 300)
		{
			dial_back_flag = 1;
			shoot_state = SHOOT_STEP_READY;
			back_start_time = xTaskGetTickCount();
			if (LK_dial_motor.give_current < 0)
				LK_dial_motor.give_current = 1200;
			else
				LK_dial_motor.give_current = -1200;

			return;
		}
	}
	Dial_Speed_Set(&LK_dial_motor.speed_set);
	PID_calc(&LK_dial_motor.speed_pid, LK_dial_motor.speed_now, LK_dial_motor.speed_set);

	LK_dial_motor.give_current = LK_dial_motor.speed_pid.out;
}

void Shoot_Task(void const *argument)
{
	Shoot_Motor_Pid_Init();
	vTaskDelay(200);

	while (1)
	{
		static uint8_t cnt = 1;

		Shoot_Motor_Data_Update();
		Fric_Start_Update();
		Fric_Ready_Update();
		shoot_control.current_heat_without_referee = Get_Current_Heat();  

#if USE_REFEREE_BULLET_SPEED_LOOP
		Shoot_Bullet_Update();
		Fric_Motor_Speed_Control();
#endif

		Fric_Motor_Current_Control();
		Dial_Motor_Control();

		Allocate_Can_Msg(fric_motor[0].give_current, fric_motor[1].give_current, 0, 0, CAN_FRIC_CMD);

		Allocate_Can_Msg(LK_MOTOR_TORQUE_CONTROL_CMD_ID, 0, LK_dial_motor.give_current, 0, CAN_DIAL_CMD);
		//Vofa_Send_Data4(LK_dial_motor.speed_now, LK_dial_motor.speed_set, 0, 0);
		Vofa_Send_Data4(shoot_control.current_heat_without_referee*(-100), motor_measure_fric[0].given_current, motor_measure_fric[1].given_current, 0);

		cnt == 120 ? cnt = 1 : cnt++; // div等于2,3,4,5的最小公倍数时重置

		vTaskDelay(3);
	}
}
