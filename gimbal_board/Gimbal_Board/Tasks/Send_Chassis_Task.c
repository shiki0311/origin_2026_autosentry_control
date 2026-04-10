/********************************************************************************************************************************************
 * @file: Send_Chassis_Task.c
 * @author: Shiki
 * @date: 2025.11.2
 * @brief:	向下板发送can消息
 * *******************************************************************************************************************************************/
#include "bsp_can_gimbal.h"
#include "remote_control.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "Referee.h"
#include "Cboard_To_Nuc_usbd_communication.h"

/*********************************导航数据解析参数*******************************************/
#define NAV_MAX_SPEED 10.0f  // 导航最大速度
#define NAV_MIN_SPEED -10.0f // 导航最小速度
/**************************哨兵健康状态枚举体***************************/
typedef enum
{
    HEALTH_NORMAL, // 正常模式，低速小陀螺
    HEALTH_HURT    // 被敌方击打，高速小陀螺
} health_state_t;

/**************************需要传到下板的导航数据***************************/
union
{
    uint16_t packed_data[4]; // 数组打包存储所有标志和裁判系统功率相关数据，用于整体赋值+can通信传输
    struct __attribute__((packed))
    {
        uint64_t nav_vx_uint : 16; // 对应packed_flags的最低十六位，下面以此类推
        uint64_t nav_vy_uint : 16;
        uint64_t nav_chassis_mode : 2;  //1底盘跟头2小陀螺
        uint64_t updownhill_state : 2;
        uint64_t health_state : 1;
        uint64_t energy_buffer : 6; //当前底盘剩余缓冲能量
        uint64_t chassis_max_power : 8; //裁判系统传过来的底盘功率上限
        uint64_t game_start : 1; // 比赛是否开始
        uint64_t reserved : 12;         // 保留位
    }single_data;
} __attribute__((packed)) nav_data_u;

/**
 * @description: 根据机器人血量和受伤类型判断是否受到弹丸击打，如果是那么进入受伤模式，维持3秒，期间如果持续受到击打，那么继续延长三秒受伤状态
 * @return {*}
 */
static health_state_t Health_State_Update(void)
{
    static uint32_t hurt_start_time;
    static uint16_t last_robot_hp;
    static health_state_t health_state = HEALTH_NORMAL;         // 在上板判断哨兵健康状态后传到下板，用于设置小陀螺转速
    bool_t hit_by_bullet = (Game_Robot_State.current_HP < last_robot_hp && Robot_Hurt.hurt_type == Hurt_Type_ArmoredPlate); // 判断是否被弹丸击打

    switch (health_state)
    {
    case HEALTH_NORMAL:
        if (hit_by_bullet)
        {
            health_state = HEALTH_HURT;
            hurt_start_time = xTaskGetTickCount();
        }
        break;

    case HEALTH_HURT:
        if (hit_by_bullet)
        {
            hurt_start_time = xTaskGetTickCount();
        }
        else if (xTaskGetTickCount() - hurt_start_time > pdMS_TO_TICKS(3000))
        {
            health_state = HEALTH_NORMAL;
        }
        break;
    }
    last_robot_hp = Game_Robot_State.current_HP;

    return health_state;
}

void Send_Chassis_Task()
{
    while (1)
    {
        
        Allocate_Can_Msg(rc_ctrl.rc.s[1] << 8 | (!toe_is_error(DBUS_TOE)), rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3], rc_ctrl.rc.ch[4], CAN_GIMBAL_TO_CHASSIS_FIRST_CMD);
        vTaskDelay(10);

        nav_data_u.single_data.nav_vx_uint = float_to_uint(NUC_Data_Receive.vx, NAV_MIN_SPEED, NAV_MAX_SPEED, 12);
        nav_data_u.single_data.nav_vy_uint = float_to_uint(NUC_Data_Receive.vy, NAV_MIN_SPEED, NAV_MAX_SPEED, 12); // 把float转换成uint16节省字节，下板收到数据后再用uint_to_float还原数据
                
        nav_data_u.single_data.nav_chassis_mode = NUC_Data_Receive.chassis_mode;
        nav_data_u.single_data.updownhill_state = NUC_Data_Receive.updownhill_state;
        nav_data_u.single_data.health_state = Referee_Data_Transmit.health_state = Health_State_Update();
        nav_data_u.single_data.energy_buffer = (Power_Heat_Data.buffer_energy > 63) ? 63 : Power_Heat_Data.buffer_energy; // 限制在0~(2^6-1)，一般不会超限
        nav_data_u.single_data.chassis_max_power = (Game_Robot_State.chassis_power_limit > 255) ? 255 : Game_Robot_State.chassis_power_limit;  //限制在0~(2^8-1)，一般不会超限

        nav_data_u.single_data.game_start = (Game_Status.game_progress == 4) ? 1 : 0;

        Allocate_Can_Msg(nav_data_u.packed_data[0], nav_data_u.packed_data[1], nav_data_u.packed_data[2], nav_data_u.packed_data[3], CAN_GIMBAL_TO_CHASSIS_SECOND_CMD);
        vTaskDelay(10);

        // Allocate_Can_Msg(0, 0, 0, 0, CAN_GIMBAL_TO_CHASSIS_THIRD_CMD);
    }
}
