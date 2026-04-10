/********************************************************************************************************************************************
 * @file: bsp_can_chassis.c
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2026赛季CAN总线支持包，此为底盘C板相关的CAN接收和发送代码
 * *******************************************************************************************************************************************
 * @attention: 1.哨兵can报文发送的流程是在各个task计算出需要向电机或其他设备（超电，另一块C板）需要发送的数据后，在xxx_task.c中调用
 *             Allocate_Can_Queue（）或者Ctrl_DM_Motor()。调用规则为如果是向达秒电机发送can报文，则调用Ctrl_DM_Motor()，其他直接
 *             调用Allocate_Can_Queue()。这两个函数的最终目的是将各个task要发送的can报文填充入对应的can发送队列（队列使用freertos实现)，
 * 			   最后can报文会在freertos的定时器任务中统一发送。
 *
 *             2.本文件为各个task提供接口函数和全局变量（注：定时器任务的创建和实现在Can_Send_Task.c/h)。
 **********************************************************************************************************************************************/
#include "bsp_can_chassis.h"
#include "bsp_cap.h"
#include "bsp_dwt.h"
#include "main.h"
#include "motor.h"
#include "detect_task.h"
#include "Chassis_Task.h"
#include "user_common_lib.h"
#include "string.h"
/************************达秒电机控制参数******************************/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -15.0f
#define T_MAX 15.0f
#define P_MIN -12.56637f
#define P_MAX 12.56637f
/*********************************导航数据解析参数*******************************************/
#define NAV_MAX_SPEED 10.0f // 导航最大速度
#define NAV_MIN_SPEED -10.0f // 导航最小速度
/***********************************************全局变量********************************/
CAN_RxHeaderTypeDef rx_header; // debug用，看can接收正不正常
chassis_rc_ctrl_t chassis_rc_ctrl = {0};
uint32_t real_power; // 功率计测出的功率
// int32_t trans = 0; 
// uint16_t can_err_cnt[9];
// uint16_t can_rec_cnt[9];
/*********************************************CAN发送队列*********************************************************************/
QueueHandle_t CAN1_send_queue; // CAN1消息队列句柄,此队列用于储存CAN1的发送消息
QueueHandle_t CAN2_send_queue; // CAN2消息队列句柄，此队列用于储存CAN2的发送消息

#define CAN_SEND_QUEUE_LENGTH 128
#define STEER_GM6020_SEND_QUEUE CAN1_send_queue
#define WHEEL_M3508_SEND_QUEUE CAN1_send_queue
#define CAP_SEND_QUEUE CAN1_send_queue
/*********************************************CAN发送消息实例*********************************************************************/
CanTxMsgTypeDef steer_send_msg; // 传rc_ctrl.rc.ch数组的前四个元素的can报文全局缓冲区
CanTxMsgTypeDef wheel_send_msg; // 传rc_ctrl.rc.ch数组的第五个元素和rc_ctrl.rc.s数组的can报文全局缓冲区
CanTxMsgTypeDef cap_send_msg;   // 大yaw轴达妙6006 can报文全局缓冲区

/**
 * @description: 配置can过滤器，开启can外设以及需要的中断
 * @return 无
 */
void Can_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter;

    can_filter.FilterActivation = ENABLE;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterBank = 0;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.SlaveStartFilterBank = 14;    //can2是从can,这行代码设置了从第几组过滤器开始是属于can2的
    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter.FilterActivation = ENABLE;  //can2配置为只接收大yaw,遥控器和导航的can报文
    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter.FilterIdHigh = (GIMBAL_TO_CHASSIS_FIRST_RecID << 5);
    can_filter.FilterIdLow = (GIMBAL_TO_CHASSIS_SECOND_RecID << 5);
    can_filter.FilterMaskIdHigh = (BIG_YAW_DM6006_RecID << 5);
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterBank = 14;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @description: CAN全局变量初始化，填充can header字段
 * @return {*} 无
 */
void Can_Msg_Init(void)
{
    // 定义不同报文与对应StdId的映射关系（数组批量处理）
    struct
    {
        CanTxMsgTypeDef *msg;
        uint32_t stdId;
    } buffer_list[] = {
        {&steer_send_msg, STEER_GM6020_TransID},
        {&wheel_send_msg, WHEEL_M3508_TransID},
        {&cap_send_msg, CAP_TransID}};

    // 遍历数组，批量初始化
    for (size_t i = 0; i < sizeof(buffer_list) / sizeof(buffer_list[0]); i++)
    {
        buffer_list[i].msg->tx_header.IDE = CAN_ID_STD;                           // 标准帧
        buffer_list[i].msg->tx_header.RTR = CAN_RTR_DATA;                         // 数据帧
        buffer_list[i].msg->tx_header.DLC = 0x08;                                 // 数据长度8字节
        buffer_list[i].msg->tx_header.StdId = buffer_list[i].stdId;               // CAN Id
        memset(buffer_list[i].msg->data, 0, sizeof(buffer_list[i].msg->data)); // 数据缓冲区清零
    }
}

/**
 * @description: 基于freertos创建CAN发送队列
 * @return 无
 */
void Create_Can_Send_Queues()
{
    taskENTER_CRITICAL(); // 进入临界区

    CAN1_send_queue = xQueueCreate(CAN_SEND_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));
    CAN2_send_queue = xQueueCreate(CAN_SEND_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));

    taskEXIT_CRITICAL(); // 退出临界区

    if (CAN1_send_queue == NULL || CAN2_send_queue == NULL)

        Error_Handler();
}
/*********************************************CAN接收函数*********************************************************************/
float EnergyDataToFloat(const uint8_t dat[8])
{
    uint32_t u32 = ((uint32_t)dat[0] << 24) |
                   ((uint32_t)dat[1] << 16) |
                   ((uint32_t)dat[2] << 8) |
                   ((uint32_t)dat[3]);

    float result;
    memcpy(&result, &u32, sizeof(result));
    return result;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    	// static int flag = 0;
    	// static float start_time = 0;

    	// if (flag == 0)
    	// {
    	// 	start_time = DWT_GetTimeline_ms();
    	// 	flag = 1;
        // trans = 1;
        // }
    	// if (flag == 1)
    	// {
    	// 	if (DWT_GetTimeline_ms() - start_time > 1000)
    	// 	{
    	// 		flag = 0;
    	// 		trans = 0;
		// 			for(int j = 0; j < 8; j ++)
        //         {
        //             if(can_rec_cnt[j] < 500)
        //             can_err_cnt[j]++;
        //         }
        //         if(can_rec_cnt[8] < 51)
        //         {
        //             can_err_cnt[8]++;
        //         }
                
        //         for (int j = 0; j < 9; j++)
        //             can_rec_cnt[j] = 0;
    	// 	}
    	// }

    uint8_t rx_data[8];

    if (hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.StdId)
        {
        case WHEEL1_M3508_RecID:
        case WHEEL2_M3508_RecID:
        case WHEEL3_M3508_RecID:
        case WHEEL4_M3508_RecID:
        {
            uint8_t i = rx_header.StdId - WHEEL1_M3508_RecID;
            get_motor_measure(&motor_measure_wheel[i], rx_data);
            detect_hook(WHEEL_MOTOR_1_TOE + i);

            // if(trans == 1)
            // {
            //     can_rec_cnt[i] ++;
            // }

            break;
        }

        case STEER1_GM6020_RecID:
        case STEER2_GM6020_RecID:
        case STEER3_GM6020_RecID:
        case STEER4_GM6020_RecID:
        {
            uint8_t i = rx_header.StdId - STEER1_GM6020_RecID;
            get_motor_measure(&motor_measure_steer[i], rx_data);
            detect_hook(STEER_MOTOR_1_TOE + i);

            // if (trans == 1)
            // {
            //     can_rec_cnt[rx_header.StdId - WHEEL1_M3508_RecID]++;
            // }

            break;
        }

        case CAP_RecID:
        {
            update_cap(rx_data);
            detect_hook(CAP_TOE);
            break;
        }

        case POWER_METER_RecID:
        {
            real_power = EnergyDataToFloat(rx_data);
            // if (trans == 1)
            // {
            //     can_rec_cnt[8]++;
            // }

            break;
        }
        default:
        {
            break;
        }
        }
    }
    if (hcan == &hcan2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        switch (rx_header.StdId)
        {
        case BIG_YAW_DM6006_RecID: // 下板只需要DM6006的位置数据完成底盘跟随云台
        {
            DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
            DM_big_yaw_motor.state = (rx_data[0]) >> 4;
            DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
            break;
        }
        case GIMBAL_TO_CHASSIS_FIRST_RecID:
        {
            uint8_t rc_connected;
            chassis_rc_ctrl.s[1] = rx_data[0];
            rc_connected = rx_data[1];
            chassis_rc_ctrl.ch[2] = (rx_data[2] << 8) | rx_data[3];
            chassis_rc_ctrl.ch[3] = (rx_data[4] << 8) | rx_data[5];
            chassis_rc_ctrl.ch[4] = (rx_data[6] << 8) | rx_data[7];

            if (rc_connected)
                detect_hook(RC_TOE);

            break;
        }
        case GIMBAL_TO_CHASSIS_SECOND_RecID:
        {
            int nav_vx_int = (rx_data[1] << 8) | rx_data[0];
            int nav_vy_int = (rx_data[3] << 8) | rx_data[2];
            
            nav_ctrl.vx = uint_to_float(nav_vx_int,NAV_MIN_SPEED,NAV_MAX_SPEED,12);
            nav_ctrl.vy = uint_to_float(nav_vy_int, NAV_MIN_SPEED, NAV_MAX_SPEED, 12);
            nav_ctrl.chassis_target_mode = rx_data[4] & 0x03; // chassis_target_mode对应rx_data[4]最低两位,下面的flag以此类推
            nav_ctrl.updownhill_state = (rx_data[4] >> 2) & 0x03;
            nav_ctrl.health_state = (rx_data[4] >> 4) & 0x01;
            nav_ctrl.buffer_energy_remain = ((rx_data[5] & 0x07) << 3) | (rx_data[4] >> 5);
            nav_ctrl.referee_power_limit = ((rx_data[6] & 0x07) << 5) | (rx_data[5] >> 3);
            nav_ctrl.game_start = (rx_data[6] >> 3) & 0x01;

            detect_hook(NAV_TOE);
            break;
        }
        default:
        {
            break;
        }
        }
    }
}
/*********************************************************CAN发送，填充CAN消息数据域************************************************************************/
/**
 * @description: 将待发送的can报文送入对应的发送队列，在Can_Send_Task统一发送
 * @return 无
 * @param {int16_t} data1/2/3/4  标准can数据帧的数据域，一共八个字节，这边用int16_t接收是考虑兼容大疆系列电机can数据发送协议，对于别的can报文需要做一些类型转换处理
 * @param {CAN_TX_ID} can_id can报文id
 * @attention 将DM电机can数据帧送入缓冲区需调用Ctrl_DM_Motor（）函数，原因是达秒电机的can数据帧传输的数据有五个，需要先在Ctrl_DM_Motor（）进行处理后再在其内部调用Allocate_Can_Msg（）
 */
void Allocate_Can_Msg(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id)
{
    int16_t data_array[4] = {data1, data2, data3, data4}; // 数据打包成数组，方便循环处理

    switch (can_cmd_id)
    {
    case CAN_STEER_GM6020_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            steer_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            steer_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(STEER_GM6020_SEND_QUEUE, &steer_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_WHEEL_M3508_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            wheel_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(WHEEL_M3508_SEND_QUEUE, &wheel_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_CAP_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            uint16_t temp = data_array[i] * 100;
            cap_send_msg.data[2 * i] = temp & 0xFF;            // 低8位
            cap_send_msg.data[2 * i + 1] = (temp >> 8) & 0xFF; // 高8位
        }
        xQueueSend(CAP_SEND_QUEUE, &cap_send_msg, 0); // 向队列中填充内容
        break;
    }
    default:
        return; // 无效命令，直接返回
    }
}

void CAN_cmd_power_meter(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef gimbal_tx_message;
    uint8_t gimbal_can_send_data[8];

    gimbal_tx_message.StdId = POWER_METER_TransID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}