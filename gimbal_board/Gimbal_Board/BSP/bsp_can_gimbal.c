/********************************************************************************************************************************************
 * @file: bsp_can.c
 * @author: Shiki
 * @date: 2025.10.21
 * @brief:	哨兵2026赛季CAN总线支持包，此为云台C板相关的CAN接收和发送代码
 * *******************************************************************************************************************************************
 * @attention: 1.哨兵can报文发送的流程是在各个task计算出需要向电机或其他设备（超电，另一块C板）需要发送的数据后，在xxx_task.c中调用
 *             Allocate_Can_Queue（）或者Ctrl_DM_Motor()。调用规则为如果是向达秒电机发送can报文，则调用Ctrl_DM_Motor()，其他直接
 *             调用Allocate_Can_Queue()。这两个函数的最终目的是将各个task要发送的can报文填充入对应的can发送队列（队列使用freertos实现)，
 * 			   最后can报文会在freertos的定时器任务中统一发送。
 *
 *             2.本文件为各个task提供接口函数和全局变量（注：定时器任务的创建和实现在Can_Send_Task.c/h)。
 **********************************************************************************************************************************************/
#include "bsp_can_gimbal.h"
#include "bsp_dwt.h"
#include "main.h"
#include "motor.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "string.h"

/************************达秒电机控制参数**********************************/
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
/*********************************CAN接收ID*******************************************/
#define BIG_YAW_DM6006_RecID 0x300 // CAN2

#define SMALL_YAW_GM6020_RecID 0x205 // CAN1
#define PITCH_GM6020_RecID 0x206     // CAN1
#define FRIC1_M3508_RecID 0x201      // CAN1
#define FRIC2_M3508_RecID 0x202      // CAN1
#define DIAL_RecID 0x311             // CAN1，暂未确定电机型号，待定

/*********************************CAN发送ID*******************************************/
#define RC_TO_CHASSIS_FIRST_ID 0x102  // CAN2,向下板发送遥控器数据
#define RC_TO_CHASSIS_SECOND_ID 0x100 // CAN2,发什么待定
#define BIG_YAW_DM6006_TransID 0x01   // CAN2,DM6006

#define SMALL_YAW_AND_PITCH_TransID 0x1FF // CAN1,两个6020一起发
#define FRIC_M3508_TransID 0x200          // CAN1,两个3508一起发
#define DIAL_TransID 0x312                // CAN1,暂未确定电机型号，待定
/************************************************全局变量*******************************/
CAN_RxHeaderTypeDef rx_header; // debug用，看can接收正不正常
// int32_t trans_freq = 0; // can发送定时器的中断回调函数每秒执行次数，配合dwt使用
/*********************************************CAN发送队列*********************************************************************/
#define CAN_TX_QUEUE_LENGTH 128
QueueHandle_t CAN1_send_queue; // CAN1消息队列句柄,此队列用于储存CAN1第一次发送失败的消息
QueueHandle_t CAN2_send_queue; // CAN2消息队列句柄，此队列用于储存CAN2第一次发送失败的消息

#define RC_TO_CHASSIS_FIRST_SEND_QUEUE CAN2_send_queue
#define RC_TO_CHASSIS_SECOND_SEND_QUEUE CAN2_send_queue
#define BIG_YAW_DM6006_SEND_QUEUE CAN2_send_queue
#define SMALL_YAW_AND_PITCH_SEND_QUEUE CAN1_send_queue
#define FRIC_M3508_SEND_QUEUE CAN1_send_queue
#define DIAL_SEND_QUEUE CAN1_send_queue
/*********************************************CAN发送消息实例*********************************************************************/
CanTxMsgTypeDef rc_to_chassis_first_send_msg;  // 传rc_ctrl.rc.ch数组的前四个元素的can报文
CanTxMsgTypeDef rc_to_chassis_second_send_msg; // 传rc_ctrl.rc.ch数组的第五个元素和rc_ctrl.rc.s数组的can报文
CanTxMsgTypeDef big_yaw_send_msg;              // 大yaw轴达妙6006 can报文
CanTxMsgTypeDef small_yaw_and_pitch_send_msg;  // pitch轴，小yaw轴6020 can报文
CanTxMsgTypeDef fric_send_msg;                 // 摩擦轮3508can报文
CanTxMsgTypeDef dial_send_msg;                 // 拨弹盘can报文

/**
 * @description: 配置can过滤器，开启can外设以及需要的中断
 * @return 无
 */
void Can_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @description: CAN缓冲区初始化，填充can header字段
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
        {&rc_to_chassis_first_send_msg, RC_TO_CHASSIS_FIRST_ID},
        {&rc_to_chassis_second_send_msg, RC_TO_CHASSIS_SECOND_ID},
        {&big_yaw_send_msg, BIG_YAW_DM6006_TransID},
        {&small_yaw_and_pitch_send_msg, SMALL_YAW_AND_PITCH_TransID},
        {&fric_send_msg, FRIC_M3508_TransID},
        {&dial_send_msg, DIAL_TransID}};

    // 对于dm以外的电机，初始化为0。对于dm电机需要经过dm协议转换，不能直接赋0！！！！！！！！
    for (size_t i = 0; i < sizeof(buffer_list) / sizeof(buffer_list[0]); i++)
    {
        buffer_list[i].msg->tx_header.IDE = CAN_ID_STD;             // 标准帧
        buffer_list[i].msg->tx_header.RTR = CAN_RTR_DATA;           // 数据帧
        buffer_list[i].msg->tx_header.DLC = 0x08;                   // 数据长度8字节
        buffer_list[i].msg->tx_header.StdId = buffer_list[i].stdId; // CAN Id

        if (buffer_list[i].stdId == BIG_YAW_DM6006_TransID)
        {
            uint16_t pos_init, vel_init, kp_init, kd_init, tor_init;
            pos_init = float_to_uint(0, P_MIN, P_MAX, 16);
            vel_init = float_to_uint(0, V_MIN, V_MAX, 12);
            kp_init = float_to_uint(0, KP_MIN, KP_MAX, 12);
            kd_init = float_to_uint(0, KD_MIN, KD_MAX, 12);
            tor_init = float_to_uint(0, T_MIN, T_MAX, 12);
            buffer_list[i].msg->data[0] = (pos_init >> 8);
            buffer_list[i].msg->data[1] = pos_init;
            buffer_list[i].msg->data[2] = (vel_init >> 4);
            buffer_list[i].msg->data[3] = ((vel_init & 0xF) << 4) | (kp_init >> 8);
            buffer_list[i].msg->data[4] = kp_init;
            buffer_list[i].msg->data[5] = (kd_init >> 4);
            buffer_list[i].msg->data[6] = ((kd_init & 0xF) << 4) | (tor_init >> 8);
            buffer_list[i].msg->data[7] = tor_init;
        }
        else
        {
            memset(buffer_list[i].msg->data, 0, sizeof(buffer_list[i].msg->data));
        }
    }
}

/**
 * @description: 基于freertos创建CAN发送队列
 * @return 无
 */
void Create_Can_Send_Queues()
{
    taskENTER_CRITICAL(); // 进入临界区

    CAN1_send_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));
    CAN2_send_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));

    taskEXIT_CRITICAL(); // 退出临界区

    if (CAN1_send_queue == NULL || CAN2_send_queue == NULL)

        Error_Handler();
}
/*********************************************CAN接收函数*********************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rx_data[8];
    if (hcan == &hcan1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.StdId)
        {
        case FRIC1_M3508_RecID:
        case FRIC2_M3508_RecID:
        {
            uint8_t i = rx_header.StdId - FRIC1_M3508_RecID;
            get_motor_measure(&motor_measure_shoot[i], rx_data);
            // detect_hook(CHASSIS_MOTOR1_TOE + i);
            break;
        }
        case SMALL_YAW_GM6020_RecID:
        {
            get_motor_measure(&motor_measure_small_yaw, rx_data);
            break;
        }
        case PITCH_GM6020_RecID:
        {
            get_motor_measure(&motor_measure_pitch, rx_data);
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
        if (rx_header.StdId == BIG_YAW_DM6006_RecID)
        {
            DM_big_yaw_motor.id = (rx_data[0]) & 0x0F;
            DM_big_yaw_motor.state = (rx_data[0]) >> 4;
            DM_big_yaw_motor.p_int = (rx_data[1] << 8) | rx_data[2];
            DM_big_yaw_motor.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
            DM_big_yaw_motor.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
            DM_big_yaw_motor.pos = uint_to_float(DM_big_yaw_motor.p_int, P_MIN, P_MAX, 16) * 57.3248408;
            DM_big_yaw_motor.vel = uint_to_float(DM_big_yaw_motor.v_int, V_MIN, V_MAX, 12);
            DM_big_yaw_motor.toq = uint_to_float(DM_big_yaw_motor.t_int, T_MIN, T_MAX, 12);
            DM_big_yaw_motor.Tmos = (float)(rx_data[6]);
            DM_big_yaw_motor.Tcoil = (float)(rx_data[7]);
        }
    }
}

/*********************************************************填充CAN消息数据域************************************************************************/
/**
 * @description: 将待发送的can报文送入对应的发送队列，在CAN_TX_TimerIRQHandler()函数中将缓冲区中的can报文统一定时发送
 * @return 无
 * @param {int16_t} data1/2/3/4  标准can数据帧的数据域，一共八个字节，这边用int16_t接收是考虑兼容大疆系列电机can数据发送协议，对于别的can报文需要做一些类型转换处理
 * @param {CAN_TX_ID} can_id can报文id
 * @attention 将DM电机can数据帧送入缓冲区需调用Ctrl_DM_Motor（）函数，原因是达秒电机的can数据帧传输的数据有五个，需要先在Ctrl_DM_Motor（）进行处理后再在其内部调用Allocate_Can_Buffer（）
 */
void Allocate_Can_Msg(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id)
{
    int16_t data_array[4] = {data1, data2, data3, data4}; // 数据打包成数组，方便循环处理

    // 根据命令ID，让指针指向对应的缓冲区
    switch (can_cmd_id)
    {
    case CAN_RC_TO_CHASSIS_FIRST_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            rc_to_chassis_first_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            rc_to_chassis_first_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(RC_TO_CHASSIS_FIRST_SEND_QUEUE, &rc_to_chassis_first_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_RC_TO_CHASSIS_SECOND_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            rc_to_chassis_second_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            rc_to_chassis_second_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(RC_TO_CHASSIS_SECOND_SEND_QUEUE, &rc_to_chassis_second_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_BIG_YAW_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            big_yaw_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            big_yaw_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(BIG_YAW_DM6006_SEND_QUEUE, &big_yaw_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_SMALL_YAW_AND_PITCH_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            small_yaw_and_pitch_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            small_yaw_and_pitch_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(SMALL_YAW_AND_PITCH_SEND_QUEUE, &small_yaw_and_pitch_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_FRIC_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            fric_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            fric_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(FRIC_M3508_SEND_QUEUE, &fric_send_msg, 0); // 向队列中填充内容
        break;
    }
    case CAN_DIAL_CMD:
    {
        for (int i = 0; i < 4; i++)
        {
            dial_send_msg.data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
            dial_send_msg.data[2 * i + 1] = data_array[i] & 0xFF;    // 低8位
        }
        xQueueSend(DIAL_SEND_QUEUE, &dial_send_msg, 0); // 向队列中填充内容
        break;
    }
    default:
        return; // 无效命令，直接返回
    }
}

void Ctrl_DM_Motor(float _pos, float _vel, float _KP, float _KD, float _torq) // can2
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(_vel, -45, 45, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    uint16_t data1, data2, data3, data4;
    data1 = pos_tmp;
    data2 = ((vel_tmp >> 4) << 8) | ((vel_tmp & 0x0F) << 4 | (kp_tmp >> 8));
    data3 = ((kp_tmp & 0xFF) << 8) | (kd_tmp >> 4);
    data4 = ((kd_tmp & 0x0F) << 12) | tor_tmp;

    Allocate_Can_Msg(data1, data2, data3, data4, CAN_BIG_YAW_CMD);
}
/*********************************************DM使能和失能函数***********************************************************/
void enable_DM(uint8_t id, DM_CTRL_MODE ctrl_mode)
{
    uint8_t TX_Data[8];
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Tx_Msg;

    if (ctrl_mode == MIT)
        Tx_Msg.StdId = 0x000 + id;
    else if (ctrl_mode == VEL_POS)
        Tx_Msg.StdId = 0x100 + id;
    else if (ctrl_mode == VEL)
        Tx_Msg.StdId = 0x200 + id;
    Tx_Msg.IDE = CAN_ID_STD;
    Tx_Msg.RTR = CAN_RTR_DATA;
    Tx_Msg.DLC = 8;

    TX_Data[0] = 0xff;
    TX_Data[1] = 0xff;
    TX_Data[2] = 0xff;
    TX_Data[3] = 0xff;
    TX_Data[4] = 0xff;
    TX_Data[5] = 0xff;
    TX_Data[6] = 0xff;
    TX_Data[7] = 0xfc;

    HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}

void disable_DM(uint8_t id, DM_CTRL_MODE ctrl_mode)
{
    uint8_t TX_Data[8];
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Tx_Msg;

    if (ctrl_mode == MIT)
        Tx_Msg.StdId = 0x000 + id;
    else if (ctrl_mode == VEL_POS)
        Tx_Msg.StdId = 0x100 + id;
    else if (ctrl_mode == VEL)
        Tx_Msg.StdId = 0x200 + id;

    Tx_Msg.IDE = CAN_ID_STD;
    Tx_Msg.RTR = CAN_RTR_DATA;
    Tx_Msg.DLC = 8;

    TX_Data[0] = 0xff;
    TX_Data[1] = 0xff;
    TX_Data[2] = 0xff;
    TX_Data[3] = 0xff;
    TX_Data[4] = 0xff;
    TX_Data[5] = 0xff;
    TX_Data[6] = 0xff;
    TX_Data[7] = 0xfd;

    HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}
