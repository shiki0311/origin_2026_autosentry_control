/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId INS_TASKHandle;
osThreadId LED_TASKHandle;
osThreadId Gimbal_TASKHandle;
osThreadId Shoot_TASKHandle;
osThreadId referee_usartHandle;
osThreadId Detect_TaskHandle;
osThreadId Send_ChassisHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void LED_Task(void const * argument);
void Gimbal_Task(void const * argument);
void Shoot_Task(void const * argument);
void referee_usart_task(void const * argument);
void detect_task(void const * argument);
void Send_Chassis_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 2560);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of LED_TASK */
  osThreadDef(LED_TASK, LED_Task, osPriorityNormal, 0, 128);
  LED_TASKHandle = osThreadCreate(osThread(LED_TASK), NULL);

  // /* definition and creation of Gimbal_TASK */
  osThreadDef(Gimbal_TASK, Gimbal_Task, osPriorityHigh, 0, 1024);
  Gimbal_TASKHandle = osThreadCreate(osThread(Gimbal_TASK), NULL);

  // /* definition and creation of Shoot_TASK */
  osThreadDef(Shoot_TASK, Shoot_Task, osPriorityAboveNormal, 0, 512);
  Shoot_TASKHandle = osThreadCreate(osThread(Shoot_TASK), NULL);

  // /* definition and creation of referee_usart */
  osThreadDef(referee_usart, referee_usart_task, osPriorityHigh, 0, 256);
  referee_usartHandle = osThreadCreate(osThread(referee_usart), NULL);

  // /* definition and creation of Detect_Task */
  osThreadDef(Detect_Task, detect_task, osPriorityAboveNormal, 0, 256);
  Detect_TaskHandle = osThreadCreate(osThread(Detect_Task), NULL);

  // /* definition and creation of Send_Chassis */
  osThreadDef(Send_Chassis, Send_Chassis_Task, osPriorityHigh, 0, 256);
  Send_ChassisHandle = osThreadCreate(osThread(Send_Chassis), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the INS_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Task */
__weak void LED_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_referee_usart_task */
/**
* @brief Function implementing the referee_usart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_referee_usart_task */
__weak void referee_usart_task(void const * argument)
{
  /* USER CODE BEGIN referee_usart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END referee_usart_task */
}

/* USER CODE BEGIN Header_detect_task */
/**
* @brief Function implementing the Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_task */
__weak void detect_task(void const * argument)
{
  /* USER CODE BEGIN detect_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END detect_task */
}

/* USER CODE BEGIN Header_Send_Chassis_Task */
/**
* @brief Function implementing the Send_Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_Chassis_Task */
__weak void Send_Chassis_Task(void const *argument)
{
  /* USER CODE BEGIN Send_Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Send_Chassis_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
