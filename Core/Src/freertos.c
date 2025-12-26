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
#include "tim.h"
#include "pump_ctrl.h"
#include "app.h"

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// 添加气泵PWM任务相关声明
void PumpPWMTask(void *argument);
osThreadId_t PumpPWMTaskHandle;
const osThreadAttr_t PumpPWMTask_attributes = {
  .name = "PumpPWMTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//机械臂初始化任务声明
/* Definitions for arm_init */
osThreadId_t arm_initHandle;
const osThreadAttr_t arm_init_attributes = {
  .name = "arm_init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};

//机械臂控制任务声明
/* Definitions for arm_control */
osThreadId_t arm_controlHandle;
const osThreadAttr_t arm_control_attributes = {
  .name = "arm_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //创建气泵PWM任务
  PumpPWMTaskHandle = osThreadNew(PumpPWMTask, NULL, &PumpPWMTask_attributes);
  
  //创建机械臂初始化任务
  /* creation of arm_init */
  arm_initHandle = osThreadNew(Arm_Init, NULL, &arm_init_attributes);

  //创建机械臂控制任务
  /* creation of arm_control */
  arm_controlHandle = osThreadNew(Arm_control, NULL, &arm_control_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void PumpPWMTask(void *argument)
{
  /* USER CODE BEGIN PumpPWMTask */
  // 启动 TIM3 通道1 PWM 输出
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
    // 示例：设置气泵功率为75%
    Pump_SetPower(75);
    // 打开电磁阀
    Pump_ValveOn();
    osDelay(2000); // 运行5秒

    // 设置气泵功率为0（关闭）
    Pump_SetPower(0);
    // 关闭电磁阀
    Pump_ValveOff();
    osDelay(2000); // 停止5秒
  }
  /* USER CODE END PumpPWMTask */
}
/* USER CODE END Application */

