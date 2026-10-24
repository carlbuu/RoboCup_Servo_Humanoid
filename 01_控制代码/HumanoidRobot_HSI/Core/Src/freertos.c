/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_tasks.h"
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
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CameraTask */
osThreadId_t CameraTaskHandle;
const osThreadAttr_t CameraTask_attributes = {
  .name = "CameraTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotTask */
osThreadId_t RobotTaskHandle;
const osThreadAttr_t RobotTask_attributes = {
  .name = "RobotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DebugTask */
osThreadId_t DebugTaskHandle;
const osThreadAttr_t DebugTask_attributes = {
  .name = "DebugTask",
  .stack_size = 384 * 4, /* 接收诊断增加printf调用深度，保留栈余量。 */
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

extern void IMU_Task(void *argument);
extern void Camera_Task(void *argument);
extern void ServoTask(void *argument);
extern void RobotTask(void *argument);
extern void Debug_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /*
   * 创建 UART 接收队列和调试互斥量。
   * 队列必须先于任务创建，确保调度器启动后任务立即可安全使用。
   */
  App_RTOS_ObjectsInit();
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
  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(IMU_Task, NULL, &IMUTask_attributes);

  /* creation of CameraTask */
  CameraTaskHandle = osThreadNew(Camera_Task, NULL, &CameraTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(ServoTask, NULL, &ServoTask_attributes);

  /* creation of RobotTask */
  RobotTaskHandle = osThreadNew(RobotTask, NULL, &RobotTask_attributes);

  /* creation of DebugTask */
  DebugTaskHandle = osThreadNew(Debug_Task, NULL, &DebugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /*
   * CMSIS-RTOS2 的 osThreadId_t 在本移植层中对应 FreeRTOS TaskHandle_t。
   * 将句柄交给 UART DMA 层后，IDLE ISR 才能精确通知对应任务。
   */
  if (RobotTaskHandle == NULL)
  {
    Error_Handler();
  }

  App_RTOS_BindTaskHandles((TaskHandle_t)IMUTaskHandle,
                           (TaskHandle_t)CameraTaskHandle,
                           (TaskHandle_t)ServoTaskHandle,
                           (TaskHandle_t)DebugTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_IMU_Task */
/**
  * @brief  Function implementing the IMUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

