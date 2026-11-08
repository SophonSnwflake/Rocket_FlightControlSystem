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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for Rocket */
osThreadId_t RocketHandle;
uint32_t RocketBuffer[ 3072 ];
osStaticThreadDef_t RocketControlBlock;
const osThreadAttr_t Rocket_attributes = {
  .name = "Rocket",
  .cb_mem = &RocketControlBlock,
  .cb_size = sizeof(RocketControlBlock),
  .stack_mem = &RocketBuffer[0],
  .stack_size = sizeof(RocketBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GNSS */
osThreadId_t GNSSHandle;
uint32_t GNSSBuffer[ 512 ];
osStaticThreadDef_t GNSSControlBlock;
const osThreadAttr_t GNSS_attributes = {
  .name = "GNSS",
  .cb_mem = &GNSSControlBlock,
  .cb_size = sizeof(GNSSControlBlock),
  .stack_mem = &GNSSBuffer[0],
  .stack_size = sizeof(GNSSBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
uint32_t IMUBuffer[ 1024 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .stack_mem = &IMUBuffer[0],
  .stack_size = sizeof(IMUBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LOG */
osThreadId_t LOGHandle;
uint32_t LOGBuffer[ 512 ];
osStaticThreadDef_t LOGControlBlock;
const osThreadAttr_t LOG_attributes = {
  .name = "LOG",
  .cb_mem = &LOGControlBlock,
  .cb_size = sizeof(LOGControlBlock),
  .stack_mem = &LOGBuffer[0],
  .stack_size = sizeof(LOGBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LORA */
osThreadId_t LORAHandle;
uint32_t LORABuffer[ 512 ];
osStaticThreadDef_t LORAControlBlock;
const osThreadAttr_t LORA_attributes = {
  .name = "LORA",
  .cb_mem = &LORAControlBlock,
  .cb_size = sizeof(LORAControlBlock),
  .stack_mem = &LORABuffer[0],
  .stack_size = sizeof(LORABuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void rocket_task(void *argument);
void GNSS_task(void *argument);
void imu_task(void *argument);
void log_task(void *argument);
void lora_task(void *argument);

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
  /* creation of Rocket */
  RocketHandle = osThreadNew(rocket_task, NULL, &Rocket_attributes);

  /* creation of GNSS */
  GNSSHandle = osThreadNew(GNSS_task, NULL, &GNSS_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(imu_task, NULL, &IMU_attributes);

  /* creation of LOG */
  LOGHandle = osThreadNew(log_task, NULL, &LOG_attributes);

  /* creation of LORA */
  LORAHandle = osThreadNew(lora_task, NULL, &LORA_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_rocket_task */
/**
  * @brief  Function implementing the Rocket thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_rocket_task */
__weak void rocket_task(void *argument)
{
  /* USER CODE BEGIN rocket_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rocket_task */
}

/* USER CODE BEGIN Header_GNSS_task */
/**
* @brief Function implementing the GNSS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GNSS_task */
__weak void GNSS_task(void *argument)
{
  /* USER CODE BEGIN GNSS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GNSS_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void *argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_log_task */
/**
* @brief Function implementing the LOG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_log_task */
__weak void log_task(void *argument)
{
  /* USER CODE BEGIN log_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END log_task */
}

/* USER CODE BEGIN Header_lora_task */
/**
* @brief Function implementing the LORA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lora_task */
__weak void lora_task(void *argument)
{
  /* USER CODE BEGIN lora_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lora_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

