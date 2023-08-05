/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "LCD_interface.h"
#include "LED_interface.h"
#include "queue.h"

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

extern GPIO_ConfigType ButtonPin1;
extern LED_ConfigType  LedPin1;

extern GPIO_ConfigType ButtonPin2;
extern LED_ConfigType  LedPin2;


/* USER CODE END Variables */
/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task2 */
osThreadId_t task2Handle;
const osThreadAttr_t task2_attributes = {
  .name = "task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void vTaskOne(void *argument);
void vTaskTwo(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);

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
  /* creation of task1 */
  task1Handle = osThreadNew(vTaskOne, NULL, &task1_attributes);

  /* creation of task2 */
  task2Handle = osThreadNew(vTaskTwo, NULL, &task2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */



  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vTaskOne */
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vTaskOne */
void vTaskOne(void *argument)
{
  /* USER CODE BEGIN vTaskOne */

	uint8_t bState;

  /* Infinite loop */
	for(;;)
	{
		bState = MGPIO_u8ReadPortPin(&ButtonPin1);
		if(GPIO_HIGH == bState)
		{
			lcd_4bit_send_string_pos(&lcd_1, 1, 1, "TaskOneRun");
			LED_vSetState(&LedPin1, LED_ON);
		}
		else if(GPIO_LOW == bState)
		{
			lcd_4bit_send_string_pos(&lcd_1, 1, 1, "NotTaskOne");
			LED_vSetState(&LedPin1, LED_OFF);
		}

		osSemaphoreRelease(myBinarySem01Handle);
	}
  /* USER CODE END vTaskOne */
}

/* USER CODE BEGIN Header_vTaskTwo */
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskTwo */
void vTaskTwo(void *argument)
{
  /* USER CODE BEGIN vTaskTwo */

	uint8_t bState;

  /* Infinite loop */
	for(;;)
	{
		bState = MGPIO_u8ReadPortPin(&ButtonPin2);
		if(GPIO_HIGH == bState)
		{
			lcd_4bit_send_string_pos(&lcd_1, 2, 1, "TaskTwoRun");
			LED_vSetState(&LedPin2, LED_ON);
		}
		else if(GPIO_LOW == bState)
		{
			lcd_4bit_send_string_pos(&lcd_1, 2, 1, "NotTaskTwo");
			LED_vSetState(&LedPin2, LED_OFF);
		}

		osSemaphoreAcquire(myBinarySem01Handle, HAL_MAX_DELAY);
	}
  /* USER CODE END vTaskTwo */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Task to be created. */


/* USER CODE END Application */

