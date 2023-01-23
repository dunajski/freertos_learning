/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include "lis3dh_example.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
  #define BIT_0  ( 1 << 0 )
  #define BIT_1  ( 1 << 1 )
  #define BIT_2  ( 1 << 2 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t blinkingRatioThreadHandle;
const osThreadAttr_t blinkingRatioTask_attributes = {
  .name = "ratioTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
osThreadId_t toggleLEDThreadHandle;
const osThreadAttr_t toggleLEDTask_attributes = {
  .name = "toggleLEDTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

static uint32_t blinking_ratio = 250; // 250 ms if OS tick is equal 1 ms
osThreadId_t sendByteOverUartHandle;
const osThreadAttr_t sendByteOverUartTask_attributes = {
  .name = "sendUARTbyte",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
static uint8_t rec_character;

// accel - accelerometer
osThreadId_t accelServiceHandle;
const osThreadAttr_t accelServiceTask_attributes = {
  .name = "accelerometer service",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4 * 2
};

static accel_state_t accel_state = ACC_INIT;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ChangeBlinkingRatioThread(void *argument);
static void ToggleLEDThread(void *argument);
static void SendBytOverUartThread(void *argument);
static void AccelServiceThread(void * argument);
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
  HAL_UART_Receive_IT(&huart2, &rec_character, sizeof(rec_character));
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
  blinkingRatioThreadHandle = osThreadNew(ChangeBlinkingRatioThread, NULL, &blinkingRatioTask_attributes);
  toggleLEDThreadHandle = osThreadNew(ToggleLEDThread, NULL, &toggleLEDTask_attributes);
  sendByteOverUartHandle = osThreadNew(SendBytOverUartThread, NULL, &sendByteOverUartTask_attributes);
  accelServiceHandle = osThreadNew(AccelServiceThread, NULL, &accelServiceTask_attributes);
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
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BLUE_BUTTON_Pin)
  {
    osThreadFlagsSet(blinkingRatioThreadHandle, BIT_1);
    osThreadFlagsSet(sendByteOverUartHandle, BIT_1);

  }
}

static void ToggleLEDThread(void * argument)
{
  (void) argument;
  for (;;)
  {
    osDelay(blinking_ratio);
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  }
}

static void ChangeBlinkingRatio(void)
{
  if (blinking_ratio == 250)
    blinking_ratio = 1000;
  else
    blinking_ratio = 250;
}

static void ChangeBlinkingRatioThread(void * argument)
{
  (void) argument;
  for(;;)
  {
    osThreadFlagsWait(BIT_1, osFlagsWaitAny, osWaitForever);

    ChangeBlinkingRatio();
  }
}

const uint8_t button_press_str[] = "Button pressed\r\n";
const uint8_t data_rec_str[] = "received sth\r\n";
const uint8_t data_rec_e_str[] = "received e\r\n";

static void SendBytOverUartThread(void *argument)
{
  (void) argument;
  uint32_t events = 0;
  for (;;)
  {
    events = osThreadFlagsWait(BIT_0 | BIT_1, osFlagsWaitAny, osWaitForever);

    switch (events)
    {
      case BIT_0:
        if (rec_character == 'e')
          HAL_UART_Transmit(&huart2, data_rec_e_str, sizeof(data_rec_e_str), 500);
        else
          HAL_UART_Transmit(&huart2, data_rec_str, sizeof(data_rec_str), 500);
        break;
      case BIT_1:
        HAL_UART_Transmit(&huart2, button_press_str, sizeof(button_press_str), 500);
        break;
      default:
        break;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    osThreadFlagsSet(sendByteOverUartHandle, BIT_0);
    HAL_UART_Receive_IT(&huart2, &rec_character, sizeof(rec_character));
  }
}

static void AccelMachineState(void)
{
  switch (accel_state)
  {
  case ACC_RUNNING:
    if (lis3dh_is_samples_ready())
      lis3dh_read_fifo();
    else
      osDelay(100);
  break;
  case ACC_INIT:
  case ACC_ERROR:
  default:
  break;
  }
}

static void AccelServiceThread(void *argument)
{
  (void) argument;
  accel_state = lis3dh_init();
  for (;;)
  {
    AccelMachineState();
  }

}
/* USER CODE END Application */

