/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void setTimestamp(uint32_t * time);
bool timeElapsed(uint32_t * time, uint32_t elapsed);
uint32_t getTimestamp();
void modifyTargetSteps(int change);
void setTargetSteps(int target);
void zeroSteps();
void processCanMessage();


#define QUEUE_SIZE 5


typedef struct
{
	uint8_t head;
	uint8_t tail;
	CAN_RAD_MESSAGE queue[QUEUE_SIZE];
} CAN_QUEUE;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 0

#define STEP_PIN_PIN_PERIOD_MS 2



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;


int steps;
int targetSteps;
bool dir;
bool inverted;
bool reachedTarget;

uint32_t pinTime;

CAN_QUEUE queue;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];


enum PIN_STATE
{
	PIN_STATE_IDLE = 0,
	PIN_STATE_STEP_HIGH,
	PIN_STATE_STEP_HIGH_IDLE,
	PIN_STATE_DIR_HIGH,
	PIN_STATE_DIR_HIGH_IDLE,
	PIN_STATE_STEP_LOW,
	PIN_STATE_STEP_LOW_IDLE,
	PIN_STATE_DIR_LOW,
	PIN_STATE_DIR_LOW_IDLE
} pinState = PIN_STATE_IDLE;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (queue.head != queue.tail)
	  {
		  //queue is not empty
		  processCanMessage();

		  //erase message, increment head
		  memset(&queue.queue[queue.head], 0, sizeof(CAN_RAD_MESSAGE));
		  queue.head = (queue.head + 1) % QUEUE_SIZE;
	  }


	  //SWAP DIR IF NEEDED
	  static enum DIR_PIN_STATE
	  {
		DIR_PIN_STATE_IDLE = 0,
		DIR_PIN_STATE_HIGH,
		DIR_PIN_STATE_LOW
	  } dirPinState = DIR_PIN_STATE_IDLE;

	  switch (dirPinState)
	  {
		  case DIR_PIN_STATE_IDLE:

			  if ((steps < targetSteps) && (dir != DIRECTION_FORWARD))
			  {
				  //need to set DIR pin forward
				  dirPinState = DIR_PIN_STATE_HIGH;
			  }
			  else if ((steps > targetSteps) && (dir != DIRECTION_BACKWARD))
			  {
				  dirPinState = DIR_PIN_STATE_LOW;
			  }

			  break;

		  case DIR_PIN_STATE_HIGH:
			  //set GPIO
			  dir = DIRECTION_FORWARD;
			  dirPinState = DIR_PIN_STATE_IDLE;
			  break;

		  case DIR_PIN_STATE_LOW:
			  //set GPIO;
			  dir = DIRECTION_BACKWARD;
			  dirPinState = DIR_PIN_STATE_IDLE;
			  break;

		  default:
			  break;

	  }

	  //STEP MOTOR
	  static enum STEP_PIN_STATE
	  {
	  	STEP_PIN_STATE_IDLE = 0,
	  	STEP_PIN_STATE_HIGH,
	  	STEP_PIN_STATE_STEP_HIGH_IDLE,
	  	STEP_PIN_STATE_HIGH_IDLE,
	  	STEP_PIN_STATE_LOW,
	  	STEP_PIN_STATE_LOW_IDLE,
	  } stepPinState = STEP_PIN_STATE_IDLE;

	  switch (stepPinState)
	  {
	  	  case STEP_PIN_STATE_IDLE:

	  		if (steps == targetSteps)
			{
	  			if (!reachedTarget)
	  			{
	  				reachedTarget = true;
	  			}
			}
	  		else
	  		{
	  			reachedTarget = false;

	  			if (steps < targetSteps)
	  			{
	  				steps = steps + (!inverted ? 1: -1);
	  			}
	  			else if (steps > targetSteps)
	  			{
	  				steps = steps + (!inverted? -1 : 1);
	  			}

	  			stepPinState = STEP_PIN_STATE_HIGH;
	  		}


	  		break;

	  	  case STEP_PIN_STATE_HIGH:
	  		  //set pin to high
	  		  setTimestamp(&pinTime);
	  		  stepPinState = STEP_PIN_STATE_HIGH_IDLE;
	  		  break;

	  	  case STEP_PIN_STATE_HIGH_IDLE:

	  		  //has completed half the period
	  		  if (timeElapsed(&pinTime, STEP_PIN_PIN_PERIOD_MS / 2))
	  		  {
	  			  stepPinState = STEP_PIN_STATE_LOW;
	  		  }
	  		  break;

	  	  case STEP_PIN_STATE_LOW:
			  //set pin to low
			  setTimestamp(&pinTime);
			  stepPinState = STEP_PIN_STATE_LOW_IDLE;
			  break;

		  case STEP_PIN_STATE_LOW_IDLE:

			  //has completed half the period
			  if (timeElapsed(&pinTime, STEP_PIN_PIN_PERIOD_MS / 2))
			  {
				  stepPinState = STEP_PIN_STATE_IDLE;
			  }
			  break;
		  default:
			  break;
	  }


	  if (reachedTarget)
	  {
		  //send can message
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

  CAN_RAD_MESSAGE newMessage;
  newMessage.id.value = RxHeader.ExtId;
  memcpy(newMessage.data, RxData, sizeof(newMessage.data));

  if (queue.head != ((queue.tail + 1) % QUEUE_SIZE))
  {
	  memcpy(&queue.queue[queue.tail], &newMessage, sizeof(CAN_RAD_MESSAGE));
	  queue.tail = (queue.tail + 1) % QUEUE_SIZE;

  }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void setTimestamp(uint32_t * time)
{
	*time = HAL_GetTick();
}

uint32_t getTimestamp()
{
	return HAL_GetTick();
}

bool timeElapsed(uint32_t * time, uint32_t elapsed)
{
	return HAL_GetTick() - elapsed >= *(time);
}

void modifyTargetSteps(int change)
{
	targetSteps = targetSteps + change;
}

void setTargetSteps(int target)
{
	targetSteps = target;
}

void zeroSteps()
{
	targetSteps = 0;
	steps = 0;
}

void processCanMessage()
{
	CAN_RAD_MESSAGE *msg;

	msg = &queue.queue[queue.head];
	switch(msg->id.function)
	{
		case RAD_FUNCTION_IDENTIFIER_STEP_STOP:
			zeroSteps();
			break;
		case RAD_FUNCTION_IDENTIFIER_STEP_CW:
		{
			int newSteps;
			memcpy(&newSteps, msg->data, sizeof(newSteps));
			modifyTargetSteps(newSteps);
		}
	}

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
