/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void setTimestamp(uint32_t * time);
bool timeElapsed(uint32_t * time, uint32_t elapsed);
uint32_t getTimestamp();
void CAN_SendMessage(CAN_RAD_MESSAGE * msg);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//MANUAL RAD BOARD ID ASSIGN

#define RAD_BOARD_ID 1

//AUTOMATIC RAD BOARD ID ASSIGN

//#define RAD_BOARD_ID radBoardId;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef   RxHeader;

uint8_t               RxData[8];

CAN_TxHeaderTypeDef   TxHeader;

uint8_t               TxData[8];

uint32_t              TxMailbox;

uint32_t ledTimer;
uint64_t ledBlinks;
uint16_t rotations;

uint32_t testTimer;

char uart_buf[50];


volatile uint8_t uart3Flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
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
  //MX_GPIO_Init();
  MX_CAN_Init();
  //MX_TIM2_Init();
  //MX_USART3_UART_Init();
  //MX_SPI2_Init();
  //MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

//  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//  {
//	  Error_Handler();
//  }

  setTimestamp(&testTimer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  static enum
	  {
		  LED_STATE_INIT = 0,
		  LED_STATE_IDLE_LOW,
		  LED_STATE_HIGH,
		  LED_STATE_IDLE_HIGH,
		  LED_STATE_LOW
	  } ledState = LED_STATE_INIT;

	  switch (ledState)
	  {
	  	  case LED_STATE_INIT:
	  		  setTimestamp(&ledTimer);
			  ledState = LED_STATE_IDLE_LOW;
			  break;

	  	  case LED_STATE_IDLE_LOW:
	  		  //1 second timeout

	  		  if (timeElapsed(&ledTimer, 1000) && ledBlinks > 0)
	  		  {
	  			  ledBlinks--;
	  			  ledState = LED_STATE_HIGH;
	  		  }
	  		  break;

	  	  case LED_STATE_HIGH:
	  		  //Turn on LED
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  		  setTimestamp(&ledTimer);
	  		  ledState = LED_STATE_IDLE_HIGH;
	  		  break;

	  	  case LED_STATE_IDLE_HIGH:
	  		  if (timeElapsed(&ledTimer, 1000))
			  {
				  ledState = LED_STATE_LOW;
			  }
			  break;

	  	  case LED_STATE_LOW:
			  //Turn off LED
	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			  setTimestamp(&ledTimer);
			  ledState = LED_STATE_IDLE_LOW;
			  break;

	  	  default:
	  		  ledState = LED_STATE_INIT;
	  		  break;
	  }


	  static enum {
		  TEST_BLINK = 0,
		  TEST_MOTOR,
		  TEST_UART,
		  TEST_CAN,
		  TEST_SPI,
		  TEST_I2C,
		  TEST_LIMIT_SWITCH,
		  TEST_FORCE_RESISTOR
	  } testState = TEST_CAN;

	  switch (testState)
	  {
	  	  case TEST_BLINK:
		  	  ledBlinks = 1;
		  	  break;

	  	  case TEST_MOTOR:


	  		uint16_t MotorTestRead = 0x0000;
	  		//HAL_UART_Receive_IT(&huart3, (uint8_t*)&MotorTestRead, sizeof(MotorTestRead));
	  		//if (uart3Flag)
	  		if (timeElapsed(&testTimer, 5000))
			{
				rotations = 100;//(uint16_t)MotorTestRead;
				ledBlinks = 1;
				if (uart3Flag){
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
					uart3Flag = 0;
				}
				else
				{
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
					uart3Flag = 1;
				}


				//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

				//HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
				int buf_len = sprintf(uart_buf, "PWM Test: 0x%x rotations\n", MotorTestRead);

				HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);
				setTimestamp(&testTimer);

			}

	  		  break;

	  	  case TEST_UART:

	  		uint16_t UART_read = 0x0000;
	  		HAL_UART_Receive_IT(&huart3, (uint8_t*)&UART_read, sizeof(UART_read));

	  		//Wait for UART msg to trigger callback, then break it down
	  		if (uart3Flag)
	  		{
	  			ledBlinks = (uint16_t)UART_read;
	  			uart3Flag = 0;

	  			int buf_len = sprintf(uart_buf, "UART Test: 0x%x \n", UART_read);

	  			HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);


	  		}


	  	  case TEST_CAN:
//	  		  if ((uint64_t)RxData > 0)
//	  		  {
//	  			  ledBlinks = (uint64_t)RxData;
//
//	  			  CAN_RAD_MESSAGE msg;
//	  			  msg.id.value = RAD_BOARD_ID;
//	  			  memcpy(msg.data, RxData, 8*sizeof(uint8_t));
//
//	  			  memset(&RxData, 0, 8*sizeof(uint8_t));
//
//
//
//	  			  HAL_CAN_SendMessage(&msg);
//	  		  }

	  		if (timeElapsed(&testTimer, 5000))
		  {

	  			static uint64_t i = 1;

	  		//ledBlinks = (uint64_t)RxData;

//			  CAN_RAD_MESSAGE msg;
//			  msg.id.value = RAD_BOARD_ID;
//			  memcpy(msg.data, &i, 8*sizeof(uint8_t));
//
//			  memset(&RxData, 0, 8*sizeof(uint8_t));

	  			TxHeader.IDE = CAN_ID_STD;
	  			TxHeader.StdId = 0x446;
	  			TxHeader.RTR = CAN_RTR_DATA;
	  			TxHeader.DLC = 2;

	  			TxData[0] = 50;
	  			TxData[1] = 0xAA;


			  //CAN_SendMessage(&msg);

			  uint8_t csend[] = {'H','E','L','L','O'}; // Tx Buffer


			  if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK)
			  	{
				  ledBlinks = 2;
			  	}

			  ledBlinks = 1;
			  i++;
			  getTimestamp(&testTimer);
		  }

	  		break;

	  	  case TEST_SPI:

	  		  if(timeElapsed(&testTimer, 5000))
	  		  {
	  			  uint16_t SPI_msg = 0x0000; //two bytes, 16 bits

				  SPI_msg |= 0x3FFF; //ANGLE REGISTER
				  SPI_msg |= (1 << 14); //READ COMMAND

				  //CALCULATE PARITY - current message is 0x7FFF, or 0b0111 1111 1111 1111 - odd number of bits
				  SPI_msg |= (1 << 15);

				  //SPI MSG is 0xFFFF for angle read

				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				  HAL_SPI_Transmit(&hspi2, (uint8_t *)&SPI_msg, sizeof(SPI_msg), 100);

				  //Not sure if I need this for a subsequent read
				  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
				  //HAL_DELAY
				  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				  //HAL_SPI_Transmit(&hspi2, (uint8_t *)&SPI_msg, sizeof(SPI_msg), timeout);

				  uint16_t SPI_read;

				  HAL_SPI_Receive(&hspi2, (uint8_t*)&SPI_read, sizeof(SPI_read), 100);

				  ledBlinks = 1;

				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

				  int buf_len = sprintf(uart_buf, "SPI Test: 0x%x \n", SPI_msg);

				  HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);

				  //Wait 5s till next test
				  setTimestamp(&testTimer);
	  		  }


	  		 break;

	  	  case TEST_LIMIT_SWITCH:

	  		 if (timeElapsed(&testTimer, 1000))
	  			  		  {
	  			 	 	 	//polling Limit Switch
	  						 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
	  						 {
	  							 int buf_len = sprintf(uart_buf, "Force sensitive Resistor A pressed\n");

	  							 HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);
	  							 getTimestamp(&testTimer);
	  						 }
	  						 if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==GPIO_PIN_SET)
	  						 {
	  							 int buf_len = sprintf(uart_buf, "Force sensitive Resistor B pressed\n");

	  							 HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);
	  							 getTimestamp(&testTimer);

	  						 }
	  			  		  }

				 //polling Limit Switch
			  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
			  {
				int buf_len = sprintf(uart_buf, "Force sensitive Resistor A pressed\n");

				HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);
			  }
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==GPIO_PIN_SET)
			{
			int buf_len = sprintf(uart_buf, "Force sensitive Resistor B pressed\n");

			HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, buf_len, 100);
			}

	  		  break;

	  	  case TEST_FORCE_RESISTOR:

	  		  if (timeElapsed(&testTimer, 1000))
	  		  {

	  		  }
	  		  break;

	  	  default:
	  		  break;


	  }


	  /**
	  static enum
	  {
		  RAD_STATE_INIT = 0,
		  RAD_STATE_IDLE,
		  RAD_STATE_RECEIVE,
		  RAD_STATE_TOGGLE_LED,
		  RAD_STATE_DELAY_AND_TX
		  //Calibration mode?
	  } radState = RAD_STATE_INIT;


	  switch(radState)
	  {

	  	  case RAD_STATE_INIT:
		  {


	  		  //i2c init
	  		  //

	  		  CAN_init();

	  		  setTimestamp(&ledTimer);

	  		  radState = RAD_STATE_IDLE;

	  		  break;
		  }

	  	  case RAD_STATE_IDLE:
		  {

	  		  if (CAN_getMessage(&canMessage) == CAN_OK)
	  		  {
	  			  radState = RAD_STATE_RECEIVE;
	  		  }

	  		  if (ledBlinks != 0 && timeElapsed(&ledTimer, 200))
	  		  {
	  			radState = RAD_STATE_TOGGLE_LED;
	  		  }


	  		  //motor heartbeat - threshold met?
	  		  	  //read encoder
	  		  	  //if encoder_readValue(&val) == ENCODER_OK
	  		  	  //estate send


	  		  //encoder heartbeat - read encoder
	  		  //if encoder_readValue(&val) == ENCODER_OK

	  		  break;
		  }

	  	  case RAD_STATE_RECEIVE:
		  {

			  if (canMessage.id.id == RAD_BOARD_ID)
			  {
				  for (int i = 0; i < 8; i++)
				  {
					  ledBlinks = ledBlinks + canMessage.data[i];
				  }
			  }


//	  		  switch(canMessage.id.id)
//	  		  {
//	  		  	  case (1):
//	  		  			//depending on function, call respective motor setting functions
//	  		  			break;
//	  		  	  default:
//
//	  		  		  //current for test
//	  		  		  for (int i = 0; i < canMessage.data[1]; i++)
//	  		  		  {
//	  		  			  //LED ON
//	  		  			  //DELAY
//	  		  			  //LED OFF
//	  		  			  //DELAY
//	  		  		  }
//
//	  		  		  break;
//
//	  		  }

	  		  radState = RAD_STATE_IDLE;


			  break;

		  }
	  	  case RAD_STATE_TOGGLE_LED:

  			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
  			  setTimestamp(&ledTimer);

  			  if (ledBlinks-- == 0)
  			  {
  				  radState = RAD_STATE_DELAY_AND_TX;
  			  }
  			  else
  			  {
  				  radState = RAD_STATE_IDLE;
  			  }

  			  break;

	  	  case RAD_STATE_DELAY_AND_TX:
	  		  //delay after LED Blinks hits zero, and then send a TX confirmation message

	  		  if (timeElapsed(&ledTimer, 2000))
	  		  {
	  			  setTimestamp(&ledTimer);


	  			  canMessage.id.value = 255;
	  			  canMessage.data[0] = 255;
	  			  HAL_CAN_SendMessage(&canMessage);



	  			  radState = RAD_STATE_IDLE;
	  		  }
	  		  break;

	  	  default:
	  		  radState = RAD_STATE_IDLE;
			  break;
	  }

	**/


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan.Init.Prescaler = 5;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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

  	CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_DISABLE;
    canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x446<<5;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x446<<5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  //set timer counter clock run at 10kHz
  //uint32_t Tim2_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

  //prescaler at 8mHz - going to 1kHz
  uint32_t Tim2_PrescalerValue = (uint32_t) (8000000/1000) - 1;
  //dropping reload from 1kHz to 10Hz - 10 pulses a second
  uint32_t Tim2_PeriodValue = (uint32_t) (Tim2_PrescalerValue/10) - 1;


  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

//    if (RxHeader.IDE == CAN_ID_EXT)
//    {
//    	CAN_addMessage(&(RxHeader.ExtId), RxData);
//    }
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim)
{
	if (htim -> Instance == TIM2)
	{
		if(rotations-- == 0){
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart3Flag = 1;
}

void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
}


void CAN_SendMessage(CAN_RAD_MESSAGE * msg)
{
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.ExtId = msg->id.value;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;

	memcpy(TxData, msg->data, sizeof(TxHeader.DLC));

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

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
