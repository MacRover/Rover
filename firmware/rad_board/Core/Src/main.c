/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor driver chip enable
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // disable stand alone mode
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS

    int32_t SPImsg = 0;
    uint8_t SPImsg_bytes[3];

    //  uint32_t SPIread;
    uint8_t SPIread_bytes[3];

    HAL_StatusTypeDef spi_status;

    // Bits 19, 18, 17
    // 111 DRVCONF (0b111 << 17)
    // 110 SGCSCONF (0b110 << 17)
    // 101 SMARTEN (0b101 << 17)
    // 100 CHOPCONF (0b100 << 17)
    // 00X DRVCTRL (0b00 << 18)

    /* DRVCONF */
    SPImsg = 0;
    SPImsg |= (0b111 << 17); // DRVCONF
    SPImsg |= (0b0 << 16); // TST: test mode
    SPImsg |= (0b11110 << 11); // SLP: Slope control
    SPImsg |= (0b0 << 10); // DIS_S2G: Short to ground protection
    SPImsg |= (0b00 << 8); // TS2G: Short detection delay
    SPImsg |= (0b0 << 7); // SDOFF: step/dir interface
    SPImsg |= (0b0 << 6); // VSENSE: full-scale sense resistor voltage setting
    SPImsg |= (0b11 << 4); // RDSEL: read out select
    SPImsg |= (0b0 << 3); // OTSENS: overtemp shutdown setting
    SPImsg |= (0b1 << 2); // SHRTSENS: short to ground sensitivity
    SPImsg |= (0b1 << 1); // EN_PFD: passive fast delay setting
    SPImsg |= (0b1 << 0); // EN_S2VS: Short to VS protection

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* SGCSCONF */
    SPImsg = 0;
    SPImsg |= (0b110 << 17); // SGCSCONF
    SPImsg |= (0b0 << 16); // SFILT: stall guard filter
    SPImsg |= (0b0000010 << 8); // SGT: stall guard threshold
    SPImsg |= (17 << 0); // CS: current scale

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* SMARTEN */
    SPImsg = 0;
    SPImsg |= (0b101 << 17); // SMARTEN
    SPImsg |= (0b0 << 15); // SEIMIN: min cool step current
    SPImsg |= (0b00 << 13); // SEDN: current dec. speed
    SPImsg |= (0b0000 << 8); // SEMAX: upper cool step threshold offset
    SPImsg |= (0b00 << 5); // SEUP: current increment size
    SPImsg |= (0b0000 << 0); // SEMIN: cool step lower threshold

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* CHOPCONF */
    SPImsg = 0;
    SPImsg |= (0b100 << 17); // CHOPCONF
    SPImsg |= (0b10 << 15); // TBL: blanking time
    SPImsg |= (0b0 << 14); // CHM: chopper mode
    SPImsg |= (0b0 << 13); // RNDTF: Random TOFF time
    SPImsg |= (0b00 << 11); // HDEC: hysteresis decay or fast decay mode
    SPImsg |= (0b0100 << 7); // HEND: hysteresis end value
    SPImsg |= (0b110 << 4); // HSTRT: hysteresis start value
    SPImsg |= (0b0100 << 0); // TOFF: mosfet off time

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* DRVCTRL */
    SPImsg = 0;
    SPImsg |= (0b00 << 18); // DRVCTRL
    SPImsg |= (0b0 << 9); // INTPOL: step interpolation
    SPImsg |= (0b0 << 8); // DEDGE: Double edge step pulses
    SPImsg |= (0b0111 << 0); // MRES: Microsteps per fullstep

    SPImsg_bytes[2] = (uint8_t) (SPImsg & 0xFF);
    SPImsg_bytes[1] = (uint8_t) ((SPImsg & 0xFF00) >> 8);
    SPImsg_bytes[0] = (uint8_t) ((SPImsg & 0xFF0000) >> 16);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    spi_status = HAL_SPI_TransmitReceive(&hspi1, SPImsg_bytes, SPIread_bytes, 3,
            1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Use LED to signal state of ST_TST
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,
                HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
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
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA,
            DRIVER_CS_Pin | DRIVER_ENN_Pin | DRIVER_ST_ALONE_Pin,
            GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DRIVER_CS_Pin DRIVER_ENN_Pin DRIVER_ST_ALONE_Pin */
    GPIO_InitStruct.Pin = DRIVER_CS_Pin | DRIVER_ENN_Pin | DRIVER_ST_ALONE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : LED_RED_Pin */
    GPIO_InitStruct.Pin = LED_RED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DRIVER_SG_TEST_Pin */
    GPIO_InitStruct.Pin = DRIVER_SG_TEST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DRIVER_SG_TEST_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : DRIVER_STEP_Pin DRIVER_DIR_Pin */
    GPIO_InitStruct.Pin = DRIVER_STEP_Pin | DRIVER_DIR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
