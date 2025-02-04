/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for bmp280
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
#include "main.h"
#include "bmp280.h"
#include "stdio.h"

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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
BMP280_HandleTypedef bmp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void BMP280_Init(void);
void Read_BMP280_Data(void);
void I2C_Scan(void);
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  I2C_Scan(); // Scan I2C devices

  BMP280_Init();  // Initialize BMP280
  /* USER CODE END 2 */
  bmp.i2c = &hi2c2;
  bmp.dev_id = 0x76; // BMP280 I2C address

  if (bmp280_read_calibration(&bmp) != HAL_OK) {
      printf("BMP280 calibration failed!\n");
      return -1;
  }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int32_t raw_temp, raw_pres, temp, pres;

      while (1) {
          if (bmp280_read_uncomp_data(&bmp, &raw_temp, &raw_pres) == HAL_OK) {
              temp = bmp280_comp_temp_32bit(&bmp, raw_temp);
              pres = bmp280_comp_pres_32bit(&bmp, raw_pres);

              printf("Temperature: %ld.%02ld°C, Pressure: %ld.%02ld hPa\n",
                     temp / 100, temp % 100,
                     pres / 100, pres % 100);
          } else {
              printf("Error reading BMP280 sensor data!\n");
          }

          HAL_Delay(1000); // Wait for 1 second
      }
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
I2C_HandleTypeDef hi2c2;


void MX_I2C2_Init(void) {
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;  // 100 kHz for example
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief Initialize the BMP280 sensor
  * @retval None
  */
void BMP280_Init(void)
{
    bmp280.i2c = &hi2c2;
    bmp280.dev_id = BMP280_I2C_ADDR_PRIM; // 0x76
    HAL_StatusTypeDef result;

    // Check if the device responds at its I2C address
    result = HAL_I2C_IsDeviceReady(bmp280.i2c, bmp280.dev_id << 1, 3, 100);
    if (result != HAL_OK)
    {
        printf("BMP280 not found at I2C address 0x%x!\n", bmp280.dev_id);
        return;
    }

    // Initialize the BMP280 with normal mode, oversampling x4 for both temperature and pressure
    uint8_t config[2];
    config[0] = BMP280_REG_CTRL_MEAS;
    config[1] = 0x54;  // Normal mode, oversampling x4 for temp & pressure
    if (HAL_I2C_Master_Transmit(bmp280.i2c, bmp280.dev_id << 1, config, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Error configuring BMP280!\n");
    }
    else
    {
        printf("BMP280 initialization successful!\n");
    }

}

/**
  * @brief Read and print data from BMP280
  * @retval None
  */
/*void Read_BMP280_Data(void)
{
  int32_t temperature, pressure;

  if (bmp280_read_uncomp_data(&bmp280, &temperature, &pressure) == BMP280_OK)
  {
    temperature = bmp280_comp_temp_32bit(&bmp280, temperature);
    pressure = bmp280_comp_pres_32bit(&bmp280, pressure);

    printf("Temperature: %.2f°C, Pressure: %.2fhPa\n",
           temperature / 100.0, pressure / 100.0);
  }
  else
  {
    printf("Error reading data from BMP280!\n");
  }
}*/

/**
  * @brief Scan for I2C devices and print addresses
  * @retval None
  */
void I2C_Scan(void)
{
    printf("Scanning I2C bus for devices...\n");

    for (uint8_t addr = 0; addr < 128; addr++)
    {
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 100);
        if (result == HAL_OK)
        {
            printf("I2C device found at address 0x%02X\n", addr);
        }
    }
    printf("I2C scan complete.\n");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
