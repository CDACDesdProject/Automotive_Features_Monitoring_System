/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart5;
BMP280_HandleTypedef bmp;
CAN_RxHeaderTypeDef txHeader;  // To send CAN message header
CAN_RxHeaderTypeDef rxHeader;  // To store received CAN message header

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IS_SENDER 1      // Change to 0 for Receiver board
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t txData[8];
uint8_t rxData[8];              // Buffer to store received data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
#if IS_SENDER
static void MX_I2C2_Init(void);
#endif
static void MX_UART5_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void CAN_SendData(int32_t temperature, int32_t pressure);
void BMP280_Init(void);
void Read_BMP280_Data(void);
#if IS_SENDER
void I2C_Scan(void);
#endif
void OLED_I2C_Scan(void);
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
  MX_I2C3_Init();
  MX_CAN1_Init();
  MX_UART5_Init();

  /* USER CODE BEGIN 2 */
  CAN_filterConfig();
#if IS_SENDER
  MX_I2C2_Init();
  BMP280_Init();
  I2C_Scan();

    bmp.i2c = &hi2c2;
    bmp.dev_id = 0x76; // BMP280 I2C address

    if (bmp280_read_calibration(&bmp) != HAL_OK) {
    		  printf("BMP280 calibration failed!\n");
    		  return -1;
    	  }
#else
    OLED_I2C_Scan();

   if(SSD1306_Init ())
   {
    SSD1306_GotoXY(0, 10);
    SSD1306_Puts("OLED", &Font_11x18, 1);
    SSD1306_UpdateScreen();
    printf("OLED Initialized\n");
   }
   else
   {
	   printf("OLED not Initialized\n");
   }
#endif
  /* USER CODE END 2 */
printf("hello");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	#if IS_SENDER  // If this board is configured as the sender

		// Declare variables for temperature and pressure data
		int32_t raw_temp, raw_pres, temp, pres;

		// Read raw temperature and pressure values from BMP280 sensor
		if (bmp280_read_uncomp_data(&bmp, &raw_temp, &raw_pres) == HAL_OK)
		{

			// Convert raw temperature and pressure data into human-readable format (°C, hPa)
			temp = bmp280_comp_temp_32bit(&bmp, raw_temp);
			pres = bmp280_comp_pres_32bit(&bmp, raw_pres);

		// Print the formatted temperature and pressure values to the console
			printf("Temperature: %ld.%ld°C, Pressure: %ld.%ld hPa\n",
				temp / 100, temp % 100,  // Converts temp into integer.decimal format
				pres / 100, pres % 100); // Converts pressure into integer.decimal format
		}
		else
		{
			// Print error message if sensor data cannot be read
			printf("Error reading BMP280 sensor data!\n");
		}

		// Send the temperature and pressure data via CAN bus to the receiver
		CAN_SendData(temp, pres);

		// Wait before reading and sending the next set of data
		for(int i = 0; i<100000; i++);

	#else  // If this board is configured as the receiver
		// The receiver does nothing except waiting for incoming CAN messages
		for(int i = 0; i<100000; i++);

	#endif

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */

#if IS_SENDER
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif
/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
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

}

/* USER CODE BEGIN 4 */
/* CAN Filter Configuration */
void CAN_filterConfig(void) {
	// Enter Filter Initialization Mode
	CAN1->FMR |= CAN_FMR_FINIT;

	// Configure Filter Bank 0
	CAN1->FA1R &= ~(1 << 0);  // Deactivate Filter Bank 0 before modifying

	// Set Filter Mode and Scale
	CAN1->FS1R |= (1 << 0);   // Set Filter 0 to 32-bit mode
	CAN1->FM1R &= ~(1 << 0);  // Set Filter 0 to ID Mask Mode

	//  Configure Filter to Accept CAN ID 0x244 and 0x245
	CAN1->sFilterRegister[0].FR1 = (0x244 << 5); //Set filter ID
	CAN1->sFilterRegister[0].FR2 = (0xFFF << 5); //Set filter mask
	CAN1->FFA1R &= ~(1 << 0);  // Assign Filter 0 to FIFO0
	CAN1->FA1R |= (1 << 0);    //Assign filter bank 0

	//  Exit Filter Initialization Mode
	CAN1->FMR &= ~CAN_FMR_FINIT;

	// Start CAN in Normal Mode
	CAN1->MCR &= ~CAN_MCR_SLEEP;  // Exit Sleep Mode
	CAN1->MCR &= ~CAN_MCR_INRQ;   // Exit Initialization Mode

	// Enable CAN RX Interrupt for FIFO0
	CAN1->IER |= CAN_IER_FMPIE0;
}

void CAN_SendData(int32_t temperature, int32_t pressure) {
     uint8_t canData[8];

     // Copy temperature and pressure values into the data array
     memcpy(&canData[0], &temperature, sizeof(int32_t));
     memcpy(&canData[4], &pressure, sizeof(int32_t));

     // Check if the TX mailbox is empty
     if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
         // Load the data into the mailbox
         CAN1->sTxMailBox[0].TDLR = *(uint32_t*)&canData[0];  // First 4 bytes
         CAN1->sTxMailBox[0].TDHR = *(uint32_t*)&canData[4];  // Last 4 bytes

         // Set Standard Identifier (StdId = 0x244)
         CAN1->sTxMailBox[0].TIR = (0x244 << 21);  // Standard ID (Bit 21:31)

         // Set DLC (Data Length Code) to 8 bytes
         CAN1->sTxMailBox[0].TDTR = 8;

         // Request transmission
         CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

         // Wait for successful transmission
         printf("CAN Transmission SUCCESS\n");
     } else {
         printf("CAN Transmission FAILED - No Empty Mailbox\n");
     }
 }



/* CAN Receive Callback */
void CAN1_RX0_IRQHandler(CAN_HandleTypeDef *hcan1) {
 	uint8_t rxData[8];
 	int32_t receivedTemperature, receivedPressure;
 	char uartBuffer[50];  // Buffer to store formatted UART data
    char OLED_BUFF[50]={0};   // Buffer to store formatted OLED data
 	// Check if there are messages in FIFO0
 	if (CAN1->RF0R & CAN_RF0R_FMP0) {  // FIFO Message Pending

     	// Read the message from FIFO0 (Data and Header)
     	uint32_t dlc = CAN1->sFIFOMailBox[0].RDTR & 8;  // DLC

     	if (dlc == 8) {
         	// Copy the 8-byte data from FIFO0
         	rxData[0] = CAN1->sFIFOMailBox[0].RDLR & 0xFF;
         	rxData[1] = (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
         	rxData[2] = (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
         	rxData[3] = (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

         	rxData[4] = CAN1->sFIFOMailBox[0].RDHR & 0xFF;
         	rxData[5] = (CAN1->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
         	rxData[6] = (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
         	rxData[7] = (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

         	// Copy the data into variables
         	memcpy(&receivedTemperature, &rxData[0], sizeof(int32_t));
         	memcpy(&receivedPressure, &rxData[4], sizeof(int32_t));

         	// Print CAN Data to Debug Console
         	printf("Received CAN Data:\n");
         	printf("Temperature: %ld.%ld°C, Pressure: %ld.%ld hPa\n",
                	receivedTemperature / 100, receivedTemperature % 100,
                	receivedPressure / 100, receivedPressure % 100);
         	bzero(OLED_BUFF,sizeof(OLED_BUFF));
         	// Print CAN Data to OLED(SSD1306) Console
         	snprintf(OLED_BUFF,sizeof(OLED_BUFF),"T:%ld.%ld C",
         	         receivedTemperature / 100, receivedTemperature % 100);
         	//SSD1306_Clear();
         	SSD1306_GotoXY(10, 10);
         	SSD1306_Puts(OLED_BUFF,&Font_11x18, 1);
         	snprintf(OLED_BUFF,sizeof(OLED_BUFF),"P: %ld.%ld hPa",
         	         receivedPressure / 100, receivedPressure % 100);
         	SSD1306_GotoXY(10, 30);
         	SSD1306_Puts(OLED_BUFF,&Font_11x18, 1);
         	//SSD1306_ScrollRight(0x00, 0x07);    // scroll entire screen (Page0 to Page7) right
         	//HAL_Delay (2000);
         	//SSD1306_Stopscroll();
         	SSD1306_UpdateScreen();

         	// Format Data for UART Transmission
         	snprintf(uartBuffer, sizeof(uartBuffer), "T:%ld.%ldC P:%ld.%ldhPa\n",
                  	receivedTemperature / 100, receivedTemperature % 100,
                  	receivedPressure / 100, receivedPressure % 100);

         	// Send Data over UART5 to ESP32
         	HAL_UART_Transmit(&huart5, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
         	printf("Sent to ESP32: %s", uartBuffer);
     	}

     	// Clear FIFO0 message pending flag
     	CAN1->RF0R |= CAN_RF0R_RFOM0;  // Release the FIFO0 occupied message object
 	} else {
     	printf("No message in FIFO0\n");
 	}
 }

/* BMP280 Initialization */
void BMP280_Init(void)
{

    HAL_StatusTypeDef result;

    // Check if the device responds at its I2C address
    result = HAL_I2C_IsDeviceReady(bmp.i2c, bmp.dev_id << 1, 3, 100);
    if (result != HAL_OK)
    {
        printf("BMP280 not found at I2C address 0x%x!\n", bmp.dev_id);
        return;
    }

    // Initialistatic void MX_I2C2_Init(void)ze the BMP280 with normal mode, oversampling x4 for both temperature and pressure
    uint8_t config[2];
    config[0] = BMP280_REG_CTRL_MEAS;
    config[1] = 0x54;  // Normal mode, oversampling x4 for temp & pressure
    if (HAL_I2C_Master_Transmit(bmp.i2c, bmp.dev_id << 1, config, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Error configuring BMP280!\n");
    }
    else
    {
        printf("BMP280 initialization successful!\n");
    }
}

/* I2C Scan Function */
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

void OLED_I2C_Scan(void)
{
    printf("Scanning I2C bus for devices...\n");
    uint8_t count=0;
    for (uint8_t addr = 0; addr < 128; addr++)
    {
        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 1, 100);
        if (result == HAL_OK)
        {
            printf("I2C device found at address 0x%02X\n", addr);
            printf("I2C scan complete.\n");
            return ;

        }
        else
        {
        	printf("%d -BMP280 not found at I2C address 0x%x!\n",count++, addr);
        }
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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */

