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
#include "stdio.h"
#include "string.h"
#include "bmp280.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart5;
BMP280_HandleTypedef bmp;
CAN_RxHeaderTypeDef txHeader;  // To send CAN message header
CAN_RxHeaderTypeDef rxHeader;  // To store received CAN message header

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IS_SENDER 0      // Change to 0 for Receiver board
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
void MX_GPIO_Init(void);
void MX_CAN1_Init(void);
void MX_I2C2_Init(void);
void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void CAN_SendData(int32_t temperature, int32_t pressure);
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
	MX_CAN1_Init();
	CAN_filterConfig();
	MX_UART5_Init();
#if IS_SENDER
    MX_I2C2_Init();
    BMP280_Init();
    I2C_Scan();
    /* USER CODE BEGIN 2 */
    bmp.i2c = &hi2c2;
    bmp.dev_id = 0x76; // BMP280 I2C address
    /* USER CODE END 2 */

	if (bmp280_read_calibration(&bmp) != HAL_OK) {
		  printf("BMP280 calibration failed!\n");
		  return -1;
	  }

#endif
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


/* CAN Initialization */
void MX_CAN1_Init(void)
{
	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */

	hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;  // Changed from LOOPBACK to NORMAL
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    	  printf("CANerror");
        Error_Handler();

    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */
}


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

 /* GPIO Initialization */
void MX_GPIO_Init(void) {
     __HAL_RCC_GPIOA_CLK_ENABLE();
     __HAL_RCC_GPIOB_CLK_ENABLE();
 }

/* I2C Initialization */
void MX_I2C2_Init(void) {
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = DISABLE;
    hi2c2.Init.NoStretchMode = DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

/* UART5 Initialization */
void MX_UART5_Init(void)
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

/* System Clock Configuration */
void SystemClock_Config(void) {
    // Assume pre-configured clock settings
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {

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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
