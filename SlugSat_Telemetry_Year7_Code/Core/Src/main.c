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
#include "usb_device.h"
//#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "CC1200.h"
#include "CC1200_Registers.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//RXTransmit function
}
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

//	// CC1200 Transmit / Receive Test
//	// Standard FIFO Access : R/W B 1 1 1 1 1 1
//	// Read=1, Write=0
//	// Burst=1, Single=0;
//	uint8_t ADDRESS_BYTE = 0x3F; // 0 0 1 1 1 1 1 1
//	uint8_t DATA_BYTE = 0xAA;    // 1 0 1 0 1 0 1 0
//	uint8_t CC1200_TEST_PACKET[2] = {ADDRESS_BYTE, DATA_BYTE}; // write data to TX FIFO
//	uint8_t CC1200_STATUS_BYTES[2]; // receive status information

	// CC1200 Functions Test

	CC1200_t CC1200_SPI_Info;
	uint8_t CC1200_Data[3]; // data received from CC1200

	//uint8_t Register_Address = 0x00;
	//uint8_t Register_Address = 0x2F;
	//uint8_t Register_Value = 0xAA;

//	/* array mapping each CC1200 register with a default value */
//	RegisterSetting_t Preferred_Register_Settings[]=
//	{
//	  {CC1200_IOCFG3,              0x06},
//	  {CC1200_IOCFG2,              0x06},
//	  {CC1200_IOCFG1,              0x30},
//	  {CC1200_IOCFG0,              0x3C},
//	  {CC1200_SYNC3,               0x00},
//	  {CC1200_SYNC2,               0x00},
//	  {CC1200_SYNC1,               0xD9},
//	  {CC1200_SYNC0,               0xCC},
//	  {CC1200_SYNC_CFG1,           0x47},
//	  {CC1200_SYNC_CFG0,           0x00},
//	  {CC1200_DEVIATION_M,         0x06},
//	  {CC1200_MODCFG_DEV_E,        0x29},
//	  {CC1200_DCFILT_CFG,          0x4B},
//	  {CC1200_PREAMBLE_CFG1,       0x14},
//	  {CC1200_PREAMBLE_CFG0,       0x8A},
//	  {CC1200_IQIC,                0x6C},
//	  {CC1200_CHAN_BW,             0x83},
//	  {CC1200_MDMCFG1,             0x42},
//	  {CC1200_MDMCFG0,             0x05},
//	  {CC1200_SYMBOL_RATE2,        0x96},
//	  {CC1200_SYMBOL_RATE1,        0xF0},
//	  {CC1200_SYMBOL_RATE0,        0x07},
//	  {CC1200_AGC_REF,             0x28},
//	  {CC1200_AGC_CS_THR,          0xF6},
//	  {CC1200_AGC_GAIN_ADJUST,     0x00},
//	  {CC1200_AGC_CFG3,            0xB1},
//	  {CC1200_AGC_CFG2,            0x20},
//	  {CC1200_AGC_CFG1,            0x12},
//	  {CC1200_AGC_CFG0,            0x84},
//	  {CC1200_FIFO_CFG,            0x00},
//	  {CC1200_DEV_ADDR,            0x00},
//	  {CC1200_SETTLING_CFG,        0x0B},
//	  {CC1200_FS_CFG,              0x14},
//	  {CC1200_WOR_CFG1,            0x08},
//	  {CC1200_WOR_CFG0,            0x21},
//	  {CC1200_WOR_EVENT0_MSB,      0x00},
//	  {CC1200_WOR_EVENT0_LSB,      0x00},
//	  {CC1200_RXDCM_TIME,          0x00},
//	  {CC1200_PKT_CFG2,            0x00},
//	  {CC1200_PKT_CFG1,            0x03},
//	  {CC1200_PKT_CFG0,            0x20},
//	  {CC1200_RFEND_CFG1,          0x0F},
//	  {CC1200_RFEND_CFG0,          0x00},
//	  {CC1200_PA_CFG1,             0x5F},
//	  {CC1200_PA_CFG0,             0x56},
//	  {CC1200_ASK_CFG,             0x0F},
//	  {CC1200_PKT_LEN,             0xFF},
//	};
//
//	/* array mapping each CC1200 extended register with a default value */
//	RegisterSetting_t Preferred_Extended_Register_Settings[]=
//	{
//	  {CC1200_IF_MIX_CFG,          0x1C},
//	  {CC1200_FREQOFF_CFG,         0x20},
//	  {CC1200_TOC_CFG,             0x03},
//	  {CC1200_MARC_SPARE,          0x00},
//	  {CC1200_ECG_CFG,             0x00},
//	  {CC1200_MDMCFG2,             0x02},
//	  {CC1200_EXT_CTRL,            0x01},
//	  {CC1200_RCCAL_FINE,          0x00},
//	  {CC1200_RCCAL_COURSE,        0x00},
//	  {CC1200_RCCAL_OFFSET,        0x00},
//	  {CC1200_FREQOFF1,            0x00},
//	  {CC1200_FREQOFF0,            0x00},
//	  {CC1200_FREQ2,               0x57},
//	  {CC1200_FREQ1,               0x4C},
//	  {CC1200_FREQ0,               0xCC},
//	  {CC1200_IF_ADC2,             0x02},
//	  {CC1200_IF_ADC1,             0xEE},
//	  {CC1200_IF_ADC0,             0x10},
//	  {CC1200_FS_DIG1,             0x07},
//	  {CC1200_FS_DIG0,             0x50},
//	  {CC1200_FS_CAL3,             0x00},
//	  {CC1200_FS_CAL2,             0x20},
//	  {CC1200_FS_CAL1,             0x40},
//	  {CC1200_FS_CAL0,             0x0E},
//	  {CC1200_FS_CHP,              0x28},
//	  {CC1200_FS_DIVTWO,           0x03},
//	  {CC1200_FS_DSM1,             0x00},
//	  {CC1200_FS_DSM0,             0x33},
//	  {CC1200_FS_DVC1,             0xFF},
//	  {CC1200_FS_DVC0,             0x17},
//	  {CC1200_FS_LBI,              0x00},
//	  {CC1200_FS_PFD,              0x00},
//	  {CC1200_FS_PRE,              0x6E},
//	  {CC1200_FS_REG_DIV_CML,      0x1C},
//	  {CC1200_FS_SPARE,            0xAC},
//	  {CC1200_FS_VCO4,             0x14},
//	  {CC1200_FS_VCO3,             0x00},
//	  {CC1200_FS_VCO2,             0x00},
//	  {CC1200_FS_VCO1,             0x00},
//	  {CC1200_FS_VCO0,             0xB5},
//	  {CC1200_GBIAS6,              0x00},
//	  {CC1200_GBIAS5,              0x02},
//	  {CC1200_GBIAS4,              0x00},
//	  {CC1200_GBIAS3,              0x00},
//	  {CC1200_GBIAS2,              0x10},
//	  {CC1200_GBIAS1,              0x00},
//	  {CC1200_GBIAS0,              0x00},
//	  {CC1200_IFAMP,               0x09},
//	  {CC1200_LNA,                 0x01},
//	  {CC1200_RXMIX,               0x01},
//	  {CC1200_XOSC5,               0x0E},
//	  {CC1200_XOSC4,               0xA0},
//	  {CC1200_XOSC3,               0x03},
//	  {CC1200_XOSC2,               0x04},
//	  {CC1200_XOSC1,               0x03},
//	  {CC1200_XOSC0,               0x00},
//	  {CC1200_ANALOG_SPARE,        0x00},
//	  {CC1200_PA_CFG3,             0x00},
//	  {CC1200_WOR_TIME1,           0x00},
//	  {CC1200_WOR_TIME0,           0x00},
//	  {CC1200_WOR_CAPTURE1,        0x00},
//	  {CC1200_WOR_CAPTURE0,        0x00},
//	  {CC1200_BIST,                0x00},
//	  {CC1200_DCFILTOFFSET_I1,     0x00},
//	  {CC1200_DCFILTOFFSET_I0,     0x00},
//	  {CC1200_DCFILTOFFSET_Q1,     0x00},
//	  {CC1200_DCFILTOFFSET_Q0,     0x00},
//	  {CC1200_IQIE_I1,             0x00},
//	  {CC1200_IQIE_I0,             0x00},
//	  {CC1200_IQIE_Q1,             0x00},
//	  {CC1200_IQIE_Q0,             0x00},
//	  {CC1200_RSSI1,               0x80},
//	  {CC1200_RSSI0,               0x00},
//	  {CC1200_MARCSTATE,           0x41},
//	  {CC1200_LQI_VAL,             0x00},
//	  {CC1200_PQT_SYNC_ERR,        0xFF},
//	  {CC1200_DEM_STATUS,          0x00},
//	  {CC1200_FREQOFF_EST1,        0x00},
//	  {CC1200_FREQOFF_EST0,        0x00},
//	  {CC1200_AGC_GAIN3,           0x00},
//	  {CC1200_AGC_GAIN2,           0xD1},
//	  {CC1200_AGC_GAIN1,           0x00},
//	  {CC1200_AGC_GAIN0,           0x3F},
//	  {CC1200_CFM_RX_DATA_OUT,     0x00},
//	  {CC1200_CFM_TX_DATA_IN,      0x00},
//	  {CC1200_ASK_SOFT_RX_DATA,    0x30},
//	  {CC1200_RNDGEN,              0x7F},
//	  {CC1200_MAGN2,               0x00},
//	  {CC1200_MAGN1,               0x00},
//	  {CC1200_MAGN0,               0x00},
//	  {CC1200_ANG1,                0x00},
//	  {CC1200_ANG0,                0x00},
//	  {CC1200_CHFILT_I2,           0x02},
//	  {CC1200_CHFILT_I1,           0x00},
//	  {CC1200_CHFILT_I0,           0x00},
//	  {CC1200_CHFILT_Q2,           0x00},
//	  {CC1200_CHFILT_Q1,           0x00},
//	  {CC1200_CHFILT_Q0,           0x00},
//	  {CC1200_GPIO_STATUS,         0x00},
//	  {CC1200_FSCAL_CTRL,          0x01},
//	  {CC1200_PHASE_ADJUST,        0x00},
//	  {CC1200_PARTNUMBER,          0x00},
//	  {CC1200_PARTVERSION,         0x00},
//	  {CC1200_SERIAL_STATUS,       0x00},
//	  {CC1200_MODEM_STATUS1,       0x01},
//	  {CC1200_MODEM_STATUS0,       0x00},
//	  {CC1200_MARC_STATUS1,        0x00},
//	  {CC1200_MARC_STATUS0,        0x00},
//	  {CC1200_PA_IFAMP_TEST,       0x00},
//	  {CC1200_FSRF_TEST,           0x00},
//	  {CC1200_PRE_TEST,            0x00},
//	  {CC1200_PRE_OVR,             0x00},
//	  {CC1200_ADC_TEST,            0x00},
//	  {CC1200_DVC_TEST,            0x0B},
//	  {CC1200_ATEST,               0x40},
//	  {CC1200_ATEST_LVDS,          0x00},
//	  {CC1200_ATEST_MODE,          0x00},
//	  {CC1200_XOSC_TEST1,          0x3C},
//	  {CC1200_XOSC_TEST0,          0x00},
//	  {CC1200_AES,                 0x00},
//	  {CC1200_MDM_TEST,            0x00},
//	  {CC1200_RXFIRST,             0x00},
//	  {CC1200_TXFIRST,             0x00},
//	  {CC1200_RXLAST,              0x00},
//	  {CC1200_TXLAST,              0x00},
//	  {CC1200_NUM_TXBYTES,         0x00},
//	  {CC1200_NUM_RXBYTES,         0x00},
//	  {CC1200_FIFO_NUM_TXBYTES,    0x0F},
//	  {CC1200_FIFO_NUM_RXBYTES,    0x00},
//	  {CC1200_RXFIFO_PRE_BUF,      0x00},
//	};

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */
	CC1200_Init(&CC1200_SPI_Info, CC1200_Data, GPIOB, GPIO_PIN_6, &hspi1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//printf("Hello World\n");
		//scanf();
//		// CC1200 Transmit / Receive Test
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&hspi1, CC1200_TEST_PACKET, CC1200_STATUS_BYTES, 2, HAL_MAX_DELAY);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

//		// Chip Select Test
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//		HAL_Delay(100);

//		// Hello World Test
//		char Message[100];
//		uint16_t Message_Length;
//		Message_Length = sprintf(Message, "Hello World!\r\n");
//		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
//		CDC_Transmit_FS((uint8_t*) ("Hello World!\r\n"), sizeof("Hello World!\r\n"));
//		HAL_Delay(1000); // delay 1 sec

		// CC1200 Functions Test
		char Message[100];
		uint16_t Message_Length;

		Message_Length = sprintf(Message, "CC1200 Command Strobe Test\r\n");
		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
		HAL_Delay(100); // delay 100 ms

		CC1200_Command_Strobe(&CC1200_SPI_Info, CC1200_COMMAND_SRES); // reset the chip

		CC1200_Command_Strobe(&CC1200_SPI_Info, CC1200_COMMAND_SRX); // reset the chip

		CC1200_Command_Strobe(&CC1200_SPI_Info, CC1200_COMMAND_STX); // reset the chip

		//CC1200_Write_Single_Register();
		//CC1200_Read_Single_Register();

		Message_Length = sprintf(Message, "\r\n");
		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
		HAL_Delay(100); // delay 100 ms

		HAL_Delay(1000); // wait 1 s

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC12 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
