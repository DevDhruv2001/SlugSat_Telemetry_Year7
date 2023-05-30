/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "CC1200.h"
#include "CC1200_Registers.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_RX_BUFFER_SIZE 1000 // Maximum size of the receive buffer
static uint8_t rx_buffer[MAX_RX_BUFFER_SIZE]; // Receive buffer
static uint32_t rx_buffer_len = 0; // Length of the data in the receive buffer

extern SPI_HandleTypeDef hspi1;
extern CC1200_t SPI_Info; // structure with MISO data buffer, GPIO CS Port/Pin, and SPI handler
extern uint8_t MISO_Data[1]; // MISO data buffer
extern uint8_t RX_Packet[128]; // RX packet

extern RegisterSetting_t Transmit_Register_Settings[];
extern RegisterSetting_t Transmit_Extended_Register_Settings[];
extern RegisterSetting_t Receive_Register_Settings[];
extern RegisterSetting_t Receive_Extended_Register_Settings[];

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
uint8_t Process_Received_Message(uint8_t* rx_buffer, uint32_t rx_buffer_len);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	// Copy the received data to the buffer
	for (int i = 0; i < *Len; i++)
	{
	if (rx_buffer_len < MAX_RX_BUFFER_SIZE)
	{
	  rx_buffer[rx_buffer_len] = Buf[i];
	  rx_buffer_len = rx_buffer_len + 1;
	}
	else
	{
	  // The receive buffer is full, discard the received data
	}
	}

	// Check if a complete message has been received
	if (rx_buffer_len > 0 && rx_buffer[rx_buffer_len - 1] == '\n')
	{
		// Process the received message
		Process_Received_Message(rx_buffer, rx_buffer_len);
		// Clear the receive buffer
		rx_buffer_len = 0;
	}

	// Set up the USB device to receive a new packet
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t Process_Received_Message(uint8_t* rx_buffer, uint32_t rx_buffer_len)
{
	char Message[10000];
	uint16_t Message_Length;
	char str1[150];
	char str2[150];
	char str3[150];
	char str4[150];
	char str5[150];
	char* Token;

	uint8_t i;
	uint8_t check;

	uint8_t Register_Address;
	uint8_t Register_Value;

	uint8_t TX_Packet[127];
	uint8_t TX_Packet_Length;

	// receive variables (unused)
	//uint8_t RX_Packet[128]; // add null character
	//uint8_t RX_Packet_Length; // max 127
	//char RX_String[128]; // convert uint8_t to char


	if(strncmp((char*) rx_buffer, "start", strlen("start")) == 0)
	{
		sprintf(str1, "User Input: Start\r\n");
		sprintf(str2, "Initialized the CC1200 for Operation\r\n");
		CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
		Message_Length = sprintf(Message, "%s%s", str1, str2);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "configure", strlen("configure")) == 0)
	{
		sprintf(str1, "User Input: Configure\r\n");
		Token = strtok((char*) rx_buffer, " "); // first token "Configure:"
		Token = strtok(NULL, "\r\n"); // second token "[mode]"
		if (strncmp(Token, "transmit", strlen("transmit")) == 0)
		{
			sprintf(str2, "Selected Mode: %s\r\n", Token);
			sprintf(str3, "Configured the CC1200 with Transmit Settings\r\n");
			check = CC1200_Configure(&SPI_Info, Transmit_Register_Settings, Transmit_Extended_Register_Settings);
			if (check == 1)
			{
				sprintf(str4, "Error Occurred\r\n");
			}
			else // check == 0
			{
				sprintf(str4, "No Error Occurred\r\n");
			}
			Message_Length = sprintf(Message, "%s%s%s%s", str1, str2, str3, str4); // include str4
		}
		else if (strncmp(Token, "receive", strlen("receive")) == 0)
		{
			sprintf(str2, "Selected Mode: %s\r\n", Token);
			sprintf(str3, "Configured the CC1200 with Receive Settings\r\n");
			check = CC1200_Configure(&SPI_Info, Receive_Register_Settings, Receive_Extended_Register_Settings);
			if (check == 1)
			{
				sprintf(str4, "Error Occurred\r\n");
			}
			else // check == 0
			{
				sprintf(str4, "No Error Occurred\r\n");
			}
			Message_Length = sprintf(Message, "%s%s%s%s", str1, str2, str3, str4); // include str4
		}
		else
		{
			sprintf(str2, "Invalid Mode: %s\r\n", Token);
			sprintf(str3, "Could Not Configure the CC1200\r\n");
			Message_Length = sprintf(Message, "%s%s%s", str1, str2, str3);
		}

		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "transmit", strlen("transmit")) == 0)
	{
		sprintf(str1, "User Input: Transmit\r\n");
		sprintf(str2, "Set the CC1200 into Transmit Mode\r\n");
		sprintf(str3, "Transmitted the Following Message: ");
		Token = strtok((char*) rx_buffer, " "); // first token "Transmit:"
		Token = strtok(NULL, "\r\n"); // second token "[message to send]"
		TX_Packet_Length = strlen(Token);
		sprintf(str4, "%s\r\n", Token);
		for (i = 0; i < TX_Packet_Length; i++)
		{
			TX_Packet[i] = (uint8_t) (Token[i]);
		}
		CC1200_Transmit(&SPI_Info, TX_Packet, TX_Packet_Length);
		Message_Length = sprintf(Message, "%s%s%s%s", str1, str2, str3, str4);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "receive", strlen("receive")) == 0)
	{
		sprintf(str1, "User Input: Receive\r\n");
		sprintf(str2, "Set the CC1200 into Receive Mode\r\n");
		//CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SFRX); // flush RX FIFO (before initiating receive)
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRX); // enable RX
		//CC1200_Receive(&SPI_Info);
		Message_Length = sprintf(Message, "%s%s", str1, str2);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "get received data", strlen("get received data")) == 0)
	{
		CC1200_Receive(&SPI_Info, RX_Packet);

		sprintf(Message, "Received the Following Message: ");
		sprintf(str1, "%s\r\n", (char*) RX_Packet);
		strcat(Message, str1);
		Message_Length = strlen(Message);

		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "exit", strlen("exit")) == 0)
	{
		sprintf(str1, "User Input: Exit\r\n");
		sprintf(str2, "Set the CC1200 into IDLE Mode\r\n");
		sprintf(str3, "Register Values Can Be Changed\r\n");
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SIDLE);
		Message_Length = sprintf(Message, "%s%s%s", str1, str2, str3);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "reset", strlen("reset")) == 0)
	{
		sprintf(str1, "User Input: Reset\r\n");
		sprintf(str2, "Set the CC1200 into IDLE Mode\r\n");
		sprintf(str3, "Register Values Reset to Default Values\r\n");
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRES);
		Message_Length = sprintf(Message, "%s%s%s", str1, str2, str3);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "status", strlen("status")) == 0)
	{
		sprintf(str1, "User Input: Status\r\n");
		sprintf(str2, "CC1200 Status: ");
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP); // get status
		sprintf(str3, "0X%02X\r\n", MISO_Data[0]);
		Message_Length = sprintf(Message, "%s%s%s", str1, str2, str3);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "command", strlen("command")) == 0)
	{
		sprintf(str1, "User Input: Command\r\n");
		sprintf(str2, "Issued the Following Command: ");
		Token = strtok((char*) rx_buffer, " "); // first token "Command:"
		Token = strtok(NULL, "\r\n"); // second token "[command strobe]"
		sprintf(str3, "%s\r\n", Token);
		Register_Address = strtol(Token, NULL, 16); // convert register address to a number
		check = CC1200_Command_Strobe(&SPI_Info, Register_Address); // issue command
		if (check == 1)
		{
			sprintf(str4, "Invalid Command\r\n");
		}
		else // check == 0
		{
			sprintf(str4, "Valid Command\r\n");
		}
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP); // get status
		sprintf(str5, "CC1200 State: 0X%02X\r\n", MISO_Data[0]);
		Message_Length = sprintf(Message, "%s%s%s%s%s", str1, str2, str3, str4, str5);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "print registers", strlen("print registers")) == 0)
	{
		sprintf(Message, "User Input: Print Registers\r\n");
		strcat(Message, "Register Space\r\n");
		strcat(Message, "Address; Value\r\n");
		for (Register_Address = 0x00; Register_Address < 0x2F; Register_Address++)
		{

			CC1200_Read_Single_Register(&SPI_Info, Register_Address);
			Register_Value = MISO_Data[0];
			sprintf(str1, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
			strcat(Message, str1);
		}
		strcat(Message, "Extended Register Space\r\n");
		strcat(Message, "Address; Value\r\n");
		for (Register_Address = 0x00; Register_Address <= 0xDA; Register_Address++)
		{
			check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
			if (check == 0)
			{
				Register_Value = MISO_Data[0];
				sprintf(str1, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
				strcat(Message, str1);
			}
			else
			{
				continue;
			}
		}
		Message_Length = strlen(Message);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "write register", strlen("write register")) == 0)
	{
		sprintf(str1, "User Input: Write Register\r\n");
		Token = strtok((char*) rx_buffer, " "); // first token "Write"
		Token = strtok(NULL, " "); // second token "Register:"
		sprintf(str2, "Address to Access: ");
		Token = strtok(NULL, " "); // third token "[register address]"
		sprintf(str3, "%s\r\n", Token);
		Register_Address = strtol(Token, NULL, 16); // convert register address to a number
		sprintf(str4, "Value Sent: ");
		Token = strtok(NULL, "\r\n"); // fourth token "[register value]"
		sprintf(str5, "%s\r\n", Token);
		Register_Value = strtol(Token, NULL, 16); // convert register value to a number
		check = CC1200_Write_Single_Register(&SPI_Info, Register_Address, Register_Value);
		Message_Length = sprintf(Message, "%s%s%s%s%s", str1, str2, str3, str4, str5);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "read register", strlen("read register")) == 0)
	{
		sprintf(str1, "User Input: Read Register\r\n");
		sprintf(str2, "Address to Access: ");
		Token = strtok((char*) rx_buffer, " "); // first token "Read"
		Token = strtok(NULL, " "); // second token "Register:"
		Token = strtok(NULL, "\r\n"); // third token "[register address]"
		sprintf(str3, "%s\r\n", Token);
		Register_Address = strtol(Token, NULL, 16); // convert register address to a number
		check = CC1200_Read_Single_Register(&SPI_Info, Register_Address);
		sprintf(str4, "Value Received: ");
		Register_Value = MISO_Data[0];
		sprintf(str5, "0X%02X\r\n", Register_Value);
		Message_Length = sprintf(Message, "%s%s%s%s%s", str1, str2, str3, str4, str5);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "write extended register", strlen("write extended register")) == 0)
	{
		sprintf(str1, "User Input: Write Extended Register\r\n");
		sprintf(str2, "Address to Access: ");
		Token = strtok((char*) rx_buffer, " "); // first token "Write"
		Token = strtok(NULL, " "); // second token "Extended"
		Token = strtok(NULL, " "); // third token "Register:"
		Token = strtok(NULL, " "); // fourth token "[register address]"
		sprintf(str3, "%s\r\n", Token);
		Register_Address = strtol(Token, NULL, 16); // convert register address to a number
		sprintf(str4, "Value Sent: ");
		Token = strtok(NULL, "\r\n"); // fifth token "[register value]"
		sprintf(str5, "%s\r\n", Token);
		Register_Value = strtol(Token, NULL, 16); // convert register value to a number
		check = CC1200_Write_Single_Extended_Register(&SPI_Info, Register_Address, Register_Value);
		Message_Length = sprintf(Message, "%s%s%s%s%s", str1, str2, str3, str4, str5);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else if (strncmp((char*) rx_buffer, "read extended register", strlen("read extended register")) == 0)
	{
		sprintf(str1, "User Input: Read Extended Register\r\n");
		sprintf(str2, "Address to Access: ");
		Token = strtok((char*) rx_buffer, " "); // first token "Read"
		Token = strtok(NULL, " "); // second token "Extended"
		Token = strtok(NULL, " "); // third token "Register:"
		Token = strtok(NULL, "\r\n"); // fourth token "[register address]"
		sprintf(str3, "%s\r\n", Token);
		Register_Address = strtol(Token, NULL, 16); // convert register address to a number
		check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
		sprintf(str4, "Value Received: ");
		Register_Value = MISO_Data[0];
		sprintf(str5, "0X%02X\r\n", Register_Value);
		Message_Length = sprintf(Message, "%s%s%s%s%s", str1, str2, str3, str4, str5);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else
	{
		CDC_Transmit_FS(rx_buffer, rx_buffer_len);
	}

	return 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
