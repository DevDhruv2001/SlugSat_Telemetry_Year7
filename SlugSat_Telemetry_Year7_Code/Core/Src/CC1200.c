/*
 * CC1200.c
 *
 *  Created on: Apr 16, 2023
 *      Author: devdhruv
 */

/* Includes */
#include "CC1200.h"

/**
  * @brief Write a value to a specified register
  * 	R/W = 0
  * 	B   = 0
  * @param Register_Address : address of register
  * @param Register_Value : value to write to register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Write_Single_Register(CC1200_t* SPI_Info, uint8_t Register_Address, uint8_t Register_Value)
{
	uint8_t retval;
	char Message[100];
	uint16_t Message_Length;

	if (Register_Address < 0x2F)
	{
		uint8_t Header_Byte = 0x00 | Register_Address; // 0000 0000 | 0 0 A5 A4 A3 A2 A1 A0
		uint8_t MOSI_Data[2] = {Header_Byte, Register_Value};
		(SPI_Info -> MOSI_Data)[0] = MOSI_Data[0];
		(SPI_Info -> MOSI_Data)[1] = MOSI_Data[1];
		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);
		HAL_Delay(10); // delay 10 ms
		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 2, HAL_MAX_DELAY);
		HAL_Delay(10); // delay 10 ms
		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	else
	{
		Message_Length = sprintf(Message, "Invalid Register Access\r\n");
		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
		HAL_Delay(100); // delay 100 ms
		retval = 1;
	}
	return retval;
}

/**
  * @brief Read a value from a specified register
  * 	R/W = 1
  * 	B   = 0
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Read_Single_Register(CC1200_t* SPI_Info, uint8_t Register_Address)
{
	uint8_t retval;
	char Message[100];
	uint16_t Message_Length;

	if (Register_Address < 0x2F)
	{
		uint8_t Header_Byte = 0x80 | Register_Address; // 1000 0000 | 0 0 A5 A4 A3 A2 A1 A0
		uint8_t MOSI_Data[2] = {Header_Byte, 0x00};
		(SPI_Info -> MOSI_Data)[0] = MOSI_Data[0];
		(SPI_Info -> MOSI_Data)[1] = MOSI_Data[1];
		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);
		HAL_Delay(10); // delay 10 ms
		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 2, HAL_MAX_DELAY);
		HAL_Delay(10); // delay 10 ms
		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	else
	{
		Message_Length = sprintf(Message, "Invalid Register Access\r\n");
		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
		HAL_Delay(100); // delay 100 ms
		retval = 1;
	}
	return retval;
}

/**
 * @brief Print the MOSI / MISO data of an SPI transfer
 * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
 * @retval none
 */
void CC1200_Print(CC1200_t* SPI_Info)
{
	uint16_t MOSI_Data_Length = sizeof(SPI_Info -> MOSI_Data);
	uint16_t MISO_Data_Length = sizeof(SPI_Info -> MOSI_Data);
	char Message[100];
	uint16_t Message_Length;

	if (MOSI_Data_Length != MISO_Data_Length)
	{
		Message_Length = sprintf(Message, "MOSI / MISO Data Buffers Have Unequal Lengths\r\n");
		CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
	}
	else
	{
		int i; // counter
		for (i = 0; i < MOSI_Data_Length; i++)
		{
			Message_Length = sprintf(Message, "Transmitted Data: 0X%02X\r\n", (SPI_Info -> MOSI_Data)[i]);
			CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
			HAL_Delay(100); // delay 100 ms
		}
		for (i = 0; i < MISO_Data_Length; i++)
		{
			Message_Length = sprintf(Message, "Received Data: 0X%02X\r\n", (SPI_Info -> MISO_Data)[0]);
			CDC_Transmit_FS((uint8_t*) Message, (Message_Length + 1));
			HAL_Delay(100); // delay 100 ms
		}
	}
}
