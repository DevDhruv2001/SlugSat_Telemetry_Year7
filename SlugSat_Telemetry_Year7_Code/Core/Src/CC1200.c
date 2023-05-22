/*
 * CC1200.c
 *
 *  Created on: Apr 16, 2023
 *      Author: devdhruv
 */

/* Includes */
#include "CC1200.h"
#include "CC1200_Registers.h"

/**
 * @brief Initializes the CC1200 for SPI communication
 * @param SPI_Info : structure with MISO data, CS Port/Pin, SPI handler
 * @param MISO_Data : buffer for data received from CC1200
 * @param CS_Port: GPIO peripheral
 * @param CS_Pin: GPIO pin
 * @param HSPI: SPI handler
 * @retval none
 */
void CC1200_Init(CC1200_t* SPI_Info, uint8_t* MISO_Data, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef* HSPI)
{
	SPI_Info -> MISO_Data = MISO_Data;
	SPI_Info -> CS_Port = CS_Port;
	SPI_Info -> CS_Pin = CS_Pin;
	SPI_Info -> HSPI = HSPI;

	HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Configures the CC1200 with the preferred register settings
 * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
 * @param setting : structure with register addresses and associated values
 * @retval Success (0) or Error (1)
 */
uint8_t CC1200_Configure(CC1200_t* SPI_Info, RegisterSetting_t* Register_Setting, RegisterSetting_t* Extended_Register_Setting)
{
	uint8_t retval = 0;

	CC1200_Command_Strobe(SPI_Info, CC1200_COMMAND_SRES); // reset the chip

	uint8_t Address;
	uint8_t ConfigIndex = 0;
	// configure standard registers
	for (Address = 0x00; Address < 0x2F; Address++)
	{
		// If at the next desired address to configure, then configure it
		if (Address == Register_Setting[ConfigIndex].Address)
		{
			CC1200_Write_Single_Register(SPI_Info, Address, Register_Setting[ConfigIndex].Value);
			CC1200_Read_Single_Register(SPI_Info, Address);
			if (*(SPI_Info->MISO_Data) != Register_Setting[ConfigIndex].Value)
			{
				retval = 1;
			}
			ConfigIndex++;
		}
	}

	ConfigIndex = 0;
	// configure extended registers
	for (Address = 0x00; Address <= 0xDA; Address++)
	{
		// If at the next desired address to configure, then configure it
		if (Address == Extended_Register_Setting[ConfigIndex].Address)
		{
			CC1200_Write_Single_Extended_Register(SPI_Info, Address, Extended_Register_Setting[ConfigIndex].Value);
			CC1200_Read_Single_Extended_Register(SPI_Info, Address);
			if (*(SPI_Info->MISO_Data) != Extended_Register_Setting[ConfigIndex].Value)
			{
				retval = 1;
			}
			ConfigIndex++;
		}
	}

	return(retval);
}

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
//	char Message[100];
//	uint16_t Message_Length;

	if (Register_Address < 0x2F)
	{
		uint8_t Header_Byte = 0x00 | Register_Address; // 0000 0000 | 0 0 A5 A4 A3 A2 A1 A0
		//uint8_t MOSI_Data[2] = {Header_Byte, Register_Value};

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

		//HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 2, HAL_MAX_DELAY);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Register_Value, SPI_Info -> MISO_Data, 1, 100);

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	else
	{
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
//	char Message[100];
//	uint16_t Message_Length;

	if (Register_Address < 0x2F)
	{
		uint8_t Header_Byte = 0x80 | Register_Address; // 1000 0000 | 0 0 A5 A4 A3 A2 A1 A0
		uint8_t Placeholder = 0x00;
		//uint8_t MOSI_Data[2] = {Header_Byte, Placeholder};

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

		//HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 2, HAL_MAX_DELAY);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Placeholder, SPI_Info -> MISO_Data, 1, 100);

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	else
	{
		retval = 1;
	}
	return retval;
}

/**
  * @brief write a value to a specified register in the extended register space
  * 	R/W = 0
  * 	B   = 1
  * 	Extended Address = 0x2F
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @param Register_Value : value to write to register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Write_Single_Extended_Register(CC1200_t* SPI_Info, uint8_t Register_Address, uint8_t Register_Value)
{
	uint8_t retval;
//	char Message[100];
//	uint16_t Message_Length;

	if ((Register_Address >= 0x3A && Register_Address <= 0x63) || (Register_Address >= 0xA3 && Register_Address <= 0xD1) ||
			(Register_Address >= 0xDB))
	{
		retval = 1;
	}
	else
	{
		uint8_t Header_Byte = 0x00 | 0x2F; // 0000 0000 | 0 0 1 0 1 1 1 1
		//uint8_t MOSI_Data[3] = {Header_Byte, Register_Address, Register_Value};

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

		//HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 3, HAL_MAX_DELAY);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Register_Address, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Register_Value, SPI_Info -> MISO_Data, 1, 100);

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	return retval;
}

/**
  * @brief Read a value from a specified register in the extended register space
  * 	R/W = 1
  * 	B   = 0
  * 	Extended Address = 0x2F
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Read_Single_Extended_Register(CC1200_t* SPI_Info, uint8_t Register_Address)
{
	uint8_t retval;
//	char Message[100];
//	uint16_t Message_Length;

	if ((Register_Address >= 0x3A && Register_Address <= 0x63) || (Register_Address >= 0xA3 && Register_Address <= 0xD1) ||
				(Register_Address >= 0xDB))
	{
		retval = 1;
	}
	else
	{
		uint8_t Header_Byte = 0x80 | 0x2F; // 1000 0000 | 0 0 1 0 1 1 1 1
		uint8_t Placeholder = 0x00;
		//uint8_t MOSI_Data[3] = {Header_Byte, Register_Address, Placeholder};
		//(SPI_Info -> MOSI_Data)[0] = MOSI_Data[0];
		//(SPI_Info -> MOSI_Data)[1] = MOSI_Data[1];
		//(SPI_Info -> MOSI_Data)[2] = MOSI_Data[2];

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

		//HAL_SPI_TransmitReceive(SPI_Info -> HSPI, MOSI_Data, SPI_Info -> MISO_Data, 3, HAL_MAX_DELAY);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Register_Address, SPI_Info -> MISO_Data, 1, 100);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Placeholder, SPI_Info -> MISO_Data, 1, 100);

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	return retval;
}

/**
  * @brief Access a Command Strobe Register
  * 	R/W = 0
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Command_Strobe(CC1200_t* SPI_Info, uint8_t Register_Address)
{
	uint8_t retval;
	//char Message[100];
	//uint16_t Message_Length;

	if ((Register_Address >= 0x30) && (Register_Address <= 0x3D))
	{
		uint8_t Header_Byte = 0x00 | Register_Address; // 0000 0000 | 0 0 A5 A4 A3 A2 A1 A0
		//SPI_Info -> MOSI_Data = &Header_Byte;

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

		HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

		retval = 0;
	}
	else
	{
		retval = 1;
	}
	return retval;
}

/**
  * @brief Transmit Packets via Standard FIFO Access
  * 	R/W = 0
  * 	B   = 1
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Transmit(CC1200_t* SPI_Info, uint8_t* TX_Packet, uint8_t TX_Packet_Length)
{
	uint8_t Header_Byte = 0x40 | 0x3F; // 0100 0000 | 0011 1111
	uint8_t i; // counter
	//uint8_t Status; // status byte

	CC1200_Command_Strobe(SPI_Info, CC1200_COMMAND_SFTX); // flush TX FIFO (before loading data)

	HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

	HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &TX_Packet_Length, SPI_Info -> MISO_Data, 1, 100);

	for(i = 0; i < TX_Packet_Length; i++)
	{
		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &(TX_Packet[i]), SPI_Info -> MISO_Data, 1, 100);
	}

	HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

	CC1200_Command_Strobe(SPI_Info, CC1200_COMMAND_STX); // enable TX

	return 0;
}

/**
  * @brief Receive Packets via Standard FIFO Access
  * 	R/W = 1
  * 	B   = 1
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Receive(CC1200_t* SPI_Info, uint8_t* RX_Packet, uint8_t* RX_Packet_Length)
{
	uint8_t Header_Byte = 0xC0 | 0x3F; // 1100 0000 | 0011 1111
	uint8_t Placeholder = 0x00;
	uint8_t Packet_Length;
	uint8_t i; // counter

	CC1200_Command_Strobe(SPI_Info, CC1200_COMMAND_SFRX); // flush RX FIFO (before initiating receive)

	CC1200_Command_Strobe(SPI_Info, CC1200_COMMAND_SRX); // enable RX

	HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Header_Byte, SPI_Info -> MISO_Data, 1, 100);

	HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Placeholder, SPI_Info -> MISO_Data, 1, 100);
	Packet_Length = (SPI_Info->MISO_Data) [0];
	*RX_Packet_Length = Packet_Length;

	for(i = 0; i < Packet_Length; i++)
	{
		HAL_SPI_TransmitReceive(SPI_Info -> HSPI, &Placeholder, SPI_Info -> MISO_Data, 1, 100);
		RX_Packet[i] = (SPI_Info->MISO_Data) [0];
	}

	RX_Packet[Packet_Length] = '\0'; // null termination

	HAL_GPIO_WritePin(SPI_Info -> CS_Port, SPI_Info -> CS_Pin, GPIO_PIN_SET);

	return 0;
}
