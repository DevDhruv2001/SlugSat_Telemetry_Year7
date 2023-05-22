/*
 * CC1200.h
 *
 *  Created on: Apr 16, 2023
 *      Author: devdhruv
 */

#ifndef INC_CC1200_H_
#define INC_CC1200_H_

/* Includes */
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CC1200_Registers.h"

/* structure for the register addresses and desired values for configuring the CC1200 */
typedef struct CC1200_RegisterSetting{
	uint8_t Address;	// The register address to access
	uint8_t Value;	// The value to be loaded at the register address
} RegisterSetting_t;

/* structure for SPI communication */
typedef struct CC1200
{
	uint8_t* MISO_Data; // store data received from CC1200
	GPIO_TypeDef* CS_Port;
	uint16_t CS_Pin;
	SPI_HandleTypeDef* HSPI; // SPI handler
} CC1200_t;

/**
 * @brief Initializes the CC1200 for SPI communication
 * @param SPI_Info : structure with MISO data, CS Port/Pin, SPI handler
 * @param MISO_Data : buffer for data received from CC1200
 * @param CS_Port: GPIO peripheral
 * @param CS_Pin: GPIO pin
 * @param HSPI: SPI handler
 * @retval Success (0) or Error (1)
 */
// document what register parameters to change
void CC1200_Init(CC1200_t* SPI_Info, uint8_t* MISO_Data, GPIO_TypeDef* CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef* HSPI);

/**
 * @brief Configures the CC1200 with the preferred register settings
 * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
 * @param SPI_Info : structure with register addresses and associated values
 * @retval Success (0) or Error (1)
 */
uint8_t CC1200_Configure(CC1200_t* SPI_Info, RegisterSetting_t* Register_Setting, RegisterSetting_t* Extended_Register_Setting);

/**
  * @brief Write a value to a specified register
  * 	R/W = 0
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @param Register_Value : value to write to register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Write_Single_Register(CC1200_t* SPI_Info, uint8_t Register_Address, uint8_t Register_Value);

/**
  * @brief Read a value from a specified register
  * 	R/W = 1
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Read_Single_Register(CC1200_t* SPI_Info, uint8_t Register_Address);

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
uint8_t CC1200_Write_Single_Extended_Register(CC1200_t* SPI_Info, uint8_t Register_Address, uint8_t Register_Value);

/**
  * @brief Read a value from a specified register in the extended register space
  * 	R/W = 1
  * 	B   = 0
  * 	Extended Address = 0x2F
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Read_Single_Extended_Register(CC1200_t* SPI_Info, uint8_t Register_Address);

/**
  * @brief Write to a Command Strobe Register
  * 	R/W = 1
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Command_Strobe(CC1200_t* SPI_Info, uint8_t Register_Address);

/**
  * @brief Transmit Packets via Standard FIFO Access
  * 	R/W = 1
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Transmit(CC1200_t* SPI_Info, uint8_t* TX_Packet, uint8_t TX_Packet_Length);

/**
  * @brief Receive Packets via Standard FIFO Access
  * 	R/W = 1
  * 	B   = 0
  * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
  * @param Register_Address : address of register
  * @retval Success (0) or Error (1)
  */
uint8_t CC1200_Receive(CC1200_t* SPI_Info, uint8_t* RX_Packet, uint8_t* RX_Packet_Length);

#endif /* INC_CC1200_H_ */
