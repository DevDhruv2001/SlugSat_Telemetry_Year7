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

/* structure for SPI communication */
typedef struct CC1200
{
	uint8_t* MOSI_Data; // store data transmitted to CC1200
	uint8_t* MISO_Data; // store data received from CC1200
	GPIO_TypeDef* CS_Port;
	uint16_t CS_Pin;
	SPI_HandleTypeDef* HSPI;	///< SPI handler
} CC1200_t;

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
 * @brief Print the MOSI / MISO data of an SPI transfer
 * @param SPI_Info : structure with MOSI/MISO data, CS Port/Pin, SPI handler
 * @retval none
 */
void CC1200_Print(CC1200_t* SPI_Info);

#endif /* INC_CC1200_H_ */
