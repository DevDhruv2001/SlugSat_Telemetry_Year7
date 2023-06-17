/*
 * Terminal.c
 *
 *  Created on: Jun 6, 2023
 *      Author: devdhruv
 */

/* Includes */
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CC1200.h"
#include "CC1200_Registers.h"
#include "Terminal.h"

/* Private Variables */
// SPI settings from main.c
extern SPI_HandleTypeDef hspi1;
extern CC1200_t SPI_Info; // structure with MISO data buffer, GPIO CS Port/Pin, and SPI handler
extern uint8_t MISO_Data[1]; // MISO data buffer
extern uint8_t RX_Packet[128]; // RX packet

// register settings from main.c
extern RegisterSetting_t Transmit_Register_Settings[];
extern RegisterSetting_t Transmit_Extended_Register_Settings[];
extern RegisterSetting_t Receive_Register_Settings[];
extern RegisterSetting_t Receive_Extended_Register_Settings[];

char Message[10000];
uint16_t Message_Length;
char Message_Part[150];
char* Token;
uint8_t counter;
uint8_t check;
uint8_t Register_Address;
uint8_t Register_Value;
uint8_t TX_Packet[127];
uint8_t TX_Packet_Length;
int16_t RSSI_Value;

// Active Functions

void Configure(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Configure\r\n");
	Token = strtok((char*) RX_Buffer, " "); // first token "configure"
	Token = strtok(NULL, "\r\n"); // second token "[mode]"
	if (strncmp(Token, "transmit", strlen("transmit")) == 0)
	{
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRES); // reset the chip
		sprintf(Message_Part, "Selected Mode: %s\r\n", Token);
		strcat(Message, Message_Part);
		strcat(Message, "Configured the CC1200 with Transmit Settings\r\n");
		check = CC1200_Configure(&SPI_Info, Transmit_Register_Settings, Transmit_Extended_Register_Settings);
		if (check == 1)
		{
			strcat(Message, "Error Occurred\r\n");
		}
		else // check == 0
		{
			strcat(Message, "No Error Occurred\r\n");
		}
	}
	else if (strncmp(Token, "receive", strlen("receive")) == 0)
	{
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRES); // reset the chip
		sprintf(Message_Part, "Selected Mode: %s\r\n", Token);
		strcat(Message, Message_Part);
		strcat(Message, "Configured the CC1200 with Receive Settings\r\n");
		check = CC1200_Configure(&SPI_Info, Receive_Register_Settings, Receive_Extended_Register_Settings);
		if (check == 1)
		{
			strcat(Message, "Error Occurred\r\n");
		}
		else // check == 0
		{
			strcat(Message, "No Error Occurred\r\n");
		}
	}
	else
	{
		sprintf(Message_Part, "Invalid Mode: %s\r\n", Token);
		strcat(Message, Message_Part);
		strcat(Message, "Could Not Configure the CC1200\r\n");
	}
}

void Transmit(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Transmit\r\n");
	strcat(Message, "Set the CC1200 into Transmit Mode\r\n");
	strcat(Message, "Transmitted the Following Message: ");
	Token = strtok((char*) RX_Buffer, " "); // first token "transmit"
	Token = strtok(NULL, "\r\n"); // second token "[message to send]"
	TX_Packet_Length = strlen(Token);
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	for (counter = 0; counter < TX_Packet_Length; counter++)
	{
		TX_Packet[counter] = (uint8_t) (Token[counter]);
	}
	CC1200_Transmit(&SPI_Info, TX_Packet, TX_Packet_Length);
}

void Receive(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Receive\r\n");
	strcat(Message, "Set the CC1200 into Receive Mode\r\n");
	CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRX); // enable RX
}

// used with the GPIO interrupt
void Read_RX_FIFO(void)
{

	RSSI_Value = CC1200_Read_RSSI(&SPI_Info);
	check = CC1200_Read_RX_FIFO(&SPI_Info, RX_Packet);

	if (check) // check == 1
	{
		sprintf(Message, "RX FIFO Empty!\r\n");
	}
	else // check == 0
	{
		sprintf(Message, "Received the Following Message: ");
		sprintf(Message_Part, "%s\r\n", (char*) RX_Packet);
		strcat(Message, Message_Part);
		sprintf(Message_Part, "RSSI (dBm): %d\r\n", RSSI_Value);
		strcat(Message, Message_Part);
		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRX); // re-enter RX
	}
}

void Status(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Status\r\n");
	strcat(Message, "CC1200 Status: ");
	CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP); // get status
	sprintf(Message_Part, "0X%02X\r\n", MISO_Data[0]);
	strcat(Message, Message_Part);
}

void Register_Access(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Register\r\n");
	Token = strtok((char*) RX_Buffer, " "); // first token "register"
	Token = strtok(NULL, " "); // second token "[access type]"
	if (strncmp(Token, "print", strlen("print")) == 0)
	{
		sprintf(Message_Part, "Selected Access Type: %s\r\n", Token);
		strcat(Message, Message_Part);
		strcat(Message, "Register Space\r\n");
		strcat(Message, "Address; Value\r\n");
		for (Register_Address = 0x00; Register_Address < 0x2F; Register_Address++)
		{

			CC1200_Read_Single_Register(&SPI_Info, Register_Address);
			Register_Value = MISO_Data[0];
			sprintf(Message_Part, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
			strcat(Message, Message_Part);
		}
		strcat(Message, "Extended Register Space\r\n");
		strcat(Message, "Address; Value\r\n");
		for (Register_Address = 0x00; Register_Address <= 0xDA; Register_Address++)
		{
			check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
			if (check == 0)
			{
				Register_Value = MISO_Data[0];
				sprintf(Message_Part, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
				strcat(Message, Message_Part);
			}
			else
			{
				continue;
			}
		}
	}
	else if (strncmp(Token, "read", strlen("read")) == 0)
	{
		Token = strtok(NULL, " "); // third token "[register space]"
		if (strncmp(Token, "extended", strlen("extended")) == 0)
		{
			strcat(Message, "Address to Read: ");
			Token = strtok(NULL, "\r\n"); // fourth token "[register address]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Address = strtol(Token, NULL, 16); // convert register address to a number
			check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
			strcat(Message, "Value Received: ");
			Register_Value = MISO_Data[0];
			sprintf(Message_Part, "0X%02X\r\n", Register_Value);
			strcat(Message, Message_Part);
		}
		else if (strncmp(Token, "regular", strlen("regular")) == 0)
		{
			strcat(Message, "Address to Read: ");
			Token = strtok(NULL, "\r\n"); // third token "[register address]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Address = strtol(Token, NULL, 16); // convert register address to a number
			check = CC1200_Read_Single_Register(&SPI_Info, Register_Address);
			strcat(Message, "Value Received: ");
			Register_Value = MISO_Data[0];
			sprintf(Message_Part, "0X%02X\r\n", Register_Value);
			strcat(Message, Message_Part);
		}
		else
		{
			strcat(Message, "Invalid Access\r\n");
		}
	}
	else if (strncmp(Token, "write", strlen("write")) == 0)
	{
		Token = strtok(NULL, " "); // third token "[register space]"
		if (strncmp(Token, "extended", strlen("extended")) == 0)
		{
			strcat(Message, "Address to Write: ");
			Token = strtok(NULL, " "); // fourth token "[register address]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Address = strtol(Token, NULL, 16); // convert register address to a number
			strcat(Message, "Value Sent: ");
			Token = strtok(NULL, "\r\n"); // fifth token "[register value]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Value = strtol(Token, NULL, 16); // convert register value to a number
			check = CC1200_Write_Single_Extended_Register(&SPI_Info, Register_Address, Register_Value);
		}
		else if (strncmp(Token, "regular", strlen("regular")) == 0)
		{
			strcat(Message, "Address to Write: ");
			Token = strtok(NULL, " "); // fourth token "[register address]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Address = strtol(Token, NULL, 16); // convert register address to a number
			strcat(Message, "Value Sent: ");
			Token = strtok(NULL, "\r\n"); // fifth token "[register value]"
			sprintf(Message_Part, "%s\r\n", Token);
			strcat(Message, Message_Part);
			Register_Value = strtol(Token, NULL, 16); // convert register value to a number
			check = CC1200_Write_Single_Register(&SPI_Info, Register_Address, Register_Value);
		}
		else
		{
			strcat(Message, "Invalid Access\r\n");
		}
	}
	else
	{
		strcat(Message, "Invalid Access\r\n");
	}
}

void Print_Message(void)
{
	Message_Length = strlen(Message);
	CDC_Transmit_FS((uint8_t*) Message, Message_Length);
}

// Inactive Functions

void Start(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Start\r\n");
	strcat(Message, "Initialized the CC1200 for Operation\r\n");
	CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
}

void Reset(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Reset\r\n");
	strcat(Message, "Set the CC1200 into IDLE Mode\r\n");
	strcat(Message, "Register Values Reset to Default Values\r\n");
	CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRES);
}

void Command(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Command\r\n");
	strcat(Message, "Issued the Following Command: ");
	Token = strtok((char*) RX_Buffer, " "); // first token "Command:"
	Token = strtok(NULL, "\r\n"); // second token "[command strobe]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Address = strtol(Token, NULL, 16); // convert register address to a number
	check = CC1200_Command_Strobe(&SPI_Info, Register_Address); // issue command
	if (check == 1)
	{
		strcat(Message, "Invalid Command\r\n");
	}
	else // check == 0
	{
		strcat(Message, "Valid Command\r\n");
	}
	CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP); // get status
	sprintf(Message_Part, "CC1200 State: 0X%02X\r\n", MISO_Data[0]);
	strcat(Message, Message_Part);
}

void Print_Registers(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Print Registers\r\n");
	strcat(Message, "Register Space\r\n");
	strcat(Message, "Address; Value\r\n");
	for (Register_Address = 0x00; Register_Address < 0x2F; Register_Address++)
	{
		CC1200_Read_Single_Register(&SPI_Info, Register_Address);
		Register_Value = MISO_Data[0];
		sprintf(Message_Part, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
		strcat(Message, Message_Part);
	}
	strcat(Message, "Extended Register Space\r\n");
	strcat(Message, "Address; Value\r\n");
	for (Register_Address = 0x00; Register_Address <= 0xDA; Register_Address++)
	{
		check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
		if (check == 0)
		{
			Register_Value = MISO_Data[0];
			sprintf(Message_Part, "0X%02X; 0X%02X\r\n", Register_Address, Register_Value);
			strcat(Message, Message_Part);
		}
		else
		{
			continue;
		}
	}
}

void Write_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Write Register\r\n");
	Token = strtok((char*) RX_Buffer, " "); // first token "Write"
	Token = strtok(NULL, " "); // second token "Register:"
	strcat(Message, "Address to Access: ");
	Token = strtok(NULL, " "); // third token "[register address]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Address = strtol(Token, NULL, 16); // convert register address to a number
	strcat(Message, "Value Sent: ");
	Token = strtok(NULL, "\r\n"); // fourth token "[register value]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Value = strtol(Token, NULL, 16); // convert register value to a number
	check = CC1200_Write_Single_Register(&SPI_Info, Register_Address, Register_Value);
}

void Read_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Read Register\r\n");
	strcat(Message, "Address to Access: ");
	Token = strtok((char*) RX_Buffer, " "); // first token "Read"
	Token = strtok(NULL, " "); // second token "Register:"
	Token = strtok(NULL, "\r\n"); // third token "[register address]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Address = strtol(Token, NULL, 16); // convert register address to a number
	check = CC1200_Read_Single_Register(&SPI_Info, Register_Address);
	strcat(Message, "Value Received: ");
	Register_Value = MISO_Data[0];
	sprintf(Message_Part, "0X%02X\r\n", Register_Value);
	strcat(Message, Message_Part);
}

void Write_Extended_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Write Extended Register\r\n");
	strcat(Message, "Address to Access: ");
	Token = strtok((char*) RX_Buffer, " "); // first token "Write"
	Token = strtok(NULL, " "); // second token "Extended"
	Token = strtok(NULL, " "); // third token "Register:"
	Token = strtok(NULL, " "); // fourth token "[register address]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Address = strtol(Token, NULL, 16); // convert register address to a number
	strcat(Message, "Value Sent: ");
	Token = strtok(NULL, "\r\n"); // fifth token "[register value]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Value = strtol(Token, NULL, 16); // convert register value to a number
	check = CC1200_Write_Single_Extended_Register(&SPI_Info, Register_Address, Register_Value);
}

void Read_Extended_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len)
{
	sprintf(Message, "User Input: Read Extended Register\r\n");
	strcat(Message, "Address to Access: ");
	Token = strtok((char*) RX_Buffer, " "); // first token "Read"
	Token = strtok(NULL, " "); // second token "Extended"
	Token = strtok(NULL, " "); // third token "Register:"
	Token = strtok(NULL, "\r\n"); // fourth token "[register address]"
	sprintf(Message_Part, "%s\r\n", Token);
	strcat(Message, Message_Part);
	Register_Address = strtol(Token, NULL, 16); // convert register address to a number
	check = CC1200_Read_Single_Extended_Register(&SPI_Info, Register_Address);
	strcat(Message, "Value Received: ");
	Register_Value = MISO_Data[0];
	sprintf(Message_Part, "0X%02X\r\n", Register_Value);
	strcat(Message, Message_Part);
}
