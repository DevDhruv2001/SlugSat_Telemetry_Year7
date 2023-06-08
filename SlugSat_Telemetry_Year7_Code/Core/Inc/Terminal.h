/*
 * Terminal.h
 *
 *  Created on: Jun 6, 2023
 *      Author: devdhruv
 */

#ifndef INC_TERMINAL_H_
#define INC_TERMINAL_H_

// Active Functions
void Configure(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Transmit(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Receive(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Status(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Register_Access(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Print_Message(void);

// Inactive Functions
void Start(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Read_RX_FIFO(void);

void Reset(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Command(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Print_Registers(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Write_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Read_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Write_Extended_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

void Read_Extended_Register(uint8_t* RX_Buffer, uint32_t RX_Buffer_Len);

#endif /* INC_TERMINAL_H_ */
