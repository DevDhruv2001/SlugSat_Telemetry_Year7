/*
 * CC1200_Registers.h
 *
 *  Created on: Apr 16, 2023
 *      Author: devdhruv
 */

#ifndef INC_CC1200_REGISTERS_H_
#define INC_CC1200_REGISTERS_H_

/* (Standard) Register Space */

#define CC1200_IOCFG3						0x00 // GPIO3 Pin COnfiguration
#define CC1200_IOCFG2                   	0x01 // GPIO2 Pin COnfiguration
#define CC1200_IOCFG1                   	0x02 // GPIO1 Pin COnfiguration
#define CC1200_IOCFG0                   	0x03 // GPIO0 Pin COnfiguration

#define CC1200_SYNC3                    	0x04 // Sync Word Configuration [31:24]
#define CC1200_SYNC2                    	0x05 // Sync Word Configuration [23:16]
#define CC1200_SYNC1                    	0x06 // Sync Word Configuration [15:8]
#define CC1200_SYNC0                    	0x07 // Sync Word Configuration [7:0]

#define CC1200_SYNC_CFG1                	0x08 // Sync Word Configuration Reg. 1
#define CC1200_SYNC_CFG0                	0x09 // Sync Word Configuration Reg. 0

#define CC1200_DEVIATION_M              	0x0A // Frequency Deviation Configuration

#define CC1200_MODCFG_DEV_E             	0x0B // Modulation Format and Frequency Deviation Configuration

#define CC1200_DCFILT_CFG               	0x0C // Digital DC Removal Configuration

#define CC1200_PREAMBLE_CFG1            	0x0D // Preamble Configuration Reg. 1
#define CC1200_PREAMBLE_CFG0            	0x0E // Preamble Configuration Reg. 0

#define CC1200_IQIC                     	0x0F // Digital Image Channel Compensation Configuration

#define CC1200_CHAN_BW                  	0x10 // Channel Filter Configuration

#define CC1200_MDMCFG1                  	0x11 // General Modem Parameter Configuration Reg. 1
#define CC1200_MDMCFG0                  	0x12 // General Modem Parameter Configuration Reg. 0

#define CC1200_SYMBOL_RATE2             	0x13 // Symbol Rate Configuration Exponent and Mantissa [19:16]
#define CC1200_SYMBOL_RATE1             	0x14 // Symbol Rate Configuration Exponent and Mantissa [15:8]
#define CC1200_SYMBOL_RATE0             	0x15 // Symbol Rate Configuration Exponent and Mantissa [7:0]

#define CC1200_AGC_REF                  	0x16 // AGC Reference Level Configuration

#define CC1200_AGC_CS_THR               	0x17 // Carrier Sense Threshold Configuration

#define CC1200_AGC_GAIN_ADJUST      	    0x18 // RSSI Offset Configuration

#define CC1200_AGC_CFG3                 	0x19 // Automatic Gain Control Configuration Reg. 3
#define CC1200_AGC_CFG2                 	0x1A // Automatic Gain Control Configuration Reg. 2
#define CC1200_AGC_CFG1                 	0x1B // Automatic Gain Control Configuration Reg. 1
#define CC1200_AGC_CFG0                 	0x1C // Automatic Gain Control Configuration Reg. 0

#define CC1200_FIFO_CFG                 	0x1D // FIFO Configuration

#define CC1200_DEV_ADDR                 	0x1E // Device Address Configuration

#define CC1200_SETTLING_CFG             	0x1F // Frequency Synthesizer Calibration and Settling Configuration

#define CC1200_FS_CFG                   	0x20 // Frequency Synthesizer Configuration

#define CC1200_WOR_CFG1                 	0x21 // Enhanced Wake On Radio Configuration Reg. 1
#define CC1200_WOR_CFG0                 	0x22 // Enhanced Wake On Radio Configuration Reg. 0
#define CC1200_WOR_EVENT0_MSB           	0x23 // Event 0 Configuration MSB
#define CC1200_WOR_EVENT0_LSB           	0x24 // Event 0 Configuration LSB

#define CC1200_RXDCM_TIME               	0x25 // RX Duty Cycle Mode Configuration

#define CC1200_PKT_CFG2                 	0x26 // Packet Configuration Reg. 2
#define CC1200_PKT_CFG1                 	0x27 // Packet Configuration Reg. 1
#define CC1200_PKT_CFG0                 	0x28 // Packet Configuration Reg. 0

#define CC1200_RFEND_CFG1               	0x29 // RFEND Configuration Reg. 1
#define CC1200_RFEND_CFG0               	0x2A // RFEND Configuraton Reg. 0

#define CC1200_PA_CFG1                  	0x2B // Power Amplifier Configuration Reg. 1
#define CC1200_PA_CFG0                  	0x2C // Power Amplifier Configuration Reg. 0

#define CC1200_ASK_CFG                  	0x2D // Amplitude Shift Keying Configuration

#define CC1200_PKT_LEN                  	0x2E // Packet Length Configuration

#define CC1200_EXT_ADDR                 	0x2F // Extended Address

/* Extended Register Space */

#define CC1200_IF_MIX_CFG					0x00 // IF Mix Configuratoin

#define CC1200_FREQOFF_CFG					0x01 // Frequency Offset Correction Configuration

#define CC1200_TOC_CFG						0x02 // Timing Offset Correction Configuration

#define CC1200_MARC_SPARE					0x03 // MARC Spare

#define CC1200_ECG_CFG						0x04 // External Clock Frequency Configuration

#define CC1200_MDMCFG2						0x05 // General Modem Parameter Configuration

#define CC1200_EXT_CTRL						0x06 // External Control Configuration

#define CC1200_RCCAL_FINE					0x07 // RC Oscillator Calibration Fine

#define CC1200_RCCAL_COURSE					0x08 // RC Oscillator Calibration Course

#define CC1200_RCCAL_OFFSET					0x09 // RC Oscillator Calibration Clock Offset

#define CC1200_FREQOFF1						0x0A // Frequency Offset MSB
#define CC1200_FREQOFF0						0x0B // Frequency Offset LSB

#define CC1200_FREQ2						0x0C // Frequency Configuration [23:16]
#define CC1200_FREQ1						0x0D // Frequency Configuraton [15:8]
#define CC1200_FREQ0						0x0E // Frequency Configuraton [7:0]

#define CC1200_IF_ADC2						0x0F // Analog to Digital Converter Configuration Reg. 2
#define CC1200_IF_ADC1						0x10 // Analog to Digital Converter Configuration Reg. 1
#define CC1200_IF_ADC0						0x11 // Analog to Digital Converter Configuration Reg. 0

#define CC1200_FS_DIG1						0x12 // Frequency Synthesizer Digital Reg. 1
#define CC1200_FS_DIG0						0x13 // Frequency Synthesizer Digital Reg. 0

#define CC1200_FS_CAL3						0x14 // Frequency Synthesizer Calibration Reg. 3
#define CC1200_FS_CAL2						0x15 // Frequency Synthesizer Calibration Reg. 2
#define CC1200_FS_CAL1						0x16 // Frequency Synthesizer Calibration Reg. 1
#define CC1200_FS_CAL0						0x17 // Frequency Synthesizer Calibration Reg. 0

#define CC1200_FS_CHP						0x18 // Frequency Synthesizer Charge Pump Configuration

#define CC1200_FS_DIVTWO					0x19 // Frequency Synthesizer Divide by 2

#define CC1200_FS_DSM1						0x1A // FS Digital Synthesizer Module Configuration Reg. 1
#define CC1200_FS_DSM0						0x1B // FS Digital Synthesizer Module Configuration Reg. 0

#define CC1200_FS_DVC1						0x1C // FS Digital Synthesizer Divider Chain Configuration Reg. 1
#define CC1200_FS_DVC0						0x1D // FS Digital Synthesizer Divider Chain Configuration Reg. 0

#define CC1200_FS_LBI						0x1E // Frequency Synthesizer Local Bias Configuration

#define CC1200_FS_PFD						0x1F // Frequency Synthesizer Phase Frequency Detector Configuration

#define CC1200_FS_PRE						0x20 // Frequency Synthesizer Prescaler Configuration

#define CC1200_FS_REG_DIV_CML				0x21 // Frequency Synthesizer Divider Regulator Configuration

#define CC1200_FS_SPARE						0x22 // Frequency Synthesizer Spare

#define CC1200_FS_VCO4						0x23 // FS Voltage Controlled Oscillator Configuration Reg. 4
#define CC1200_FS_VCO3						0x24 // FS Voltage Controlled Oscillator Configuration Reg. 3
#define CC1200_FS_VCO2						0x25 // FS Voltage Controlled Oscillator Configuration Reg. 2
#define CC1200_FS_VCO1						0x26 // FS Voltage Controlled Oscillator Configuration Reg. 1
#define CC1200_FS_VCO0						0x27 // FS Voltage Controlled Oscillator Configuration Reg. 0

#define CC1200_GBIAS6						0x28 // Global Bias Configuration Reg. 6
#define CC1200_GBIAS5						0x29 // Global Bias Configuration Reg. 5
#define CC1200_GBIAS4						0x2A // Global Bias Configuration Reg. 4
#define CC1200_GBIAS3						0x2B // Global Bias Configuration Reg. 3
#define CC1200_GBIAS2						0x2C // Global Bias Configuration Reg. 2
#define CC1200_GBIAS1						0x2D // Global Bias Configuration Reg. 1
#define CC1200_GBIAS0						0x2E // Global Bias Configuration Reg. 0

#define CC1200_IFAMP						0x2F // Intermediate Frequency Amplifier Configuration

#define CC1200_LNA							0x30 // Low Noise Amplifier Configuration

#define CC1200_RXMIX						0x31 // RX Mixer Configuration

#define CC1200_XOSC5						0x32 // Crystal Oscillator Configuration Reg. 5
#define CC1200_XOSC4						0x33 // Crystal Oscillator Configuration Reg. 4
#define CC1200_XOSC3						0x34 // Crystal Oscillator Configuration Reg. 3
#define CC1200_XOSC2						0x35 // Crystal Oscillator Configuration Reg. 2
#define CC1200_XOSC1						0x36 // Crystal Oscillator Configuration Reg. 1
#define CC1200_XOSC0						0x37 // Crystal Oscillator Configuration Reg. 0

#define CC1200_ANALOG_SPARE					0x38 // Analog Spare

#define CC1200_PA_CFG3						0x39 // Power Amplifier Configuration

#define CC1200_WOR_TIME1					0x64 // eWOR Timer Counter Value MSB
#define CC1200_WOR_TIME0					0x65 // eWOR Timer Counter Value LSB

#define CC1200_WOR_CAPTURE1					0x66 // eWOR Timer Capture Value MSB
#define CC1200_WOR_CAPTURE0					0x67 // eWOR Timer Capture Value LSB

#define CC1200_BIST							0x68 // MARC Built-In Self-Test

#define CC1200_DCFILTOFFSET_I1				0x69 // DC Filter Offset I MSB

#define CC1200_DCFILTOFFSET_I0				0x6A // DC Filter Offset I LSB

#define CC1200_DCFILTOFFSET_Q1				0x6B // DC Filter Offset Q MSB

#define CC1200_DCFILTOFFSET_Q0				0x6C // DC Filter Offset Q LSB

#define CC1200_IQIE_I1						0x6D // IQ Imbalance Value I MSB
#define CC1200_IQIE_I0						0x6E // IQ Imbalance Value I LSB

#define CC1200_IQIE_Q1						0x6F // IQ Imbalance Value Q MSB
#define CC1200_IQIE_Q0						0x70 // IQ Imbalance Value Q LSB

#define CC1200_RSSI1						0x71 // Receive Signal Strength Indicator Reg. 1
#define CC1200_RSSI0						0x72 // Receive Signal Strength Indicator Reg. 0

#define CC1200_MARCSTATE					0x73 // MARC State

#define CC1200_LQI_VAL						0x74 // Link Quality Indicator Value

#define CC1200_PQT_SYNC_ERR					0x75 // Preamble and Sync Word Error

#define CC1200_DEM_STATUS					0x76 // Demodulator Status

#define CC1200_FREQOFF_EST1					0x77 // Frequency Offset Estimate MSB

#define CC1200_FREQOFF_EST0					0x78 // Frequency Offset Estimate LSB

#define CC1200_AGC_GAIN3					0x79 // Automatic Gain Control Reg. 3
#define CC1200_AGC_GAIN2					0x7A // Automatic Gain Control Reg. 2
#define CC1200_AGC_GAIN1					0x7B // Automatic Gain Control Reg. 1
#define CC1200_AGC_GAIN0					0x7C // Automatic Gain Control Reg. 0

#define CC1200_CFM_RX_DATA_OUT				0x7D // Custom Frequency Modulation RX Data

#define CC1200_CFM_TX_DATA_IN				0x7E // Custom Frequency Modulation TX Data

#define CC1200_ASK_SOFT_RX_DATA				0x7F // ASK Soft Decision Output

#define CC1200_RNDGEN						0x80 // Random Number Generator Value

#define CC1200_MAGN2						0x81 // Signal Magnitude after CORDIC [16]
#define CC1200_MAGN1						0x82 // Signal Magnitude after CORDIC [15:8]
#define CC1200_MAGN0						0x83 // Signal Magnitude after CORDIC [7:0]

#define CC1200_ANG1							0x84 // Signal Angular after CORDIC [9:8]
#define CC1200_ANG0							0x85 // Signal Angular after CORDIC [7:0]

#define CC1200_CHFILT_I2					0x86 // Channel Filter Data Real Part [16]
#define CC1200_CHFILT_I1					0x87 // Channel Filter Data Real Part [15:8]
#define CC1200_CHFILT_I0					0x88 // Channel Filter Data Real Part [7:0]

#define CC1200_CHFILT_Q2					0x89 // Channel Filter Data Imaginary Part [16]
#define CC1200_CHFILT_Q1					0x8A // Channel Filter Data Imaginary Part [15:8]
#define CC1200_CHFILT_Q0					0x8B // Channel Filter Data Imaginary Part [7:0]

#define CC1200_GPIO_STATUS					0x8C // General Purpose Input/Output Status

#define CC1200_FSCAL_CTRL					0x8D // Frequency Synthesizer Calibration Control

#define CC1200_PHASE_ADJUST					0x8E // Frequency Synthesizer Phase Adjust

#define CC1200_PARTNUMBER					0x8F // Part Number

#define CC1200_PARTVERSION					0x90 // Part Revision

#define CC1200_SERIAL_STATUS				0x91 // Serial Status

#define CC1200_MODEM_STATUS1				0x92 // Modem Status Reg. 1
#define CC1200_MODEM_STATUS0				0x93 // Modem Status Reg. 0

#define CC1200_MARC_STATUS1					0x94 // MARC Status Reg. 1
#define CC1200_MARC_STATUS0					0x95 // MARC Status Reg. 0

#define CC1200_PA_IFAMP_TEST				0x96 // Power Amplifier Intermediate Frequency Amplifier Test

#define CC1200_FSRF_TEST					0x97 // Frequency Synthesizer Test

#define CC1200_PRE_TEST						0x98 // Frequency Synthesizer Prescaler Test

#define CC1200_PRE_OVR						0x99 // Frequency Synthesizer Prescaler Override

#define CC1200_ADC_TEST						0x9A // Analog to Digital Converter Test

#define CC1200_DVC_TEST						0x9B // Digital Divide Chain Test

#define CC1200_ATEST						0x9C // Analog Test

#define CC1200_ATEST_LVDS					0x9D // Analog Test LVDS

#define CC1200_ATEST_MODE					0x9E // Analog Test Mode

#define CC1200_XOSC_TEST1					0x9F // Crystal Oscillator Test Reg. 1
#define CC1200_XOSC_TEST0					0xA0 // Crystal Oscillator Test Reg. 0

#define CC1200_AES							0xA1 // Advanced Encryption Standard Status

#define CC1200_MDM_TEST						0xA2 // Modem Test

#define CC1200_RXFIRST						0xD2 // RX FIFO Pointer First Entry

#define CC1200_TXFIRST						0xD3 // TX FIFO Pointer First Entry

#define CC1200_RXLAST						0xD4 // RX FIFO Pointer Last Entry

#define CC1200_TXLAST						0xD5 // TX FIFO Point Last Entry

#define CC1200_NUM_TXBYTES					0xD6 // TX FIFO Status

#define CC1200_NUM_RXBYTES					0xD7 // RX FIFO Status

#define CC1200_FIFO_NUM_TXBYTES				0xD8 // TX FIFO Status

#define CC1200_FIFO_NUM_RXBYTES				0xD9 // RX FIFO Status

#define CC1200_RXFIFO_PRE_BUF				0xDA // RX FIFO First Byte

/* Command Strobes */

#define CC1200_COMMAND_SRES					0x30 // Reset Chip
#define CC1200_COMMAND_SFSTXON				0x31 // Enable and calibrate frequency synthesizer
#define CC1200_COMMAND_SXOFF				0x32 // Enter XOFF state when CSn is de-asserted
#define CC1200_COMMAND_SCAL					0x33 // Calibrate frequency synthesizer
#define CC1200_COMMAND_SRX					0x34 // Enable RX
#define CC1200_COMMAND_STX					0x35 // Enable TX
#define CC1200_COMMAND_SIDLE				0x36 // Exit RX/TX, turn of frequency synthesizer and exit eWOR mode
#define CC1200_COMMAND_SAFC					0x37 // Automatic Frequency Compensation
#define CC1200_COMMAND_SWOR					0x38 // Start automatic RX polling sequence (eWOR) as shown in Sec 9.6
#define CC1200_COMMAND_SPWD					0x39 // Enter SLEEP mode when CSN is de-asserted
#define CC1200_COMMAND_SFRX					0x3A // Flush the RX FIFO
#define CC1200_COMMAND_SFTX					0x3B // Flush the TX FIFO
#define CC1200_COMMAND_SWORRST				0x3C // Reset the eWOR timer to the Event 1 value
#define CC1200_COMMAND_SNOP					0x3D // No operation

#endif /* INC_CC1200_REGISTERS_H_ */
