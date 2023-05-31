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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CC1200_t SPI_Info; // structure with MISO data buffer, GPIO CS Port/Pin, and SPI handler
uint8_t MISO_Data[1]; // MISO data buffer
uint8_t RX_Packet[128]; // RX packet


/* RF Parameters
 * Carrier Frequency: 436.499939 MHz
 * Xtal Frequency:    40.000000 MHz
 * Symbol Rate:       56 ksps
 * Bit Rate:          112 kbps
 * RX Filter BW:      185.185185 kHz
 * Modulation Format: 4-GFSK
 * Deviation:         50.048828 kHz
 * TX Power:          14 dBm
 * Manchester Enable: No
 * PA Ramping:        Yes
 * Whitening:         No
 * Performance Mode:  High Performance
 * */

/* Packet TX
 * Length Config:    Variable
 * Preamble Count:   3 bytes
 * Sync Word Length: 16 bits
 * Add Seq. Number
 */

RegisterSetting_t Transmit_Register_Settings[] =
{
	{CC1200_IOCFG3,              0x06},
	{CC1200_IOCFG2,              0x06}, // Smart RF value 0x06
	{CC1200_IOCFG1,              0x30},
	{CC1200_IOCFG0,              0x3C},
	{CC1200_SYNC3,               0x00},
	{CC1200_SYNC2,               0x00},
	{CC1200_SYNC1,               0xD9},
	{CC1200_SYNC0,               0xCC},
	{CC1200_SYNC_CFG1,           0x47},
	{CC1200_SYNC_CFG0,           0x00},
	{CC1200_DEVIATION_M,         0x48},
	{CC1200_MODCFG_DEV_E,        0x2C},
	{CC1200_DCFILT_CFG,          0x4B},
	{CC1200_PREAMBLE_CFG1,       0x14},
	{CC1200_PREAMBLE_CFG0,       0x8A},
	{CC1200_IQIC,                0xEC},
	{CC1200_CHAN_BW,             0x09},
	{CC1200_MDMCFG1,             0x42},
	{CC1200_MDMCFG0,             0x05},
	{CC1200_SYMBOL_RATE2,        0x96},
	{CC1200_SYMBOL_RATE1,        0xF0},
	{CC1200_SYMBOL_RATE0,        0x07},
	{CC1200_AGC_REF,             0x2F},
	{CC1200_AGC_CS_THR,          0xF6},
	{CC1200_AGC_GAIN_ADJUST,     0x00},
	{CC1200_AGC_CFG3,            0xB1},
	{CC1200_AGC_CFG2,            0x20},
	{CC1200_AGC_CFG1,            0x12},
	{CC1200_AGC_CFG0,            0x84},
	{CC1200_FIFO_CFG,            0x00},
	{CC1200_DEV_ADDR,            0x00},
	{CC1200_SETTLING_CFG,        0x0B},
	{CC1200_FS_CFG,              0x14},
	{CC1200_WOR_CFG1,            0x08},
	{CC1200_WOR_CFG0,            0x21},
	{CC1200_WOR_EVENT0_MSB,      0x00},
	{CC1200_WOR_EVENT0_LSB,      0x00},
	{CC1200_RXDCM_TIME,          0x00},
	{CC1200_PKT_CFG2,            0x00},
	{CC1200_PKT_CFG1,            0x03}, // transmit pkt cfg 1 is 0x03
	{CC1200_PKT_CFG0,            0x20},
	{CC1200_RFEND_CFG1,          0x0F},
	{CC1200_RFEND_CFG0,          0x00},
	{CC1200_PA_CFG1,             0x7F},
	{CC1200_PA_CFG0,             0x53},
	{CC1200_ASK_CFG,             0x0F},
	{CC1200_PKT_LEN,             0xFF},
};

RegisterSetting_t Transmit_Extended_Register_Settings[] =
{
	{CC1200_IF_MIX_CFG,          0x1C},
	{CC1200_FREQOFF_CFG,         0x20},
	{CC1200_TOC_CFG,             0x03},
	{CC1200_MARC_SPARE,          0x00},
	{CC1200_ECG_CFG,             0x00},
	{CC1200_MDMCFG2,             0x02},
	{CC1200_EXT_CTRL,            0x01},
	{CC1200_RCCAL_FINE,          0x00},
	{CC1200_RCCAL_COURSE,        0x00},
	{CC1200_RCCAL_OFFSET,        0x00},
	{CC1200_FREQOFF1,            0x00},
	{CC1200_FREQOFF0,            0x00},
	{CC1200_FREQ2,               0x57},
	{CC1200_FREQ1,               0x4C},
	{CC1200_FREQ0,               0xCC},
	{CC1200_IF_ADC2,             0x02},
	{CC1200_IF_ADC1,             0xEE},
	{CC1200_IF_ADC0,             0x10},
	{CC1200_FS_DIG1,             0x04},
	{CC1200_FS_DIG0,             0x50},
	{CC1200_FS_CAL3,             0x00},
	{CC1200_FS_CAL2,             0x20},
	{CC1200_FS_CAL1,             0x40},
	{CC1200_FS_CAL0,             0x0E},
	{CC1200_FS_CHP,              0x28},
	{CC1200_FS_DIVTWO,           0x03},
	{CC1200_FS_DSM1,             0x00},
	{CC1200_FS_DSM0,             0x33},
	{CC1200_FS_DVC1,             0xF7},
	{CC1200_FS_DVC0,             0x0F},
	{CC1200_FS_LBI,              0x00},
	{CC1200_FS_PFD,              0x00},
	{CC1200_FS_PRE,              0x6E},
	{CC1200_FS_REG_DIV_CML,      0x1C},
	{CC1200_FS_SPARE,            0xAC},
	{CC1200_FS_VCO4,             0x14},
	{CC1200_FS_VCO3,             0x00},
	{CC1200_FS_VCO2,             0x00},
	{CC1200_FS_VCO1,             0x00},
	{CC1200_FS_VCO0,             0xB5},
	{CC1200_GBIAS6,              0x00},
	{CC1200_GBIAS5,              0x02},
	{CC1200_GBIAS4,              0x00},
	{CC1200_GBIAS3,              0x00},
	{CC1200_GBIAS2,              0x10},
	{CC1200_GBIAS1,              0x00},
	{CC1200_GBIAS0,              0x00},
	{CC1200_IFAMP,               0x09},
	{CC1200_LNA,                 0x01},
	{CC1200_RXMIX,               0x01},
	{CC1200_XOSC5,               0x0E},
	{CC1200_XOSC4,               0xA0},
	{CC1200_XOSC3,               0x03},
	{CC1200_XOSC2,               0x04},
	{CC1200_XOSC1,               0x03},
	{CC1200_XOSC0,               0x00},
	{CC1200_ANALOG_SPARE,        0x00},
	{CC1200_PA_CFG3,             0x00},
	{CC1200_WOR_TIME1,           0x00},
	{CC1200_WOR_TIME0,           0x00},
	{CC1200_WOR_CAPTURE1,        0x00},
	{CC1200_WOR_CAPTURE0,        0x00},
	{CC1200_BIST,                0x00},
	{CC1200_DCFILTOFFSET_I1,     0x00},
	{CC1200_DCFILTOFFSET_I0,     0x00},
	{CC1200_DCFILTOFFSET_Q1,     0x00},
	{CC1200_DCFILTOFFSET_Q0,     0x00},
	{CC1200_IQIE_I1,             0x00},
	{CC1200_IQIE_I0,             0x00},
	{CC1200_IQIE_Q1,             0x00},
	{CC1200_IQIE_Q0,             0x00},
	{CC1200_RSSI1,               0x80},
	{CC1200_RSSI0,               0x00},
	{CC1200_MARCSTATE,           0x41},
	{CC1200_LQI_VAL,             0x00},
	{CC1200_PQT_SYNC_ERR,        0xFF},
	{CC1200_DEM_STATUS,          0x00},
	{CC1200_FREQOFF_EST1,        0x00},
	{CC1200_FREQOFF_EST0,        0x00},
	{CC1200_AGC_GAIN3,           0x00},
	{CC1200_AGC_GAIN2,           0xD1},
	{CC1200_AGC_GAIN1,           0x00},
	{CC1200_AGC_GAIN0,           0x3F},
	{CC1200_CFM_RX_DATA_OUT,     0x00},
	{CC1200_CFM_TX_DATA_IN,      0x00},
	{CC1200_ASK_SOFT_RX_DATA,    0x30},
	{CC1200_RNDGEN,              0x7F},
	{CC1200_MAGN2,               0x00},
	{CC1200_MAGN1,               0x00},
	{CC1200_MAGN0,               0x00},
	{CC1200_ANG1,                0x00},
	{CC1200_ANG0,                0x00},
	{CC1200_CHFILT_I2,           0x02},
	{CC1200_CHFILT_I1,           0x00},
	{CC1200_CHFILT_I0,           0x00},
	{CC1200_CHFILT_Q2,           0x00},
	{CC1200_CHFILT_Q1,           0x00},
	{CC1200_CHFILT_Q0,           0x00},
	{CC1200_GPIO_STATUS,         0x00},
	{CC1200_FSCAL_CTRL,          0x01},
	{CC1200_PHASE_ADJUST,        0x00},
	{CC1200_PARTNUMBER,          0x20},
	{CC1200_PARTVERSION,         0x11},
	{CC1200_SERIAL_STATUS,       0x00},
	{CC1200_MODEM_STATUS1,       0x10},
	{CC1200_MODEM_STATUS0,       0x00},
	{CC1200_MARC_STATUS1,        0x00},
	{CC1200_MARC_STATUS0,        0x00},
	{CC1200_PA_IFAMP_TEST,       0x00},
	{CC1200_FSRF_TEST,           0x00},
	{CC1200_PRE_TEST,            0x00},
	{CC1200_PRE_OVR,             0x00},
	{CC1200_ADC_TEST,            0x00},
	{CC1200_DVC_TEST,            0x0B},
	{CC1200_ATEST,               0x40},
	{CC1200_ATEST_LVDS,          0x00},
	{CC1200_ATEST_MODE,          0x00},
	{CC1200_XOSC_TEST1,          0x3C},
	{CC1200_XOSC_TEST0,          0x00},
	{CC1200_AES,                 0x00},
	{CC1200_MDM_TEST,            0x00},
	{CC1200_RXFIRST,             0x00},
	{CC1200_TXFIRST,             0x00},
	{CC1200_RXLAST,              0x00},
	{CC1200_TXLAST,              0x00},
	{CC1200_NUM_TXBYTES,         0x00},
	{CC1200_NUM_RXBYTES,         0x00},
	{CC1200_FIFO_NUM_TXBYTES,    0x0F},
	{CC1200_FIFO_NUM_RXBYTES,    0x00},
	{CC1200_RXFIFO_PRE_BUF,      0x00},
//	{CC1200_AES_KEY15,           0x00},
//	{CC1200_AES_KEY14,           0x00},
//	{CC1200_AES_KEY13,           0x00},
//	{CC1200_AES_KEY12,           0x00},
//	{CC1200_AES_KEY11,           0x00},
//	{CC1200_AES_KEY10,           0x00},
//	{CC1200_AES_KEY9,            0x00},
//	{CC1200_AES_KEY8,            0x00},
//	{CC1200_AES_KEY7,            0x00},
//	{CC1200_AES_KEY6,            0x00},
//	{CC1200_AES_KEY5,            0x00},
//	{CC1200_AES_KEY4,            0x00},
//	{CC1200_AES_KEY3,            0x00},
//	{CC1200_AES_KEY2,            0x00},
//	{CC1200_AES_KEY1,            0x00},
//	{CC1200_AES_KEY0,            0x00},
//	{CC1200_AES_BUFFER15,        0x00},
//	{CC1200_AES_BUFFER14,        0x00},
//	{CC1200_AES_BUFFER13,        0x00},
//	{CC1200_AES_BUFFER12,        0x00},
//	{CC1200_AES_BUFFER11,        0x00},
//	{CC1200_AES_BUFFER10,        0x00},
//	{CC1200_AES_BUFFER9,         0x00},
//	{CC1200_AES_BUFFER8,         0x00},
//	{CC1200_AES_BUFFER7,         0x00},
//	{CC1200_AES_BUFFER6,         0x00},
//	{CC1200_AES_BUFFER5,         0x00},
//	{CC1200_AES_BUFFER4,         0x00},
//	{CC1200_AES_BUFFER3,         0x00},
//	{CC1200_AES_BUFFER2,         0x00},
//	{CC1200_AES_BUFFER1,         0x00},
//	{CC1200_AES_BUFFER0,         0x00},
};

/* Packet RX
 * Length Config:    Variable
 * Seq. Number Included in Payload
 */

RegisterSetting_t Receive_Register_Settings[] =
{
  {CC1200_IOCFG3,              0x06},
  {CC1200_IOCFG2,              0x06}, // Smart RF value 0x06
  {CC1200_IOCFG1,              0x30},
  {CC1200_IOCFG0,              0x3C},
  {CC1200_SYNC3,               0x00},
  {CC1200_SYNC2,               0x00},
  {CC1200_SYNC1,               0xD9},
  {CC1200_SYNC0,               0xCC},
  {CC1200_SYNC_CFG1,           0x47},
  {CC1200_SYNC_CFG0,           0x00},
  {CC1200_DEVIATION_M,         0x48},
  {CC1200_MODCFG_DEV_E,        0x2C},
  {CC1200_DCFILT_CFG,          0x4B},
  {CC1200_PREAMBLE_CFG1,       0x14},
  {CC1200_PREAMBLE_CFG0,       0x8A},
  {CC1200_IQIC,                0xEC},
  {CC1200_CHAN_BW,             0x09},
  {CC1200_MDMCFG1,             0x42},
  {CC1200_MDMCFG0,             0x05},
  {CC1200_SYMBOL_RATE2,        0x96},
  {CC1200_SYMBOL_RATE1,        0xF0},
  {CC1200_SYMBOL_RATE0,        0x07},
  {CC1200_AGC_REF,             0x29},
  {CC1200_AGC_CS_THR,          0xF6},
  {CC1200_AGC_GAIN_ADJUST,     0x00},
  {CC1200_AGC_CFG3,            0xB1},
  {CC1200_AGC_CFG2,            0x20},
  {CC1200_AGC_CFG1,            0x12},
  {CC1200_AGC_CFG0,            0x84},
  {CC1200_FIFO_CFG,            0x00},
  {CC1200_DEV_ADDR,            0x00},
  {CC1200_SETTLING_CFG,        0x0B},
  {CC1200_FS_CFG,              0x14},
  {CC1200_WOR_CFG1,            0x08},
  {CC1200_WOR_CFG0,            0x21},
  {CC1200_WOR_EVENT0_MSB,      0x00},
  {CC1200_WOR_EVENT0_LSB,      0x00},
  {CC1200_RXDCM_TIME,          0x00},
  {CC1200_PKT_CFG2,            0x00},
  {CC1200_PKT_CFG1,            0x03}, // receive pkt cfg1 is 0x03
  {CC1200_PKT_CFG0,            0x20},
  {CC1200_RFEND_CFG1,          0x0F},
  {CC1200_RFEND_CFG0,          0x00},
  {CC1200_PA_CFG1,             0x7F},
  {CC1200_PA_CFG0,             0x53},
  {CC1200_ASK_CFG,             0x0F},
  {CC1200_PKT_LEN,             0xFF},
};

RegisterSetting_t Receive_Extended_Register_Settings[] =
{
	{CC1200_IF_MIX_CFG,          0x1C},
	{CC1200_FREQOFF_CFG,         0x20},
	{CC1200_TOC_CFG,             0x03},
	{CC1200_MARC_SPARE,          0x00},
	{CC1200_ECG_CFG,             0x00},
	{CC1200_MDMCFG2,             0x02},
	{CC1200_EXT_CTRL,            0x01},
	{CC1200_RCCAL_FINE,          0x00},
	{CC1200_RCCAL_COURSE,        0x00},
	{CC1200_RCCAL_OFFSET,        0x00},
	{CC1200_FREQOFF1,            0x00},
	{CC1200_FREQOFF0,            0x00},
	{CC1200_FREQ2,               0x57},
	{CC1200_FREQ1,               0x4C},
	{CC1200_FREQ0,               0xCC},
	{CC1200_IF_ADC2,             0x02},
	{CC1200_IF_ADC1,             0xEE},
	{CC1200_IF_ADC0,             0x10},
	{CC1200_FS_DIG1,             0x07},
	{CC1200_FS_DIG0,             0xA5},
	{CC1200_FS_CAL3,             0x00},
	{CC1200_FS_CAL2,             0x20},
	{CC1200_FS_CAL1,             0x40},
	{CC1200_FS_CAL0,             0x0E},
	{CC1200_FS_CHP,              0x28},
	{CC1200_FS_DIVTWO,           0x03},
	{CC1200_FS_DSM1,             0x00},
	{CC1200_FS_DSM0,             0x33},
	{CC1200_FS_DVC1,             0xFF},
	{CC1200_FS_DVC0,             0x17},
	{CC1200_FS_LBI,              0x00},
	{CC1200_FS_PFD,              0x00},
	{CC1200_FS_PRE,              0x6E},
	{CC1200_FS_REG_DIV_CML,      0x1C},
	{CC1200_FS_SPARE,            0xAC},
	{CC1200_FS_VCO4,             0x14},
	{CC1200_FS_VCO3,             0x00},
	{CC1200_FS_VCO2,             0x00},
	{CC1200_FS_VCO1,             0x00},
	{CC1200_FS_VCO0,             0xB5},
	{CC1200_GBIAS6,              0x00},
	{CC1200_GBIAS5,              0x02},
	{CC1200_GBIAS4,              0x00},
	{CC1200_GBIAS3,              0x00},
	{CC1200_GBIAS2,              0x10},
	{CC1200_GBIAS1,              0x00},
	{CC1200_GBIAS0,              0x00},
	{CC1200_IFAMP,               0x09},
	{CC1200_LNA,                 0x01},
	{CC1200_RXMIX,               0x01},
	{CC1200_XOSC5,               0x0E},
	{CC1200_XOSC4,               0xA0},
	{CC1200_XOSC3,               0x03},
	{CC1200_XOSC2,               0x04},
	{CC1200_XOSC1,               0x03},
	{CC1200_XOSC0,               0x00},
	{CC1200_ANALOG_SPARE,        0x00},
	{CC1200_PA_CFG3,             0x00},
	{CC1200_WOR_TIME1,           0x00},
	{CC1200_WOR_TIME0,           0x00},
	{CC1200_WOR_CAPTURE1,        0x00},
	{CC1200_WOR_CAPTURE0,        0x00},
	{CC1200_BIST,                0x00},
	{CC1200_DCFILTOFFSET_I1,     0x00},
	{CC1200_DCFILTOFFSET_I0,     0x00},
	{CC1200_DCFILTOFFSET_Q1,     0x00},
	{CC1200_DCFILTOFFSET_Q0,     0x00},
	{CC1200_IQIE_I1,             0x00},
	{CC1200_IQIE_I0,             0x00},
	{CC1200_IQIE_Q1,             0x00},
	{CC1200_IQIE_Q0,             0x00},
	{CC1200_RSSI1,               0x80},
	{CC1200_RSSI0,               0x00},
	{CC1200_MARCSTATE,           0x41},
	{CC1200_LQI_VAL,             0x00},
	{CC1200_PQT_SYNC_ERR,        0xFF},
	{CC1200_DEM_STATUS,          0x00},
	{CC1200_FREQOFF_EST1,        0x00},
	{CC1200_FREQOFF_EST0,        0x00},
	{CC1200_AGC_GAIN3,           0x00},
	{CC1200_AGC_GAIN2,           0xD1},
	{CC1200_AGC_GAIN1,           0x00},
	{CC1200_AGC_GAIN0,           0x3F},
	{CC1200_CFM_RX_DATA_OUT,     0x00},
	{CC1200_CFM_TX_DATA_IN,      0x00},
	{CC1200_ASK_SOFT_RX_DATA,    0x30},
	{CC1200_RNDGEN,              0x7F},
	{CC1200_MAGN2,               0x00},
	{CC1200_MAGN1,               0x00},
	{CC1200_MAGN0,               0x00},
	{CC1200_ANG1,                0x00},
	{CC1200_ANG0,                0x00},
	{CC1200_CHFILT_I2,           0x02},
	{CC1200_CHFILT_I1,           0x00},
	{CC1200_CHFILT_I0,           0x00},
	{CC1200_CHFILT_Q2,           0x00},
	{CC1200_CHFILT_Q1,           0x00},
	{CC1200_CHFILT_Q0,           0x00},
	{CC1200_GPIO_STATUS,         0x00},
	{CC1200_FSCAL_CTRL,          0x01},
	{CC1200_PHASE_ADJUST,        0x00},
	{CC1200_PARTNUMBER,          0x20},
	{CC1200_PARTVERSION,         0x11},
	{CC1200_SERIAL_STATUS,       0x00},
	{CC1200_MODEM_STATUS1,       0x10},
	{CC1200_MODEM_STATUS0,       0x00},
	{CC1200_MARC_STATUS1,        0x00},
	{CC1200_MARC_STATUS0,        0x00},
	{CC1200_PA_IFAMP_TEST,       0x00},
	{CC1200_FSRF_TEST,           0x00},
	{CC1200_PRE_TEST,            0x00},
	{CC1200_PRE_OVR,             0x00},
	{CC1200_ADC_TEST,            0x00},
	{CC1200_DVC_TEST,            0x0B},
	{CC1200_ATEST,               0x40},
	{CC1200_ATEST_LVDS,          0x00},
	{CC1200_ATEST_MODE,          0x00},
	{CC1200_XOSC_TEST1,          0x3C},
	{CC1200_XOSC_TEST0,          0x00},
	{CC1200_AES,                 0x00},
	{CC1200_MDM_TEST,            0x00},
	{CC1200_RXFIRST,             0x00},
	{CC1200_TXFIRST,             0x00},
	{CC1200_RXLAST,              0x00},
	{CC1200_TXLAST,              0x00},
	{CC1200_NUM_TXBYTES,         0x00},
	{CC1200_NUM_RXBYTES,         0x00},
	{CC1200_FIFO_NUM_TXBYTES,    0x0F},
	{CC1200_FIFO_NUM_RXBYTES,    0x00},
	{CC1200_RXFIFO_PRE_BUF,      0x00},
//	{CC1200_AES_KEY15,           0x00},
//	{CC1200_AES_KEY14,           0x00},
//	{CC1200_AES_KEY13,           0x00},
//	{CC1200_AES_KEY12,           0x00},
//	{CC1200_AES_KEY11,           0x00},
//	{CC1200_AES_KEY10,           0x00},
//	{CC1200_AES_KEY9,            0x00},
//	{CC1200_AES_KEY8,            0x00},
//	{CC1200_AES_KEY7,            0x00},
//	{CC1200_AES_KEY6,            0x00},
//	{CC1200_AES_KEY5,            0x00},
//	{CC1200_AES_KEY4,            0x00},
//	{CC1200_AES_KEY3,            0x00},
//	{CC1200_AES_KEY2,            0x00},
//	{CC1200_AES_KEY1,            0x00},
//	{CC1200_AES_KEY0,            0x00},
//	{CC1200_AES_BUFFER15,        0x00},
//	{CC1200_AES_BUFFER14,        0x00},
//	{CC1200_AES_BUFFER13,        0x00},
//	{CC1200_AES_BUFFER12,        0x00},
//	{CC1200_AES_BUFFER11,        0x00},
//	{CC1200_AES_BUFFER10,        0x00},
//	{CC1200_AES_BUFFER9,         0x00},
//	{CC1200_AES_BUFFER8,         0x00},
//	{CC1200_AES_BUFFER7,         0x00},
//	{CC1200_AES_BUFFER6,         0x00},
//	{CC1200_AES_BUFFER5,         0x00},
//	{CC1200_AES_BUFFER4,         0x00},
//	{CC1200_AES_BUFFER3,         0x00},
//	{CC1200_AES_BUFFER2,         0x00},
//	{CC1200_AES_BUFFER1,         0x00},
//	{CC1200_AES_BUFFER0,         0x00},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	char Message[10000];
//	uint16_t Message_Length;
//	char str1[150];
//
//	CC1200_Receive(&SPI_Info, RX_Packet);
//
//	sprintf(Message, "Received the Following Message: ");
//	sprintf(str1, "%s\r\n", (char*) RX_Packet);
//	strcat(Message, str1);
//	Message_Length = strlen(Message);
//
//	CDC_Transmit_FS((uint8_t*) Message, Message_Length);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//HAL_TIM_Base_Start_IT(&htim2);

	char Message[10000];
	uint16_t Message_Length;
	char str1[150];

	uint8_t check;

	check = CC1200_Receive(&SPI_Info, RX_Packet);

	if (check) // check == 1
	{
		sprintf(str1, "RX FIFO Empty!\r\n");
		strcat(Message, str1);
		Message_Length = strlen(Message);
		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
	else // check == 0
	{
		sprintf(Message, "Received the Following Message: ");
		sprintf(str1, "%s\r\n", (char*) RX_Packet);
		strcat(Message, str1);
		Message_Length = strlen(Message);

		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
	}
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // start with chip select high
	//uint8_t state;
	//uint8_t MOSI_Data[1];
	//char Message[100];
	//uint16_t Message_Length;
	//uint8_t flag = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
//		// Pin Test
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
//		HAL_Delay(100);

//		// Hello World Test
//		Message_Length = sprintf(Message, "Hello World!\r\n");
//		CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//		HAL_Delay(100); // delay 1 sec
//		Message_Length = sprintf(Message, "Bye World!\r\n");
//		CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//		HAL_Delay(1000); // delay 1 sec

//		// verify SO goes low
//		MOSI_Data[0] = 0x3D; // no operation command
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // set chip select high
//		HAL_Delay(10); // delay
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // set chip select low
//		while(flag)
//		{
//			state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6); // read MISO pin
//			if (state == 0) // if MISO pin is low, the crystal oscillator is stable
//			{
//				flag = 0;
//				Message_Length = sprintf(Message, "Chip Ready\r\n");
//				CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//				HAL_Delay(100);
//			}
//			else // otherwise, the crystal oscillator is not stable
//			{
//				flag = 1;
//				state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6); // read CS pin
//				if (state == 0)
//				{
//					Message_Length = sprintf(Message, "Chip Select Low\r\n");
//					CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//					HAL_Delay(100);
//				}
//				else
//				{
//					Message_Length = sprintf(Message, "Chip Select High\r\n");
//					CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//					HAL_Delay(100);
//				}
//				Message_Length = sprintf(Message, "Chip Not Ready\r\n");
//				CDC_Transmit_FS((uint8_t*) Message, (Message_Length));
//				HAL_Delay(1000);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // set MISO low
//			}
//		}
//		HAL_SPI_TransmitReceive(&hspi1, MOSI_Data, MISO_Data, 1, 100); // get the status byte, should be IDLE
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(100);

//		// verify Read & Write
//		CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
//		HAL_Delay(10);
//		CC1200_Write_Single_Register(&SPI_Info, 0x00, 0xAA);
//		HAL_Delay(10);
//		CC1200_Read_Single_Register(&SPI_Info, 0x00);
//		HAL_Delay(10);
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(1000);

//		// verify Extended Register Read & Write
//		CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
//		HAL_Delay(10);
//		CC1200_Write_Single_Extended_Register(&SPI_Info, 0x00, 0xFA);
//		HAL_Delay(10);
//		CC1200_Read_Single_Extended_Register(&SPI_Info, 0x00);
//		HAL_Delay(10);
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(1000);

		// verify configure
//		uint8_t Address;
//		uint8_t ConfigIndex = 0;
//		CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
//		HAL_Delay(10000);
//		for (Address = 0x00; Address < 0x2F; Address++)
//		{
//			Message_Length = sprintf(Message, "Address: 0X%02X\r\n", Address);
//			CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//			HAL_Delay(1000);
//			CC1200_Write_Single_Register(&SPI_Info, Address, Preferred_Register_Settings[ConfigIndex].Value);
//			Message_Length = sprintf(Message, "Transmitted Byte: 0X%02X\r\n", Preferred_Register_Settings[ConfigIndex].Value);
//			CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//			HAL_Delay(1000);
//			CC1200_Read_Single_Register(&SPI_Info, Address);
//			Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//			CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//			HAL_Delay(1000);
//			ConfigIndex++;
//		}


//		// verify transmit and receive
//		CC1200_Init(&SPI_Info, MISO_Data, GPIOB, GPIO_PIN_6, &hspi1);
//		HAL_Delay(10);
//
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SIDLE);
//		HAL_Delay(10);
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP);
//		HAL_Delay(10);
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(1000);
//
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_STX);
//		HAL_Delay(10);
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP);
//		HAL_Delay(10);
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(1000);
//
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SRX);
//		HAL_Delay(10);
//		CC1200_Command_Strobe(&SPI_Info, CC1200_COMMAND_SNOP);
//		HAL_Delay(10);
//		Message_Length = sprintf(Message, "Received Byte: 0X%02X\r\n", MISO_Data[0]);
//		CDC_Transmit_FS((uint8_t*) Message, Message_Length);
//		HAL_Delay(1000);

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
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
