/*
 * nRF24L01.h
 *
 *  Created on: Nov 5, 2023
 *      Author: egbra
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx.h"

// Struct to hold the various information associated with a radio module
typedef struct nRF24Handle_tag {
	GPIO_TypeDef* CSN_GPIO_Port;
	uint16_t CSN_Pin;

	GPIO_TypeDef* CE_GPIO_Port;
	uint16_t CE_Pin;

	SPI_HandleTypeDef* SPI;
} nRF24Handle;


/******************* REGISTER ADDRESSES (Pages 54-59) *****************/
#define NRF24_REG				uint8_t

#define NRF24_REG_CONFIG		0x00
#define NRF24_REG_EN_AA			0x01
#define NRF24_REG_EN_RXADDR		0x02
#define NRF24_REG_SETUP_AW		0x03
#define NRF24_REG_SETUP_RETR	0x04
#define NRF24_REG_RF_CH			0x05
#define NRF24_REG_RF_SETUP		0x06
#define NRF24_REG_STATUS		0x07
#define NRF24_REG_OBSERVE_TX	0x08
#define NRF24_REG_RPD			0x09
#define NRF24_REG_RX_ADDR_P0	0x0A
#define NRF24_REG_RX_ADDR_P1	0x0B
#define NRF24_REG_RX_ADDR_P2	0x0C
#define NRF24_REG_RX_ADDR_P3	0x0D
#define NRF24_REG_RX_ADDR_P4	0x0E
#define NRF24_REG_RX_ADDR_P5	0x0F
#define NRF24_REG_TX_ADDR		0x10
#define NRF24_REG_RX_PW_P0		0x11
#define NRF24_REG_RX_PW_P1		0x12
#define NRF24_REG_RX_PW_P2		0x13
#define NRF24_REG_RX_PW_P3		0x14
#define NRF24_REG_RX_PW_P4		0x15
#define NRF24_REG_RX_PW_P5		0x16
#define NRF24_REG_FIFO_STATUS	0x17
#define NRF24_REG_DYNPD			0x1C
#define NRF24_REG_FEATURE		0x1D

/******************* BIT POSITIONS (Pages 54-59) *********************/

#define NRF24_BIT							uint8_t

// Bit positions for config register
#define NRF24_BITS_CONFIG__MASK_RX_DR		(1 << 6)
#define NRF24_BITS_CONFIG__MASK_TX_DS		(1 << 5)
#define NRF24_BITS_CONFIG__MASK_MAX_RT		(1 << 4)
#define NRF24_BITS_CONFIG__EN_CRC			(1 << 3)
#define NRF24_BITS_CONFIG__CRCO				(1 << 2)
#define NRF24_BITS_CONFIG__PWR_UP			(1 << 1)
#define NRF24_BITS_CONFIG__PRIM_RX			(1 << 0)

// Bit positions for Enhanced Shockburst register
#define NRF24_BITS_EN_AA					(0x3F)
#define NRF24_BITS_EN_AA__ENAA_P5			(1 << 5)
#define NRF24_BITS_EN_AA__ENAA_P4			(1 << 4)
#define NRF24_BITS_EN_AA__ENAA_P3			(1 << 3)
#define NRF24_BITS_EN_AA__ENAA_P2			(1 << 2)
#define NRF24_BITS_EN_AA__ENAA_P1			(1 << 1)
#define NRF24_BITS_EN_AA__ENAA_P0			(1 << 0)

// Bit positions for Enabled RX Address register
#define NRF24_BITS_EN_RXADDR__ERX_P5		(1 << 5)
#define NRF24_BITS_EN_RXADDR__ERX_P4		(1 << 4)
#define NRF24_BITS_EN_RXADDR__ERX_P3		(1 << 3)
#define NRF24_BITS_EN_RXADDR__ERX_P2		(1 << 2)
#define NRF24_BITS_EN_RXADDR__ERX_P1		(1 << 1)
#define NRF24_BITS_EN_RXADDR__ERX_P0		(1 << 0)

// Change address widths (2:0)
#define NRF24_BITS_SETUP_AW__3BYTES			(0b01 << 0)
#define NRF24_BITS_SETUP_AW__4BYTES			(0b10 << 0)
#define NRF24_BITS_SETUP_AW__5BYTES			(0b11 << 0)

// Bit positions for changing automatic retransmission delay and retransmit count
#define NRF24_BITS_SETUP_RETR__ARD(delay) 			(delay << 4)
#define NRF24_BITS_SETUP_RETR__ARC(retransmissions) (retransmissions << 0)

// Bit positions for change the frequency channel (6:0)
#define NRF24_BITS_RF_CH__RF_CH(freq) 				(freq << 0)

// Bit positions for the RF Setup register
#define NRF24_BITS_RF_SETUP__CONT_WAVE		(1 << 7)
#define NRF24_BITS_RF_SETUP__RF_DR_LOW		(1 << 5)
#define NRF24_BITS_RF_SETUP__PLL_LOCK		(1 << 4)
#define NRF24_BITS_RF_SETUP__RF_DR_HIGH		(1 << 3)

#define NRF24_BITS_RF_SETUP__RF_DR			(0b101 << 3)
#define NRF24_BITS_RF_SETUP__RF_DR_1MBPS	(0b000 << 3)
#define NRF24_BITS_RF_SETUP__RF_DR_2MBPS	(0b001 << 3)
#define NRF24_BITS_RF_SETUP__RF_DR_250KBPS	(0b100 << 3)

#define NRF24_BITS_RF_SETUP__RF_PWR			(0b11 << 1)
#define NRF24_BITS_RF_SETUP__RF_PWR_n18db	(0b00 << 1)
#define NRF24_BITS_RF_SETUP__RF_PWR_n12db	(0b01 << 1)
#define NRF24_BITS_RF_SETUP__RF_PWR_n6db	(0b10 << 1)
#define NRF24_BITS_RF_SETUP__RF_PWR_0db		(0b11 << 1)

// Bit positions for the Status register
#define NRF24_BITS_STATUS__RX_DR			(1 << 6)
#define NRF24_BITS_STATUS__TX_DS			(1 << 5)
#define NRF24_BITS_STATUS__MAX_RT			(1 << 4)
#define NRF24_BITS_STATUS__RX_P_NO			(0x7 << 1)
#define NRF24_BITS_STATUS__TX_FULL			(1 << 0)

// Bit positions for the transmit observe register
#define NRF24_BITS_OBSERVE_TX__PLOS_CNT		(0xF << 4)
#define NRF24_BITS_OBSERVE_TX__ARC_CNT		(0xF << 0)

// Bit positions for Received Power Detector register
#define NRF24_BITS_RPD__RPD					(1 << 0)

// Bit positions for FIFO Status register
#define NRF24_BITS_FIFO_STATUS__TX_REUSE	(1 << 6)
#define NRF24_BITS_FIFO_STATUS__TX_FULL		(1 << 5)
#define NRF24_BITS_FIFO_STATUS__TX_EMPTY	(1 << 4)
#define NRF24_BITS_FIFO_STATUS__RX_FULL		(1 << 1)
#define NRF24_BITS_FIFO_STATUS__RX_EMPTY	(1 << 0)

// Bit positions for Dynamic payload length register
#define NRF24_BITS_DYNPD__DPL_P5 			(1 << 5)
#define NRF24_BITS_DYNPD__DPL_P4 			(1 << 4)
#define NRF24_BITS_DYNPD__DPL_P3 			(1 << 3)
#define NRF24_BITS_DYNPD__DPL_P2 			(1 << 2)
#define NRF24_BITS_DYNPD__DPL_P1 			(1 << 1)
#define NRF24_BITS_DYNPD__DPL_P0 			(1 << 0)

// Bit positions for feature register
#define NRF24_BITS_FEATURE__EN_DPL			(1 << 2)
#define NRF24_BITS_FEATURE__EN_ACK_PAY		(1 << 1)
#define NRF24_BITS_FEATURE__EN_DYN_ACK		(1 << 0)


/************ COMMANDS (pg. 48) ***********/

#define NRF24_CMD_R_REGISTER(reg) 			((0b000 << 5) | reg)
#define NRF24_CMD_W_REGISTER(reg) 			((0b001 << 5) | reg)

#define NRF24_CMD_R_RX_PAYLOAD				0b01100001
#define NRF24_CMD_W_TX_PAYLOAD				0b10100000
#define NRF24_CMD_FLUSH_TX					0b11100001
#define NRF24_CMD_FLUSH_RX					0b11100010
#define NRF24_CMD_REUSE_TX_PL				0b11100011
#define NRF24_CMD_R_RX_PL_WID				0b01100000

#define NRF24_CMD_W_ACK_PAYLOAD(pipe) 		((0b10101 << 3) | pipe)

#define NRF24_CMD_W_TX_PAYLOAD_NO_ACK		0b10110000
#define NRF24_CMD_NOP						0b11111111


/*************** DATA TYPES ****************/

// Power
#define NRF24_POWER 				uint8_t
#define NRF24_POWER_ON				1
#define NRF24_POWER_OFF				0

// Pipe
#define NRF24_PIPE					int8_t
#define NRF24_PIPE_ERR				-1
#define NRF24_PIPE0					0
#define NRF24_PIPE1					1
#define NRF24_PIPE2					2
#define NRF24_PIPE3					3
#define NRF24_PIPE4					4
#define NRF24_PIPE5					5

#define NRF24_PIPE_STATUS			uint8_t
#define NRF24_PIPE_ENABLE			1
#define NRF24_PIPE_DISABLE			0

// RF Power
#define NRF24_PA					uint8_t
#define NRF24_PA_MAX				0
#define NRF24_PA_HIGH				1
#define NRF24_PA_LOW				2
#define NRF24_PA_MIN				3

// Data rate
#define NRF24_DATARATE				uint8_t
#define NRF24_DATARATE_250KBPS		0
#define NRF24_DATARATE_1MBPS		1
#define NRF24_DATARATE_2MBPS		2

// Address width
#define NRF24_ADDRESSWIDTH			uint8_t
#define NRF24_ADDRESSWIDTH_3BYTES	0
#define NRF24_ADDRESSWIDTH_4BYTES	1
#define NRF24_ADDRESSWIDTH_5BYTES	2

// Auto Ack enable/disable
#define NRF24_AA_STATUS				uint8_t
#define NRF24_AA_ENABLE				1
#define NRF24_AA_DISABLE			0

/************** INTERNAL FUNCTIONS **************/

// Send a low signal to the chip select line
void _NRF24_CSN_Select(nRF24Handle* dev);

// Send a high signal to the chip select line
void _NRF24_CSN_Unselect(nRF24Handle* dev);

// Send a high signal to the chip enable line
void _NRF24_CE_Enable(nRF24Handle* dev);

// Send a low signal to the chip enable line
void _NRF24_CE_Disable(nRF24Handle* dev);

// Write a single byte to the register
void _NRF24_WriteReg(nRF24Handle* dev, uint8_t reg, uint8_t data);

// Write multiple bytes to the register
void _NRF24_WriteRegMultiByte(nRF24Handle* dev, uint8_t reg, uint8_t *data, uint8_t len);

// Read a single byte from the register
uint8_t _NRF24_ReadReg(nRF24Handle* dev, uint8_t reg);

// Read multiple bytes from the register
void _NRF24_ReadRegMultiByte(nRF24Handle* dev, uint8_t reg, uint8_t *data, uint8_t len);

// Send a single command
// For commands, see pg. 48
void _NRF24_SendCmd(nRF24Handle* dev, uint8_t cmd);

// Initialize the addresses for each pipe to their default value
void _NRF24_InitAddresses(nRF24Handle* pDev);

// Issue several NOP commands to make a short delay before reading the Status register
void _NRF24_Delay(nRF24Handle* pDev);

// For whatever reason, the first write to the NRF24 sometimes doesn't work
// By writing a 0x0 to the config, which shouldn't work, I hope to get this
// initial invalid write out of the way to begin with
void _NRF24_FirstWrite(nRF24Handle* pDev);

// Read the payload from the RX FIFO
void _NRF24_ReadPayload(nRF24Handle* pDev, uint8_t *pData);

// Set a bit in a single-byte register
void _NRF24_SetRegBit(nRF24Handle* pDev, NRF24_REG reg, NRF24_BIT iBit);

// Clear a bit in a single-byte register
void _NRF24_ClearRegBit(nRF24Handle* pDev, NRF24_REG reg, NRF24_BIT iBit);

/************** API FUNCTIONS *************/
// Initialization
void NRF24_Init(nRF24Handle* dev);

// Configure for TX mode
void NRF24_TXMode(nRF24Handle* dev, uint8_t* addr, uint8_t channel);

// Configure for RX mode
void NRF24_RXMode(nRF24Handle* pDev, uint8_t* addr, uint8_t iChannel);

// Transmit up to 32 bytes
uint8_t NRF24_Transmit(nRF24Handle* pDev, uint8_t* pData, uint8_t len);

// Receive up to 32 bytes, blocking
// Return the pipe we receive from if it's a message, -1 if otherwise
NRF24_PIPE NRF24_Receive(nRF24Handle* pDev, uint8_t* pData);

// Print values from the registers in the radio module
void NRF24_PrintDebugInfo(nRF24Handle* pDev);

// Test the ability to read from and write to a multi-byte register
uint8_t NRF24_Test(nRF24Handle* pDev);

// Toggle the power on the module
void NRF24_SetPower(nRF24Handle* pDev, NRF24_POWER iPower);

/********* Setters **********/

// Set the channel for the chip to operate on
// Must be a number from 0 - 127, or 0 to disable the pipe
void NRF24_SetChannel(nRF24Handle* pDev, uint8_t iChannel);

// Enable/disable a pipe
void NRF24_SetPipeStatus(nRF24Handle* pDev, NRF24_PIPE iPipe, NRF24_PIPE_STATUS iStatus);

// Set the RF power level in dBm
void NRF24_SetPA(nRF24Handle* pDev, NRF24_PA iRFPower);

// Set the data rate of the transmission
void NRF24_SetDataRate(nRF24Handle* pDev, NRF24_DATARATE iDataRate);

// Set the address width
void NRF24_SetAddressWidth(nRF24Handle* pDev, NRF24_ADDRESSWIDTH iAW);

// Set the payload width at a particular pipe
// Must be an integer from 1 to 32 bytes
void NRF24_SetPayloadWidth(nRF24Handle* pDev, NRF24_PIPE iPipe, uint8_t iPW);

// Set Auto Ack abilities
void NRF24_SetAutoAck(nRF24Handle* pDev, NRF24_PIPE iPipe, NRF24_AA_STATUS iStatus);

// Enable/disable auto ack on all pipes
void NRF24_SetAutoAckAllPipes(nRF24Handle* pDev, NRF24_AA_STATUS iStatus);

// Set the address of one of the RX pipes
void NRF24_SetRXAddr(nRF24Handle* pDev, NRF24_PIPE iPipe, uint8_t* pAddr);

// Set the TX address
void NRF24_SetTXAddr(nRF24Handle* pDev, uint8_t* pAddr);

/********** Getters **********/

// Get the address width, in bytes
// Return: 3, 4, or 5
uint8_t NRF24_GetAddressWidth(nRF24Handle* pDev);


#endif /* INC_NRF24L01_H_ */
