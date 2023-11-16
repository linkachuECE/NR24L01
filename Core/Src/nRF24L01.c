/*
 * nRF24L01.c
 *
 *  Created on: Nov 5, 2023
 *      Author: egbra
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "nRF24L01.h"
#include <stdio.h>
#include <stdlib.h>

/************* INTERNAL FUNCTIONS *************/

// Send a low signal to the chip select line
void _NRF24_CSN_Select(nRF24Handle* pDev){
	HAL_GPIO_WritePin(pDev->CSN_GPIO_Port, pDev->CSN_Pin, 0);
}

// Send a high signal to the chip select line
void _NRF24_CSN_Unselect(nRF24Handle* pDev){
	HAL_GPIO_WritePin(pDev->CSN_GPIO_Port, pDev->CSN_Pin, 1);
}

// Send a high signal to the chip enable line
void _NRF24_CE_Enable(nRF24Handle* pDev){
	HAL_GPIO_WritePin(pDev->CSN_GPIO_Port, pDev->CE_Pin, 1);
}

// Send a low signal to the chip enable line
void _NRF24_CE_Disable(nRF24Handle* pDev){
	HAL_GPIO_WritePin(pDev->CSN_GPIO_Port, pDev->CE_Pin, 0);
}

// Write a single byte to the register
void _NRF24_WriteReg(nRF24Handle* pDev, uint8_t reg, uint8_t data){
	// Select chip using chip select (Active low)
	_NRF24_CSN_Select(pDev);

	// Send the write command to the module
	_NRF24_SendCmd(pDev, NRF24_CMD_W_REGISTER(reg));

	// Send the write command to the RF24
	HAL_SPI_Transmit(pDev->SPI, &data, 1, 1000);

	// Unselect the module again
	_NRF24_CSN_Unselect(pDev);
}

// Write multiple bytes to the register
void _NRF24_WriteRegMultiByte(nRF24Handle* pDev, uint8_t reg, uint8_t *pData, uint8_t len){
	// Select chip using chip select (Active low)
	_NRF24_CSN_Select(pDev);

	// Send the write command to the module
	_NRF24_SendCmd(pDev, NRF24_CMD_W_REGISTER(reg));

	// Send the data itself
	HAL_SPI_Transmit(pDev->SPI, pData, len, 1000);

	// Unselect the module again
	_NRF24_CSN_Unselect(pDev);
}

// Read a single byte from the register
uint8_t _NRF24_ReadReg(nRF24Handle* pDev, uint8_t reg){
	// Select chip using chip select (Active low)
	_NRF24_CSN_Select(pDev);

	// Send the read command to the RF24
	_NRF24_SendCmd(pDev, NRF24_CMD_R_REGISTER(reg));

	// Wait for the data to be received and store it
	uint8_t rxBuf;
	HAL_SPI_Receive(pDev->SPI, &rxBuf, 1, 1000);

	// Unselect the module again
	_NRF24_CSN_Unselect(pDev);

	return rxBuf;
}

// Read multiple bytes from the register
void _NRF24_ReadRegMultiByte(nRF24Handle* pDev, uint8_t reg, uint8_t *pData, uint8_t len){
	// Select chip using chip select (Active low)
	_NRF24_CSN_Select(pDev);

	// Send the read command to the RF24
	_NRF24_SendCmd(pDev, NRF24_CMD_R_REGISTER(reg));

	// Wait for the data to be received and store it
	HAL_SPI_Receive(pDev->SPI, pData, len, 1000);

	// Unselect the module again
	_NRF24_CSN_Unselect(pDev);
}

// Send a single command
// For commands, see pg. 48
void _NRF24_SendCmd(nRF24Handle* pDev, uint8_t cmd){
	HAL_SPI_Transmit(pDev->SPI, &cmd, 1, 100);
}

// Issue several NOP commands to make a short delay before reading the Status register
void _NRF24_Delay(nRF24Handle* pDev){
	for(int i = 0; i < 2; i++){
		_NRF24_SendCmd(pDev, NRF24_CMD_NOP);
	}
}

// Initialize all of the TX and RX registers to their default values
void _NRF24_InitAddresses(nRF24Handle* pDev){
	// Initialize multi-byte address registers
	uint8_t addrRXP0Default[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t addrRXP1Default[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addrTXDefault[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_RX_ADDR_P0, addrRXP0Default, 5);
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_RX_ADDR_P1, addrRXP1Default, 5);
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_TX_ADDR, addrTXDefault, 5);

	// Initialize single-byte address registers
	uint8_t addrRXP2Default = 0xC3;
	uint8_t addrRXP3Default = 0xC4;
	uint8_t addrRXP4Default = 0xC5;
	uint8_t addrRXP5Default = 0xC6;
	_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P2, addrRXP2Default);
	_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P3, addrRXP3Default);
	_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P4, addrRXP4Default);
	_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P5, addrRXP5Default);

	if(DEBUG_PRINT){
		printf("******* After Initializing addresses *********\n\r");
		NRF24_PrintDebugInfo(pDev);
	}
}

// For whatever reason, the first write to the NRF24 usually doesn't work
// By writing a 0x0 to the config, which shouldn't work, I hope to get this
// initial invalid write out of the way to begin with
void _NRF24_FirstWrite(nRF24Handle* pDev){
	// Write 0 to some register. If this is actually the first write, it shouldn't work
	_NRF24_WriteReg(pDev, NRF24_REG_FEATURE, 0x00);
}

// Read the payload from the RX FIFO
void _NRF24_ReadPayload(nRF24Handle* pDev, uint8_t *pData){
	// Select the chip
	_NRF24_CSN_Select(pDev);

	// Send the command to dump the payload
	_NRF24_SendCmd(pDev, NRF24_CMD_R_RX_PAYLOAD);

	// Received the payload and store it, delaying for up to 1000 seconds
	HAL_SPI_Receive(pDev->SPI, pData, 32, 1000);

	// Unselect the chip
	_NRF24_CSN_Unselect(pDev);
}

// Set a bit in a single-byte register
void _NRF24_SetRegBit(nRF24Handle* pDev, NRF24_REG reg, NRF24_BIT iBit){
	// Read from the the specified register
	uint8_t regVal = _NRF24_ReadReg(pDev, reg);

	// Set the bit
	regVal |= iBit;

	// Write the val back to the register
	_NRF24_WriteReg(pDev, reg, regVal);
}

// Unset a bit in a single-byte register
void _NRF24_ClearRegBit(nRF24Handle* pDev, NRF24_REG reg, NRF24_BIT iBit){
	// Read from the the specified register
	uint8_t regVal = _NRF24_ReadReg(pDev, reg);

	// Clear the bit
	regVal &= ~(iBit);

	// Write the val back to the register
	_NRF24_WriteReg(pDev, reg, regVal);
}

/********** API FUNCTIONS **************/


void NRF24_Init(nRF24Handle* pDev){

	// Issue the first write so that any following writes will actually work
	_NRF24_FirstWrite(pDev);

	// Reset the configuration register entirely
	_NRF24_WriteReg(pDev, NRF24_REG_CONFIG, 0x00);

	// Disable Enhance Shockburst Auto Acknowledge
	_NRF24_WriteReg(pDev, NRF24_REG_EN_AA, 0x00);

	// Disable RX for all pipes to begin with
	_NRF24_WriteReg(pDev, NRF24_REG_EN_RXADDR, 0x00);

	// Set the first two bits of the address width register to set the address width to five bytes
	_NRF24_WriteReg(pDev, NRF24_REG_SETUP_AW, NRF24_BITS_SETUP_AW__5BYTES);

	// Set no retransmissions, for now
	_NRF24_WriteReg(pDev, NRF24_REG_SETUP_RETR, 0x00);

	// Set the channel to 2.4GHz
	_NRF24_WriteReg(pDev, NRF24_REG_RF_CH, NRF24_BITS_RF_CH__RF_CH(0));

	// Set the RF Setup for minimum PA and 250kbps
	_NRF24_WriteReg(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR_n18db | NRF24_BITS_RF_SETUP__RF_DR_250KBPS);

	// Flush the FIFOs
	_NRF24_SendCmd(pDev, NRF24_CMD_FLUSH_RX);
	_NRF24_SendCmd(pDev, NRF24_CMD_FLUSH_TX);

	// Delay for a moment to let the FIFO Status register refresh
	_NRF24_Delay(pDev);

	// Clear any and all bits in the status register (Note: we clear these bits by writing 1)
	_NRF24_WriteReg(pDev, NRF24_REG_STATUS, NRF24_BITS_STATUS__RX_DR | NRF24_BITS_STATUS__TX_DS | NRF24_BITS_STATUS__MAX_RT);

	// Reset the RX and TX addresses to default
	_NRF24_InitAddresses(pDev);

	// Clear the DYNPD register
	_NRF24_WriteReg(pDev, NRF24_REG_DYNPD, 0x00);

	// Clear the feature register
	_NRF24_WriteReg(pDev, NRF24_REG_FEATURE, 0x00);

	// Debugging purposes
	if(DEBUG_PRINT){
		printf("************ Immediately after initialization ************\n\r");

		NRF24_PrintDebugInfo(pDev);
	}
}

// Configure the module for TX mode
void NRF24_TXMode(nRF24Handle* pDev, uint8_t* pAddr, uint8_t iChannel){
	// Power down the module for a moment
	NRF24_SetPower(pDev, NRF24_POWER_OFF);

	// Set the Same address on all pipes (don't ask)
	NRF24_SetTXAddr(pDev, pAddr);
	NRF24_SetRXAddr(pDev, NRF24_PIPE0, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE1, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE2, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE3, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE4, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE5, pAddr);

	// Write the same address to the RX buffer on pipe 0
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_RX_ADDR_P0, pAddr, 5);

	// Configure the channel
	_NRF24_WriteReg(pDev, NRF24_REG_RF_CH, iChannel);

	// Clear the RX bit to go into TX mode
	_NRF24_ClearRegBit(pDev, NRF24_REG_CONFIG, NRF24_BITS_CONFIG__PRIM_RX);

	// Power the module back up
	NRF24_SetPower(pDev, NRF24_POWER_ON);

	if(DEBUG_PRINT){
		printf("Info after TX setup:\n\r");
		NRF24_PrintDebugInfo(pDev);
	}
}

// Set the RF module to go into RX Mode
// For the time-being, we're just using Pipe 1 for RX purposes
void NRF24_RXMode(nRF24Handle* pDev, uint8_t* pAddr, uint8_t iChannel){

	// Power down while configuring
	NRF24_SetPower(pDev, NRF24_POWER_OFF);

	// Setup the channel
	NRF24_SetChannel(pDev, iChannel);

	// Set the TX Address to this
	NRF24_SetTXAddr(pDev, pAddr);

	// Set pipe 1 to this address
//	NRF24_SetRXAddr(pDev, NRF24_PIPE0, pAddr);
	NRF24_SetRXAddr(pDev, NRF24_PIPE1, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE2, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE3, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE4, pAddr);
//	NRF24_SetRXAddr(pDev, NRF24_PIPE5, pAddr);

	// Enable pipe 1
//	NRF24_SetPipeStatus(pDev, NRF24_PIPE0, NRF24_PIPE_ENABLE);
	NRF24_SetPipeStatus(pDev, NRF24_PIPE1, NRF24_PIPE_ENABLE);
//	NRF24_SetPipeStatus(pDev, NRF24_PIPE2, NRF24_PIPE_ENABLE);
//	NRF24_SetPipeStatus(pDev, NRF24_PIPE3, NRF24_PIPE_ENABLE);
//	NRF24_SetPipeStatus(pDev, NRF24_PIPE4, NRF24_PIPE_ENABLE);
//	NRF24_SetPipeStatus(pDev, NRF24_PIPE5, NRF24_PIPE_ENABLE);

	// Set the payload width for pipe 1
//	NRF24_SetPayloadWidth(pDev, NRF24_PIPE0, 32);
	NRF24_SetPayloadWidth(pDev, NRF24_PIPE1, 32);
//	NRF24_SetPayloadWidth(pDev, NRF24_PIPE2, 32);
//	NRF24_SetPayloadWidth(pDev, NRF24_PIPE3, 32);
//	NRF24_SetPayloadWidth(pDev, NRF24_PIPE4, 32);
//	NRF24_SetPayloadWidth(pDev, NRF24_PIPE5, 32);

	// Set the RX bit to go into RX mode
	_NRF24_SetRegBit(pDev, NRF24_REG_CONFIG, NRF24_BITS_CONFIG__PRIM_RX);

	// Power the module back up
	NRF24_SetPower(pDev, NRF24_POWER_ON);

	if(DEBUG_PRINT){
		printf("Info after RX setup:\n\r");
		NRF24_PrintDebugInfo(pDev);
	}
}

// Transmit up to 32 bytes, must first call NRF24_TXMode
// Returns 1 if the message was transmit successfully, 0 if otherwise
uint8_t NRF24_Transmit(nRF24Handle* pDev, uint8_t* pData, uint8_t len){
	// Return value
	uint8_t iRetv = 0;

	// Select the chip
	_NRF24_CSN_Select(pDev);

	// Send the command for transmitting the TX payload
	_NRF24_SendCmd(pDev, NRF24_CMD_W_TX_PAYLOAD);

	// Send the payload to the TX FIFO
	HAL_SPI_Transmit(pDev->SPI, pData, len, 1000);

	// Delay for a moment
	_NRF24_Delay(pDev);

	// Check the status of the FIFO
	uint8_t regFIFOStatus = _NRF24_ReadReg(pDev, NRF24_REG_FIFO_STATUS);

	// Set the timeout to be 3 times the length of the message
	uint8_t iTimeout = len * 3;

	// Pull the enable pin high
	_NRF24_CE_Enable(pDev);

	// While there is still data in the TX FIFO and we haven't timed out
	while(!(regFIFOStatus & NRF24_BITS_FIFO_STATUS__TX_EMPTY) && --iTimeout > 0){

		// Delay for a moment
		_NRF24_Delay(pDev);

		// Read the FIFO status again
		regFIFOStatus = _NRF24_ReadReg(pDev, NRF24_REG_FIFO_STATUS);
	}

	// Pull the enable pin low again
	_NRF24_CE_Disable(pDev);

	// Flush the TX FIFO
	_NRF24_SendCmd(pDev, NRF24_CMD_FLUSH_TX);

	// Mark whether we successfully transmit or not
	iRetv = (iTimeout > 0);

	// Unselect the chip
	_NRF24_CSN_Unselect(pDev);

	return iRetv;
}

// Receive 32 bytes, blocking. Must first call NRF24_RXMode
// Return the pipe we receive from if it's a message, 0 if otherwise
NRF24_PIPE NRF24_Receive(nRF24Handle* pDev, uint8_t* pData){
	// Leave Standby-I mode and enter RX Mode
	_NRF24_CE_Enable(pDev);

	if(DEBUG_PRINT){
		printf("Receiving...\n\r");
	}

	// Track how many times we've looped
	uint32_t iTracker = 1;

	// Value to hold the status register
	uint8_t regStatus = _NRF24_ReadReg(pDev, NRF24_REG_STATUS);

	// Value to hold the FIFO status register
	uint8_t regFIFOStatus = _NRF24_ReadReg(pDev, NRF24_REG_FIFO_STATUS);

	// Loop until the data received flag is set or the RX EMPTY flag is cleared
	while(!(regStatus & NRF24_BITS_STATUS__RX_DR) && (regFIFOStatus & NRF24_BITS_FIFO_STATUS__RX_EMPTY)){

		// Delay for a moment
		_NRF24_Delay(pDev);

		if(DEBUG_PRINT){
			// Print debug info every once in a while
			if(iTracker % 100000 == 0){
				NRF24_PrintDebugInfo(pDev);
			}
			// Print that we're still receiving every once in a while
			else if(iTracker % 10000 == 0){
				printf("Receiving...\n\r");
			}
		}

		// Regather the Status register
		regStatus = _NRF24_ReadReg(pDev, NRF24_REG_STATUS);

		// Regather the FIFO register
		regFIFOStatus = _NRF24_ReadReg(pDev, NRF24_REG_FIFO_STATUS);

		// Increment the tracker
		iTracker++;
	}

	// Read the payload
	_NRF24_ReadPayload(pDev, pData);

	// Clear the RX Data Received flag (You must set it, to clear, it's counterintuitive)
	_NRF24_SetRegBit(pDev, NRF24_REG_STATUS, NRF24_BITS_STATUS__RX_DR);

	// Get the pipe that the current payload came from
	NRF24_PIPE iPayloadPipe = (regStatus & NRF24_BITS_STATUS__RX_P_NO) >> 1;

	// Return the pipe that we're receiving from
	return iPayloadPipe;
}

// Test whether the module is reading and writing properly
// Returns 1 if the read and write work properly, 0 if otherwise
uint8_t NRF24_Test(nRF24Handle* pDev){
	// Return value
	uint8_t bRetv = 1;

	// Test address
	char *testAddr = "nRF24";

	// Write the test address to the tx address register
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_TX_ADDR, (uint8_t*)testAddr, 5);

	// Value to hold the read
	uint8_t returnAddr[5];

	// Read back the address we just wrote to
	_NRF24_ReadRegMultiByte(pDev, NRF24_REG_TX_ADDR, returnAddr, 5);

	// Compare the expected and returned values
	for(int i = 0; i < 5; i++){
		if(testAddr[i] != returnAddr[i]){
			bRetv = 0;
			break;
		}
	}

	return bRetv;
}

// Power the chip up or down
void NRF24_SetPower(nRF24Handle* pDev, NRF24_POWER iPower){
	// Switch the power bit off or on
	switch (iPower){
		case NRF24_POWER_ON:
			_NRF24_SetRegBit(pDev, NRF24_REG_CONFIG, NRF24_BITS_CONFIG__PWR_UP);
			break;

		case NRF24_POWER_OFF:
		default:
			_NRF24_ClearRegBit(pDev, NRF24_REG_CONFIG, NRF24_BITS_CONFIG__PWR_UP);
			break;
	}
}

// Set the channel for the chip to operate on
// Must be a number from 0 - 127
void NRF24_SetChannel(nRF24Handle* pDev, uint8_t iChannel){
	// If the channel is higher than 127, set it 127
	iChannel = MIN(iChannel, 0x7F);

	// Write the channel to the appropriate register
	_NRF24_WriteReg(pDev, NRF24_REG_RF_CH, iChannel);
}

// Enable/disable a pipe
void NRF24_SetPipeStatus(nRF24Handle* pDev, NRF24_PIPE iPipe, NRF24_PIPE_STATUS iStatus){
	if(iStatus == NRF24_PIPE_ENABLE){
		_NRF24_SetRegBit(pDev, NRF24_REG_EN_RXADDR, (1 << iPipe));
	} else if (iStatus == NRF24_PIPE_DISABLE) {
		_NRF24_ClearRegBit(pDev, NRF24_REG_EN_RXADDR, (1 << iPipe));
	}
}

// Set the power amplification in dBm
void NRF24_SetPA(nRF24Handle* pDev, NRF24_PA iRFPower){
	// Clear the power bits initially
	_NRF24_ClearRegBit(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR);

	// Choose a register bit and set it
	switch(iRFPower){
		case NRF24_PA_MAX:
			_NRF24_SetRegBit(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR_0db);
			break;
		case NRF24_PA_HIGH:
			_NRF24_SetRegBit(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR_n6db);
			break;
		case NRF24_PA_LOW:
			_NRF24_SetRegBit(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR_n12db);
			break;
		case NRF24_PA_MIN:
			_NRF24_SetRegBit(pDev, NRF24_REG_RF_SETUP, NRF24_BITS_RF_SETUP__RF_PWR_n18db);
			break;
	}
}

// Set the data rate of the transmission
void NRF24_SetDataRate(nRF24Handle* pDev, NRF24_DATARATE iDataRate){
	// Grab the current status of the RF Setup register
	uint8_t regRFSetup = _NRF24_ReadReg(pDev, NRF24_REG_RF_SETUP);

	// Config found on pg. 55:
	// [RF_DR_LOW, RF_DR_HIGH]:
	// '00' - 1Mbps
	// '01' - 2Mbps
	// '10' - 250kbps
	// '11' - Reserved
	switch(iDataRate){
		case NRF24_DATARATE_250KBPS:
			regRFSetup |= NRF24_BITS_RF_SETUP__RF_DR_LOW;
			regRFSetup &= ~(NRF24_BITS_RF_SETUP__RF_DR_HIGH);
			break;
		case NRF24_DATARATE_1MBPS:
			regRFSetup &= ~(NRF24_BITS_RF_SETUP__RF_DR_LOW);
			regRFSetup &= ~(NRF24_BITS_RF_SETUP__RF_DR_HIGH);
			break;
		case NRF24_DATARATE_2MBPS:
			regRFSetup &= ~(NRF24_BITS_RF_SETUP__RF_DR_LOW);
			regRFSetup |= NRF24_BITS_RF_SETUP__RF_DR_HIGH;
			break;
	}

	// Write the config back to the register
	_NRF24_WriteReg(pDev, NRF24_REG_RF_SETUP, regRFSetup);
}

// Set the address width
void NRF24_SetAddressWidth(nRF24Handle* pDev, NRF24_ADDRESSWIDTH iAW){

	uint8_t iRegVal = 0;

	switch (iAW){
		case NRF24_ADDRESSWIDTH_3BYTES:
			iRegVal = NRF24_BITS_SETUP_AW__3BYTES;
			break;

		case NRF24_ADDRESSWIDTH_4BYTES:
			iRegVal = NRF24_BITS_SETUP_AW__4BYTES;
			break;

		case NRF24_ADDRESSWIDTH_5BYTES:
			iRegVal = NRF24_BITS_SETUP_AW__5BYTES;
			break;
	}

	_NRF24_WriteReg(pDev, NRF24_REG_SETUP_AW, iRegVal);
}

// Set the payload width at a particular pipe
// Must be an integer from 1 to 32 bytes, or 0 to disable the pipe
void NRF24_SetPayloadWidth(nRF24Handle* pDev, NRF24_PIPE iPipe, uint8_t iPW){
	// Register of the pipe we want to edit the payload width of
	uint8_t iPipeReg = 0;

	// Decide which pipe register to write to

	switch(iPipe){
		case NRF24_PIPE0:
			iPipeReg = NRF24_REG_RX_PW_P0;
			break;
		case NRF24_PIPE1:
			iPipeReg = NRF24_REG_RX_PW_P1;
			break;
		case NRF24_PIPE2:
			iPipeReg = NRF24_REG_RX_PW_P2;
			break;
		case NRF24_PIPE3:
			iPipeReg = NRF24_REG_RX_PW_P3;
			break;
		case NRF24_PIPE4:
			iPipeReg = NRF24_REG_RX_PW_P4;
			break;
		case NRF24_PIPE5:
			iPipeReg = NRF24_REG_RX_PW_P5;
			break;
	}

	// Write the value to the register
	_NRF24_WriteReg(pDev, iPipeReg, iPW);
}

// Get the address width, in bytes
// Return: 3, 4, or 5
uint8_t NRF24_GetAddressWidth(nRF24Handle* pDev){

	// Get the value stores in the register
	uint8_t iRegVal = _NRF24_ReadReg(pDev, NRF24_REG_SETUP_AW);

	// Value signififying the number of bytes in an address
	uint8_t iRetVal = 0;

	switch (iRegVal){
		case NRF24_BITS_SETUP_AW__3BYTES:
			iRetVal = 3;
			break;

		case NRF24_BITS_SETUP_AW__4BYTES:
			iRetVal = 4;
			break;

		case NRF24_BITS_SETUP_AW__5BYTES:
			iRetVal = 5;
			break;
	}

	return iRetVal;
}

// Set the address of one of the RX pipes
// NOTE: The 4 MSB for pipes 1-5 must be identical, and are stored in RX_ADDR_P1 39:8
void NRF24_SetRXAddr(nRF24Handle* pDev, NRF24_PIPE iPipe, uint8_t* pAddr){

	// Retrieve the current address width
	uint8_t iAddressWidth = NRF24_GetAddressWidth(pDev);

	switch (iPipe){
		case NRF24_PIPE0:
			_NRF24_WriteRegMultiByte(pDev, NRF24_REG_RX_ADDR_P0, pAddr, iAddressWidth);
			break;
		case NRF24_PIPE1:
			_NRF24_WriteRegMultiByte(pDev, NRF24_REG_RX_ADDR_P1, pAddr, iAddressWidth);
			break;

		// If the address is any of the following, the LSB is stored in the corresponding register
		// And the MSBs are stores in P1
		case NRF24_PIPE2:
			_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P2, *pAddr);
			break;

		case NRF24_PIPE3:
			_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P3, *pAddr);
			break;

		case NRF24_PIPE4:
			_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P4, *pAddr);
			break;

		case NRF24_PIPE5:
			_NRF24_WriteReg(pDev, NRF24_REG_RX_ADDR_P5, *pAddr);
			break;
	}
}

// Set the TX Address
// NOTE: The 4 MSB for pipes 1-5 must be identical, and are stored in RX_ADDR_P1 39:8
void NRF24_SetTXAddr(nRF24Handle* pDev, uint8_t* pAddr){

	// Retrieve the current address width
	uint8_t iAddressWidth = NRF24_GetAddressWidth(pDev);

	// Write the address to the TX_ADDR register
	_NRF24_WriteRegMultiByte(pDev, NRF24_REG_TX_ADDR, pAddr, iAddressWidth);

}

// Set the ability to auto acknowledge
void NRF24_SetAutoAck(nRF24Handle* pDev, NRF24_PIPE iPipe, NRF24_AA_STATUS iStatus){

	// Read the current values in the register
	uint8_t regEnAA = _NRF24_ReadReg(pDev, NRF24_REG_EN_AA);

	// Holds the mask we'll use to manipulate the register
	uint8_t iMask = 0;

	// Check which pipe we want to enable AA on
	switch(iPipe){
		case NRF24_PIPE0:
			iMask = NRF24_BITS_EN_AA__ENAA_P0;
			break;

		case NRF24_PIPE1:
			iMask = NRF24_BITS_EN_AA__ENAA_P1;
			break;

		case NRF24_PIPE2:
			iMask = NRF24_BITS_EN_AA__ENAA_P2;
			break;

		case NRF24_PIPE3:
			iMask = NRF24_BITS_EN_AA__ENAA_P3;
			break;

		case NRF24_PIPE4:
			iMask = NRF24_BITS_EN_AA__ENAA_P4;
			break;

		case NRF24_PIPE5:
			iMask = NRF24_BITS_EN_AA__ENAA_P5;
			break;
	}

	// Set or reset the appropriate bit
	if(iStatus == NRF24_AA_ENABLE){
		regEnAA |= iMask;
	} else if (iStatus == NRF24_AA_DISABLE){
		regEnAA &= ~(iMask);
	}

	// Write the new register value back
	_NRF24_WriteReg(pDev, NRF24_REG_EN_AA, regEnAA);
}

// Set the ability to auto acknowledge for all or none of the pipes
void NRF24_SetAutoAckAllPipes(nRF24Handle* pDev, NRF24_AA_STATUS iStatus){

	uint8_t iReg = 0x00;

	if(iStatus == NRF24_AA_ENABLE){
		_NRF24_WriteReg(pDev, NRF24_REG_EN_AA, iReg | NRF24_BITS_EN_AA);
	} else if (iStatus == NRF24_AA_DISABLE){
		_NRF24_WriteReg(pDev, NRF24_REG_EN_AA, iReg & ~(NRF24_BITS_EN_AA));
	}
}

// Print info from all registers to the console
void NRF24_PrintDebugInfo(nRF24Handle* pDev){
	// Select the chip
	_NRF24_CSN_Select(pDev);

	// Get values at various registers
	uint8_t regConfig 		= _NRF24_ReadReg(pDev, NRF24_REG_CONFIG);
	uint8_t regEnAA			= _NRF24_ReadReg(pDev, NRF24_REG_EN_AA);
	uint8_t regEnRxAddr 	= _NRF24_ReadReg(pDev, NRF24_REG_EN_RXADDR);
	uint8_t regSetupAW 		= _NRF24_ReadReg(pDev, NRF24_REG_SETUP_AW);
	uint8_t regSetupRetr 	= _NRF24_ReadReg(pDev, NRF24_REG_SETUP_RETR);
	uint8_t regRFCH 		= _NRF24_ReadReg(pDev, NRF24_REG_RF_CH);
	uint8_t regRFSetup 		= _NRF24_ReadReg(pDev, NRF24_REG_RF_SETUP);
	uint8_t regStatus 		= _NRF24_ReadReg(pDev, NRF24_REG_STATUS);
	uint8_t regObserveTX 	= _NRF24_ReadReg(pDev, NRF24_REG_OBSERVE_TX);
	uint8_t regRPD			= _NRF24_ReadReg(pDev, NRF24_REG_RPD);
	uint8_t regRxAddrP2 	= _NRF24_ReadReg(pDev, NRF24_REG_RX_ADDR_P2);
	uint8_t regRxAddrP3 	= _NRF24_ReadReg(pDev, NRF24_REG_RX_ADDR_P3);
	uint8_t regRxAddrP4 	= _NRF24_ReadReg(pDev, NRF24_REG_RX_ADDR_P4);
	uint8_t regRxAddrP5 	= _NRF24_ReadReg(pDev, NRF24_REG_RX_ADDR_P5);
	uint8_t regRxPwP0		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P0);
	uint8_t regRxPwP1		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P1);
	uint8_t regRxPwP2		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P2);
	uint8_t regRxPwP3		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P3);
	uint8_t regRxPwP4		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P4);
	uint8_t regRxPwP5		= _NRF24_ReadReg(pDev, NRF24_REG_RX_PW_P5);
	uint8_t regFIFOStatus 	= _NRF24_ReadReg(pDev, NRF24_REG_FIFO_STATUS);
	uint8_t regDynPd		= _NRF24_ReadReg(pDev, NRF24_REG_DYNPD);
	uint8_t regFeature		= _NRF24_ReadReg(pDev, NRF24_REG_FEATURE);

	// Get values from multi-byte registers
	uint8_t regRxAddrP0[5];
	_NRF24_ReadRegMultiByte(pDev, NRF24_REG_RX_ADDR_P0, regRxAddrP0, 5);

	uint8_t regRxAddrP1[5];
	_NRF24_ReadRegMultiByte(pDev, NRF24_REG_RX_ADDR_P1, regRxAddrP1, 5);

	uint8_t regTxAddr[5];
	_NRF24_ReadRegMultiByte(pDev, NRF24_REG_TX_ADDR, regTxAddr, 5);

	// We don't need to access the chip anymore so we'll unselect it
	_NRF24_CSN_Unselect(pDev);

	// Config values
	uint8_t bConfig_MASK_RX_DR 	= (regConfig & NRF24_BITS_CONFIG__MASK_RX_DR) != 0;		// Enable/Disable RX Ready interrupt
	uint8_t bConfig_MASK_TX_DS 	= (regConfig & NRF24_BITS_CONFIG__MASK_TX_DS) != 0;		// Enable/Disable TX Data sent interrupt
	uint8_t bConfig_MASK_MAX_RT	= (regConfig & NRF24_BITS_CONFIG__MASK_MAX_RT) != 0;	// Enable/Disable Max Retransmit interrupt
	uint8_t bConfig_EN_CRC		= (regConfig & NRF24_BITS_CONFIG__EN_CRC) != 0;			// Enable/Disable cyclic redundancy check
	uint8_t bConfig_CRCO		= (regConfig & NRF24_BITS_CONFIG__CRCO) != 0;			// The number of bytes in the CRC encoding scheme
	uint8_t bConfig_PWR_UP		= (regConfig & NRF24_BITS_CONFIG__PWR_UP) != 0;			// POWER UP/POWER DOWN
	uint8_t bConfig_PRIM_RX		= (regConfig & NRF24_BITS_CONFIG__PRIM_RX) != 0;		// RX/TX Control

	// Auto acknowledge values
	uint8_t bEnAutoAck_pipe5	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P5) != 0;
	uint8_t bEnAutoAck_pipe4	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P4) != 0;
	uint8_t bEnAutoAck_pipe3	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P3) != 0;
	uint8_t bEnAutoAck_pipe2	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P2) != 0;
	uint8_t bEnAutoAck_pipe1	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P1) != 0;
	uint8_t bEnAutoAck_pipe0	= (regEnAA & NRF24_BITS_EN_AA__ENAA_P0) != 0;

	// Pipes currently enabled
	uint8_t bEnRxAddr_pipe0		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P0) != 0;
	uint8_t bEnRxAddr_pipe1		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P1) != 0;
	uint8_t bEnRxAddr_pipe2		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P2) != 0;
	uint8_t bEnRxAddr_pipe3		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P3) != 0;
	uint8_t bEnRxAddr_pipe4		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P4) != 0;
	uint8_t bEnRxAddr_pipe5		= (regEnRxAddr & NRF24_BITS_EN_RXADDR__ERX_P5) != 0;


	// Address width
	uint8_t iAddressWidth;
	switch(regSetupAW){
		case NRF24_BITS_SETUP_AW__3BYTES:
			iAddressWidth = 3;
			break;
		case NRF24_BITS_SETUP_AW__4BYTES:
			iAddressWidth = 4;
			break;
		case NRF24_BITS_SETUP_AW__5BYTES:
			iAddressWidth = 5;
			break;
		default:
			iAddressWidth = 0;
			break;
	}

	// Retransmission delay = val * 250 nanoseconds
	uint8_t iRetransDelayBits = (regSetupRetr >> 4) & 0xF;
	uint16_t iRetransDelayUS = (iRetransDelayBits + 1) * 250;

	// Auto retransmit max count
	uint8_t iRetransMaxCount = regSetupRetr & 0xF;

	// Frequency channel. Frequency = 2.4GHz + freqChannel Mhz
	uint8_t  iFreqChannel 	= regRFCH;
	uint16_t iFreqMHz 		= 2400 + iFreqChannel;

	// RF Setup values
	uint8_t bRFSetup_ContWave		= (regRFSetup & NRF24_BITS_RF_SETUP__CONT_WAVE) != 0; 	// Enable/Disable continuous carrier transmit
	uint8_t bRFSetup_PLLLock		= (regRFSetup & NRF24_BITS_RF_SETUP__PLL_LOCK) != 0;	// Enable/Disable PLL Lock
	uint8_t iRFSetup_DataRateLow	= (regRFSetup & NRF24_BITS_RF_SETUP__RF_DR_LOW) != 0;	// First bit for the data rate
	uint8_t iRFSetup_DataRateHigh 	= (regRFSetup & NRF24_BITS_RF_SETUP__RF_DR_HIGH)!= 0;	// Second bit for the data rate
	uint8_t iRFSetup_RFPWR			= (regRFSetup & NRF24_BITS_RF_SETUP__RF_PWR) >> 1;

	// Calculate the data rate
	uint16_t iDataRatekbps = 0;
	if (iRFSetup_DataRateLow == 1){
		iDataRatekbps = 250;
	} else if (iRFSetup_DataRateHigh == 0){
		iDataRatekbps = 1000;
	} else if (iRFSetup_DataRateHigh == 1){
		iDataRatekbps = 2000;
	}

	// Values from the status register
	uint8_t bStatusRXDR 	= (regStatus & NRF24_BITS_STATUS__RX_DR) != 0;
	uint8_t bStatusTXDS 	= (regStatus & NRF24_BITS_STATUS__TX_DS) != 0;
	uint8_t bStatusMAXRT 	= (regStatus & NRF24_BITS_STATUS__MAX_RT) != 0;
	uint8_t iStatusRXPipeNo = (regStatus >> 1) & NRF24_BITS_STATUS__RX_P_NO;
	uint8_t bStatusTXFull 	= (regStatus & NRF24_BITS_STATUS__TX_FULL) != 0;

	// Observe TX register values
	uint8_t iObserveTxLost 		= (regObserveTX >> 4) & 0xF;
	uint8_t iObserveTxRetrans	= (regObserveTX >> 0) & 0xF;

	// RPD register values
	uint8_t bRPD = regRPD;

	// Calculate the power amplification
	int8_t iRFSetup_RFPWRdBs = 0;
	switch (iRFSetup_RFPWR << 1){
		case NRF24_BITS_RF_SETUP__RF_PWR_n18db:
			iRFSetup_RFPWRdBs = -18;
			break;
		case NRF24_BITS_RF_SETUP__RF_PWR_n12db:
			iRFSetup_RFPWRdBs = -12;
			break;
		case NRF24_BITS_RF_SETUP__RF_PWR_n6db:
			iRFSetup_RFPWRdBs = -6;
			break;
		case NRF24_BITS_RF_SETUP__RF_PWR_0db:
			iRFSetup_RFPWRdBs = 0;
			break;
	}

	// Number of bytes in payload
	uint8_t iRxPwP0 = regRxPwP0;
	uint8_t iRxPwP1 = regRxPwP1;
	uint8_t iRxPwP2 = regRxPwP2;
	uint8_t iRxPwP3 = regRxPwP3;
	uint8_t iRxPwP4 = regRxPwP4;
	uint8_t iRxPwP5 = regRxPwP5;

	// FIFO Status register values
	uint8_t bFIFOStatusTXReuse 	= (regFIFOStatus & NRF24_BITS_FIFO_STATUS__TX_REUSE) != 0;
	uint8_t bFIFOStatusTXFull 	= (regFIFOStatus & NRF24_BITS_FIFO_STATUS__TX_FULL) != 0;
	uint8_t bFIFOStatusTXEmpty 	= (regFIFOStatus & NRF24_BITS_FIFO_STATUS__TX_EMPTY) != 0;
	uint8_t bFIFOStatusRXFull 	= (regFIFOStatus & NRF24_BITS_FIFO_STATUS__RX_FULL) != 0;
	uint8_t bFIFOStatusRXEmpty 	= (regFIFOStatus & NRF24_BITS_FIFO_STATUS__RX_EMPTY) != 0;

	// Dynamic payload length
	uint8_t bDPLP5 = (regDynPd & NRF24_BITS_DYNPD__DPL_P5) != 0;
	uint8_t bDPLP4 = (regDynPd & NRF24_BITS_DYNPD__DPL_P4) != 0;
	uint8_t bDPLP3 = (regDynPd & NRF24_BITS_DYNPD__DPL_P3) != 0;
	uint8_t bDPLP2 = (regDynPd & NRF24_BITS_DYNPD__DPL_P2) != 0;
	uint8_t bDPLP1 = (regDynPd & NRF24_BITS_DYNPD__DPL_P1) != 0;
	uint8_t bDPLP0 = (regDynPd & NRF24_BITS_DYNPD__DPL_P0) != 0;

	// Feature register values
	uint8_t bFeature_ENDPL 	= (regFeature & NRF24_BITS_FEATURE__EN_DPL) != 0;
	uint8_t bFeature_ACKPAY = (regFeature & NRF24_BITS_FEATURE__EN_ACK_PAY) != 0;
	uint8_t bFeature_DYNACK = (regFeature & NRF24_BITS_FEATURE__EN_DYN_ACK) != 0;

	// Print out all the relevant info
	printf("******* DEBUG INFO *******\n\r");
	printf("- CONFIG register:\n\r");
	printf("\t- MASK_RX_DR: %d\n\r", 	bConfig_MASK_RX_DR);
	printf("\t- MASK_TX_DS: %d\n\r", 	bConfig_MASK_TX_DS);
	printf("\t- MASK_MAX_RT: %d\n\r", 	bConfig_MASK_MAX_RT);
	printf("\t- EN_CRC: %d\n\r", 		bConfig_EN_CRC);
	printf("\t- CRCO: %d\n\r", 			bConfig_CRCO);
	printf("\t- PWR_UP: %d\n\r", 		bConfig_PWR_UP);
	printf("\t- PRIM_RX: %d\n\r", 		bConfig_PRIM_RX);

	printf("- EN_AA register:\n\r");
	printf("\t- ENAA_P5: %d\n\r", bEnAutoAck_pipe5);
	printf("\t- ENAA_P4: %d\n\r", bEnAutoAck_pipe4);
	printf("\t- ENAA_P3: %d\n\r", bEnAutoAck_pipe3);
	printf("\t- ENAA_P2: %d\n\r", bEnAutoAck_pipe2);
	printf("\t- ENAA_P1: %d\n\r", bEnAutoAck_pipe1);
	printf("\t- ENAA_P0: %d\n\r", bEnAutoAck_pipe0);

	printf("- EN_RXADDR register:\n\r");
	printf("\t- ERX_P0: %d\n\r", 	bEnRxAddr_pipe0);
	printf("\t- ERX_P1: %d\n\r", 	bEnRxAddr_pipe1);
	printf("\t- ERX_P2: %d\n\r", 	bEnRxAddr_pipe2);
	printf("\t- ERX_P3: %d\n\r", 	bEnRxAddr_pipe3);
	printf("\t- ERX_P4: %d\n\r", 	bEnRxAddr_pipe4);
	printf("\t- ERX_P5: %d\n\r", 	bEnRxAddr_pipe5);

	printf("- SETUP_AW register: \n\r");
	printf("\t- Address width: %d bytes\n\r", iAddressWidth);

	printf("- SETUP_RETR register:\n\r");
	printf("\t- Automatic retransmission delay (ARD): 0x%x = %duS delay\n\r", iRetransDelayBits, iRetransDelayUS);
	printf("\t- Automatic retransmission count (ARC): %d max count\n\r", iRetransMaxCount);

	printf("- RF_CH register: \n\r");
	printf("\t- Channel No: %d, Frequency: %dMHz\n\r", iFreqChannel, iFreqMHz);

	printf("- RF_SETUP register: \n\r");
	printf("\t- CONT_WAVE: %d\n\r", bRFSetup_ContWave);
	printf("\t- PLL_LOCK: %d\n\r", bRFSetup_PLLLock);
	printf("\t- RF_DR_LOW: %d, RF_DR_HIGH: %d, data rate: %dkbps\n\r", iRFSetup_DataRateLow, iRFSetup_DataRateHigh, iDataRatekbps);
	printf("\t- RF_PWR: %d, amplification power: %ddBm\n\r", iRFSetup_RFPWR, iRFSetup_RFPWRdBs);

	printf("- STATUS register:\n\r");
	printf("\t- RX_DR: %d\n\r", bStatusRXDR);
	printf("\t- TX_DS: %d\n\r", bStatusTXDS);
	printf("\t- MAX_RT: %d\n\r", bStatusMAXRT);
	printf("\t- RX_P_NO: %d", iStatusRXPipeNo); printf(iStatusRXPipeNo == 6 ? " (Not used)\n\r" : "\n\r");
	printf("\t- TX_FULL: %d\n\r", bStatusTXFull);

	printf("- OBSERVE_TX register:\n\r");
	printf("\t- PLOS_CNT: %d\n\r", iObserveTxLost);
	printf("\t- ARC_CNT: %d\n\r", iObserveTxRetrans);

	printf("- RPD register:\n\r");
	printf("\t- RPD: %d\n\r", bRPD);

	printf("- RX_ADDR_PX registers:\n\r");
	printf("\t- RX_ADDR_P0: 0x%02x%02x%02x%02x%02x\n\r", regRxAddrP0[4], regRxAddrP0[3], regRxAddrP0[2], regRxAddrP0[1], regRxAddrP0[0]);
	printf("\t- RX_ADDR_P1: 0x%02x%02x%02x%02x%02x\n\r", regRxAddrP1[4], regRxAddrP1[3], regRxAddrP1[2], regRxAddrP1[1], regRxAddrP1[0]);
	printf("\t- RX_ADDR_P2: 0x%02x\n\r", regRxAddrP2);
	printf("\t- RX_ADDR_P3: 0x%02x\n\r", regRxAddrP3);
	printf("\t- RX_ADDR_P4: 0x%02x\n\r", regRxAddrP4);
	printf("\t- RX_ADDR_P5: 0x%02x\n\r", regRxAddrP5);

	printf("- TX_ADDR register:\n\r");
	printf("\t- TX_ADDR: 0x%02x%02x%02x%02x%02x\n\r", regTxAddr[4], regTxAddr[3], regTxAddr[2], regTxAddr[1], regTxAddr[0]);

	printf("- RX_PW_PX registers:\n\r");
	printf("\t- RX_PW_P0: %d\n\r", iRxPwP0);
	printf("\t- RX_PW_P1: %d\n\r", iRxPwP1);
	printf("\t- RX_PW_P2: %d\n\r", iRxPwP2);
	printf("\t- RX_PW_P3: %d\n\r", iRxPwP3);
	printf("\t- RX_PW_P4: %d\n\r", iRxPwP4);
	printf("\t- RX_PW_P5: %d\n\r", iRxPwP5);

	printf("- FIFO Status register:\n\r");
	printf("\t- TX_REUSE: %d\n\r", bFIFOStatusTXReuse);
	printf("\t- TX_FULL: %d\n\r", bFIFOStatusTXFull);
	printf("\t- TX_EMPTY: %d\n\r", bFIFOStatusTXEmpty);
	printf("\t- RX_FULL: %d\n\r", bFIFOStatusRXFull);
	printf("\t- RX_EMPTY: %d\n\r", bFIFOStatusRXEmpty);

	printf("- DYNPD register:\n\r");
	printf("\t- DPL_P5: %d\n\r", bDPLP5);
	printf("\t- DPL_P4: %d\n\r", bDPLP4);
	printf("\t- DPL_P3: %d\n\r", bDPLP3);
	printf("\t- DPL_P2: %d\n\r", bDPLP2);
	printf("\t- DPL_P1: %d\n\r", bDPLP1);
	printf("\t- DPL_P0: %d\n\r", bDPLP0);

	printf("- FEATURE register:\n\r");
	printf("\t- EN_DPL: %d\n\r", bFeature_ENDPL);
	printf("\t- ACKPAY: %d\n\r", bFeature_ACKPAY);
	printf("\t- DYNACK: %d\n\r", bFeature_DYNACK);

	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
	printf("\n\r");
}
