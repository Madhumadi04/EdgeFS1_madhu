/*
 * File:		iolink.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		1. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
IO-Link IC functions
IO-Link protocol functions
 */

#include "edge.h"

extern bool buzzer;
// Global variables
uint8_t iolctrl2;

// IO-Link IC Functions
//------------------------------------------------------------------------------
uint8_t IOL_ReadReg(uint8_t regno)
	{
	uint16_t rtv, param;
	param = ((uint16_t)regno) << 9;
	rtv = SPI2_SendWord(param);
	return (uint8_t)rtv;
	}

//------------------------------------------------------------------------------
uint8_t IOL_WrUpdReg(uint8_t regno, uint8_t data)
	{
	uint16_t rtv, param;
	param = 0x6000 | ((uint16_t)regno) << 9 | data;
	rtv = SPI2_SendWord(param);
	return (uint8_t)rtv;
	}

//------------------------------------------------------------------------------
int IOL_Init(void)
	{
	if (IOL_ReadReg(0x9) != 0xF4)    // MODE2, test of IC connection
		return - 1;	
	IOL_WrUpdReg(0xA, 0x55);    	// NSF, 20us
	iolctrl2 = 0x0F;
	IOL_WrUpdReg(0xE, iolctrl2);   // CTRL2: SIO mode, OC timeout 480 us
	IOL_WrUpdReg(0xC, 0xE0);    	// TMRCTRL: long overcurrent
	IOL_WrUpdReg(0xD, 0x0C);    	// CTRL1: CQ3-4 enable
	IOL_WrUpdReg(0x0, 0x40);     	// IRQMASK, Supply
	if((IOL_ReadReg(0x6) & 0x20) == 0)    	// STATUS1, bit UnderVoltage
		return 1;
	else
		return 0;
	}

//------------------------------------------------------------------------------
void IOL_DigOut(uint32_t outno, uint32_t set)
	{
	uint8_t mask;	
	switch (outno)
		{
		case 0:	// TO DO: all DIGOUTs together
			break;
		case 1:	// CQ3
			switch(set)
				{
			case 0: SETPIN(DIGOUT1_GPIO_Port, DIGOUT1_Pin); break;
			case 1: CLRPIN(DIGOUT1_GPIO_Port, DIGOUT1_Pin); break;
			case 2: TGLPIN(DIGOUT1_GPIO_Port, DIGOUT1_Pin); break;
				}
			break;
		case 2:	// CQ4
			switch(set)
				{
			case 0: SETPIN(DIGOUT2_GPIO_Port, DIGOUT2_Pin); break;
			case 1: CLRPIN(DIGOUT2_GPIO_Port, DIGOUT2_Pin); break;
			case 2: TGLPIN(DIGOUT2_GPIO_Port, DIGOUT2_Pin); break;
				}
			break;
		case 3:	// L+3
			mask = 0x40;
			goto IOLWr;
		case 4:	// L+4
			mask = 0x80;
			goto IOLWr;
		case 5:	// L+1
			mask = 0x10;
			goto IOLWr;
		case 6:	// L+2
			mask = 0x20;
IOLWr: switch (set)
				{
				case 0: iolctrl2 &= ~mask; break;
				case 1: iolctrl2 |= mask; break;
				case 2: iolctrl2 ^= mask; break;
				}
			IOL_WrUpdReg(0xE, iolctrl2);
			break;
			}
	}

// Communication & Auxilliary Functions

//------------------------------------------------------------------------------
uint16_t SPI2_SendWord(uint16_t data)
	{
	CS_IOL_ACTIVE;
	while (!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE)) ;
	hspi2.Instance->DR = data;
	while (!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE)) ;
	return hspi2.Instance->DR;
	CS_IOL_IDLE;
	}

/*
//------------------------------------------------------------------------------
static uint8_t SPI2_SendByte(uint8_t data)
	{
	while (!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE)) ;
	hspi2.Instance->DR = data;
	while (!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_RXNE)) ;
	return hspi2.Instance->DR;
	}

//------------------------------------------------------------------------------
static void SPI2_ProcData(uint8_t* inbuf, int inlen, uint8_t* outbuf, int outlen)
	{
	int i;
	CS_IOL_IDLE;
	CS_IOL_ACTIVE;
	for (i = 0; i < inlen; i++)
		SPI2_SendByte(*inbuf++);
	for (i = 0; i < outlen; i++)
		*outbuf++ = SPI2_SendByte(0);
	CS_IOL_IDLE;
	}
*/
