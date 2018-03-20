/*
 * File:		modbus.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		28. 1. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
MODBUS functions
SmartMeter function
 */

#include "edge.h"

static uint16_t MBCalcCRC(uint8_t *buf, int n);

extern volatile bool rx6_EOF_rcv;
static uint8_t mbfunc0, mbreg0, mbcount0; 	// MODBUS: remember from a query
static uint8_t mbaddr = 3; 	// MODBUS slave address, now for Smartmmeter
uint8_t mbdata[MB_DATA_SIZE], mberrslave = 0;

//------------------------------------------------------------------------------
int SMReadVal(uint16_t reg, float* value)
	{
	int n, rtv = 0;
	n = MBSendQuery(MB_METER_ADDR, MB_FUNC_READ, reg, 2, NULL);
	HAL_Delay(150);   	// at least 110 ms
	rtv = -1;
	if (rx6_EOF_rcv)	// RS485	
			{
			rx6_EOF_rcv = false;
			rtv = MBProcResponse(mbdata, MB_DATA_SIZE);
			if (rtv == 2)
				{
				n = __REV(*(uint32_t*)mbdata);       	// Endian swap
				*value = *((float*)&n);
				}
			}
	return rtv;
	}	

//------------------------------------------------------------------------------
int MBSendQuery(uint8_t addr, uint8_t func, uint16_t reg, uint8_t rcount, uint8_t *data)
	{
	uint16_t crc;
	uint8_t buf[42];
	int i = 0, j;

	if (rcount > 16)	rcount = 16;
	buf[i++] = addr;
	buf[i++] = func;
	switch (func)
		{
		case 0x10:		// register(s) write
			buf[i++] = BHI(reg);
			buf[i++] = BLO(reg);
			buf[i++] = 0; 	// Register Count MSB always 0
			buf[i++] = rcount;
			buf[i++] = 2 * rcount; 	// byte count
			for(j = 0 ; j < 2*rcount ; j++)
				buf[i++] = *data++;
			break;
		case 0x03:			// Holding Register(s) read
			buf[i++] = BHI(reg);
			buf[i++] = BLO(reg);
			buf[i++] = 0; 	// Register Count MSB always 0
			buf[i++] = rcount;
			break;
		}
	crc = MBCalcCRC(buf, i);
	buf[i++] = BLO(crc);
	buf[i++] = BHI(crc);
	mbfunc0 = func;
	mbreg0 = reg;
	mbcount0 = rcount;
	RS485ReadFlush();
	RS485SendBuf(buf, i);
	return i;
	}

//------------------------------------------------------------------------------
int MBProcResponse(uint8_t *data, int nbuf)
	{
	uint8_t buf[38];
	int i, nrcv, rtv = 0;
	uint8_t func, count;

	nrcv = RS485ReadBuf(buf, sizeof(buf));
	if (nrcv > 4)
		{
		if (MBCalcCRC(buf, nrcv) != 0)
			rtv = -3;  		// CRC Error
		else
			{	
			func = buf[1];   		// returned function number
			if(nrcv == 5 && func == (mbfunc0 + 0x80))
				{
				mberrslave = buf[2];  		// error code from slave
				rtv = -2;
				}					
			else
				{
				switch (func)
					{
					case MB_FUNC_WRITE:
						rtv = 1;
						break;
					case MB_FUNC_READ:
						if (nrcv == (2 * mbcount0 + 5))
							{
							count = buf[2];
							for (i = 0; i < count; i++)
								{
								if (i == nbuf)
									return -6;					// IF attempt to write behind data buffer
								*data++ = buf[i + 3];
								}
							rtv = 2;						
							}
						else
							rtv = -5;   		// error of the response format e.g. wrong byte number
		break;
					}
				}
			}
		}
	else
		rtv = -4;  	// short of bytes
		return rtv;
	}

//------------------------------------------------------------------------------
#define SEED 0xFFFF  // initialization for CRC16
#define GP   0xA001  // generating polynomial
uint16_t MBCalcCRC(uint8_t *buf, int n)
	{
	uint16_t crc;
	bool carry;
	int i;

	crc = SEED;
	while (n--)
		{
		crc ^= *buf++;
		for (i = 0; i < 8; i++)
			{
			carry = (bool)(crc & 0x0001);
			crc >>= 1;
			if (carry) crc ^= GP;
			}
		}
	return crc;
	}


