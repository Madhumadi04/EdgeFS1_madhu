/*
 * File:		edge.h
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		23. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
Common definitions
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EDGE_H
#define __EDGE_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Common defines
#define STDIO_UART			2	// UARTx redirection, 0 for none and also for the semihosting
#define RS232_DMM					// Agilent DMM on USART2 (9600 Bd, RTS/CTS) otherwise STDIO here possible
#define MODBUS						// MODBUS mode on RS485/USART6
#define GSM_MQTT					// Quectel GSM/LTE on USART1, MQTT pusblishing/subscription
// #define COM_SPI					// SPI on COM connector for LoRa RFM95
// #define ADI_IOL_OUT				// IO-Link direct output test by DigIn levels

// Special macros
#define COUNTOF(a)   (sizeof(a) / sizeof(*(a)))

// Pointers to variable parts (Little Endian like ARM)
#define LB0(x) *((unsigned char *)&(x)+0)
#define LB1(x) *((unsigned char *)&(x)+1)
#define LB2(x) *((unsigned char *)&(x)+2)
#define LB3(x) *((unsigned char*)&(x)+3)

#define  WLO(x)  *((unsigned short int *)&(x))
#define  WHI(x)  *(((unsigned short int *)&(x))+1)
#define  BLO(x)  *((unsigned char *)&(x))
#define  BHI(x)  *(((unsigned char *)&(x))+1)
#define  No(v)  (sizeof((v))/sizeof((v)[0]))

// 1Wire
#define N_ONEWIRE_SENSORS	3
#define N_DS12B20				3

// MQTT
#define N_MQTT_OUT			10		// published
#define N_MQTT_IN				4		// subscribed
#define N_MQTT_TOPIC			(N_MQTT_OUT + N_MQTT_IN)

// MODBUS
#define MB_METER_ADDR		1	// 3 for PowerMetering on Adam's stand
#define MB_DATA_SIZE			8	// maximum 8 bytes for Smartmeter Serial Number response
#define MB_FUNC_READ			0x03
#define MB_FUNC_WRITE		0x10
#define MB_METER_REG_SN		0x1000
#define MB_METER_REG_VOLT	0x2000
#define MB_METER_REG_POWER	0x2080
#define MB_METER_REG_FREQ	0x2020
#define MB_METER_REG_CURRENT	0x2060
#define MB_METER_REG_PFACTOR	0x20E0
#define MODBUS						// MODBUS mode on RS485/USART6

// SPI
#define  CS_IOL_IDLE			SETPIN(CS_IOLINK_GPIO_Port,CS_IOLINK_Pin)
#define  CS_IOL_ACTIVE		CLRPIN(CS_IOLINK_GPIO_Port,CS_IOLINK_Pin)
#define  CS_COM_IDLE			SETPIN(CS1_COM_GPIO_Port,CS1_COM_Pin)
#define  CS_COM_ACTIVE		CLRPIN(CS1_COM_GPIO_Port,CS1_COM_Pin)

// Macros -------------------------------------------------------------
// Pin operation
#define  SETPIN(port,pin)  (port->BSRR = (pin))
#define  CLRPIN(port,pin)  (port->BSRR = ((pin) << 16))
#define  TSTPIN(port,pin)  (port->IDR & (pin))
#define  TGLPIN(port,pin)  (port->ODR ^= (pin))

#define  AUX1_ON		SETPIN(ADC1_IN8_ADI5_AUX1_GPIO_Port,ADC1_IN8_ADI5_AUX1_Pin)
#define  AUX1_OFF		CLRPIN(ADC1_IN8_ADI5_AUX1_GPIO_Port,ADC1_IN8_ADI5_AUX1_Pin)

// Enums
enum DIGOUT_SET {DO_LOW, DO_HIGH, DO_TOGGLE};

// Public types
struct MQTT_TAB
	{
	char topic[33];
	char message[9];
	char format[9];
	float value;
	};

// HAL peripheral structure definitions
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern CAN_HandleTypeDef hcan1;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

// HAL function declaration
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_CAN1_Init(void);
void MX_RTC_Init(void);
void MX_SPI2_Init(void);
void MX_SPI3_Init(void);
void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

// Function declaration
uint32_t COMSendStr(char *s);
uint32_t COMReadStr(char *buf, uint32_t buflen);
uint32_t COMSendBuf(uint8_t *buf, uint32_t size);
uint32_t COMSendByte(uint8_t data);
uint32_t RS232SendStr(char *s);
uint32_t RS232ReadStr(char *buf, uint32_t buflen);
uint32_t RS485SendStr(char *s);
uint32_t RS485ReadStr(char *buf, uint32_t buflen);
uint32_t RS485SendBuf(uint8_t *buf, uint32_t size);
uint32_t RS485ReadBuf(uint8_t *buf, uint32_t buflen);
void RS485ReadFlush(void);
uint32_t IOL1SendStr(char *s);
uint32_t IOL1ReadStr(char *buf, uint32_t buflen);
uint32_t IOL2SendStr(char *s);
uint32_t IOL2ReadStr(char *buf, uint32_t buflen);
uint16_t SPI2_SendWord(uint16_t data);
uint8_t IOL_ReadReg(uint8_t regno);
uint8_t IOL_WrUpdReg(uint8_t regno, uint8_t data);
int IOL_Init(void);
void IOL_DigOut(uint32_t outno, uint32_t set);
// void MQTTDigOut(void);
int MBSendQuery(uint8_t addr, uint8_t func, uint16_t reg, uint8_t rcount, uint8_t *data);
int MBProcResponse(uint8_t *data, int nbuf);
int SMReadVal(uint16_t reg, float* value);
uint8_t SPI3_SendByte(uint8_t data);
uint8_t RFM_Read1Reg(uint8_t regno);
uint8_t RFM_Write1Reg(uint8_t regno, uint8_t data);
static int ParseMessage(char *istr);
static int ProcMessage(void);

#endif /* __EDGE_H */
