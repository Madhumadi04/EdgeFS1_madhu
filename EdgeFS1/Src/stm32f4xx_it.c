/*
 * File:		stm32f4xx_it.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		20. 1. 2018, 23. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
Test of UART1 (circullar buffers, ISR, STDIO redirection)
UART2 & 6 by ...SendStr() & ...ReadStr()
ADC averaging
! changed COMSendBuf(uint8_t* buf,...)
 */

/* Includes ------------------------------------------------------------------*/
#include "edge.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include <stdbool.h>
#include <stdio.h>
#include <sys/stat.h>

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
   while (1)
  {
  }
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
HAL_IncTick();
// HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

// Special Characters
#define	CR		13	// 0x0D
#define	LF		10	// 0x0A

// USART1
// Receiver buffer & variables
#define RX1_BUFFER_SIZE 50
char rx1_buffer[RX1_BUFFER_SIZE];
char rx1_EOF_chr = LF;
uint16_t rx1_wr_index, rx1_rd_index;
volatile uint16_t rx1_counter;
volatile bool rx1_EOF_rcv, rx1_buffer_overflow; 
// Transmitter buffer & variables
#define TX1_BUFFER_SIZE 100
char tx1_buffer[TX1_BUFFER_SIZE];
volatile uint16_t tx1_wr_index, tx1_rd_index, tx1_counter;

// USART2
// Receiver buffer & variables
#define RX2_BUFFER_SIZE 50
char rx2_buffer[RX2_BUFFER_SIZE];
char rx2_EOF_chr = LF;
uint16_t rx2_wr_index, rx2_rd_index;
volatile uint16_t rx2_counter;
volatile bool rx2_EOF_rcv, rx2_buffer_overflow; 
// Transmitter buffer & variables
#define TX2_BUFFER_SIZE 100
char tx2_buffer[TX2_BUFFER_SIZE];
volatile uint16_t tx2_wr_index, tx2_rd_index, tx2_counter;

// USART6
// Receiver buffer & variables
#define RX6_BUFFER_SIZE 50
uint8_t rx6_buffer[RX6_BUFFER_SIZE];
char rx6_EOF_chr = LF;
uint16_t rx6_wr_index, rx6_rd_index;
volatile uint16_t rx6_counter;
volatile bool rx6_EOF_rcv, rx6_buffer_overflow; 
// Transmitter buffer & variables
#define TX6_BUFFER_SIZE 100
uint8_t tx6_buffer[TX6_BUFFER_SIZE];
volatile uint16_t tx6_wr_index, tx6_rd_index, tx6_counter;

// UART4
// Receiver buffer & variables
#define RX4_BUFFER_SIZE 50
char rx4_buffer[RX4_BUFFER_SIZE];
char rx4_EOF_chr = LF;
uint16_t rx4_wr_index, rx4_rd_index;
volatile uint16_t rx4_counter;
volatile bool rx4_EOF_rcv, rx4_buffer_overflow; 
// Transmitter buffer & variables
#define TX4_BUFFER_SIZE 100
char tx4_buffer[TX4_BUFFER_SIZE];
volatile uint16_t tx4_wr_index, tx4_rd_index, tx4_counter;

// UART5
// Receiver buffer & variables
#define RX5_BUFFER_SIZE 50
char rx5_buffer[RX5_BUFFER_SIZE];
char rx5_EOF_chr = LF;
uint16_t rx5_wr_index, rx5_rd_index;
volatile uint16_t rx5_counter;
volatile bool rx5_EOF_rcv, rx5_buffer_overflow; 
// Transmitter buffer & variables
#define TX5_BUFFER_SIZE 100
char tx5_buffer[TX5_BUFFER_SIZE];
volatile uint16_t tx5_wr_index, tx5_rd_index, tx5_counter;

//-----------------------------------------------------------------------------
// Auxilliary functions for the redirection
int _fstat(int fd, struct stat *pStat)
	{
	pStat->st_mode = S_IFCHR;
	return 0;
	}

caddr_t _sbrk(int increment)
	{
	extern char end asm("end");
	register char *pStack asm("sp");
 
	static char *s_pHeapEnd;
 
	if (!s_pHeapEnd)
		s_pHeapEnd = &end;
 
	if (s_pHeapEnd + increment > pStack)
		return (caddr_t) - 1;
 
	char *pOldHeapEnd = s_pHeapEnd;
	s_pHeapEnd += increment;
	return (caddr_t)pOldHeapEnd;
	}

#if STDIO_UART == 1
//  Functions for ARM GCC STDIO redirection of USART1
//-----------------------------------------------------------------------------
int _write(int fd, char *pBuffer, int size)
{
uint32_t nchr = 0;
while (tx1_counter == TX1_BUFFER_SIZE);
__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
for (int i = 0; i < size; i++)
	{
	tx1_buffer[tx1_wr_index] = pBuffer[i];
	if (++tx1_wr_index == TX1_BUFFER_SIZE)
		tx1_wr_index = 0;
	++tx1_counter;
	++nchr;
	}
__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
return nchr;
}

//-----------------------------------------------------------------------------
int _read(int fd, char *pBuffer, int size)
{
uint32_t nchr = 0;
if (rx1_counter != 0)
	{
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	for (int i = 0; i < size; i++)
		{
		pBuffer[i] = rx1_buffer[rx1_rd_index];
		++nchr;
		if (++rx1_rd_index == RX1_BUFFER_SIZE)
			rx1_rd_index = 0;
		if (--rx1_counter == 0)
			break;
		}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	}
return nchr;
}

//-----------------------------------------------------------------------------
uint32_t RS232ReadStr(char *buf, uint32_t buflen)
	{
	uint32_t nchr = 0;
	char c;
	if (rx2_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				c = *buf++ = rx2_buffer[rx2_rd_index];
				++nchr;
				if (++rx2_rd_index == RX2_BUFFER_SIZE)
					rx2_rd_index = 0;
				if (--rx2_counter == 0  || c == rx2_EOF_chr)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}
	*buf = '\0';
	return nchr;
	}
#endif

//-----------------------------------------------------------------------------
uint32_t RS232SendStr(char *s)
	{
	uint32_t nchr = 0;
	while (tx2_counter == TX2_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
	while (*s != '\0')
		{
		tx2_buffer[tx2_wr_index] = *s++;
		if (++tx2_wr_index == TX2_BUFFER_SIZE)
			tx2_wr_index = 0;
		++tx2_counter;
		++nchr;
		}
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
	return nchr;
	}


#if STDIO_UART == 2
//  Functions for ARM GCC STDIO redirection of USART2
//-----------------------------------------------------------------------------
int _write(int fd, char *pBuffer, int size)
	{
	uint32_t nchr = 0;
	while (tx2_counter == TX2_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
	for (int i = 0; i < size; i++)
		{
		tx2_buffer[tx2_wr_index] = pBuffer[i];
		if (++tx2_wr_index == TX2_BUFFER_SIZE)
			tx2_wr_index = 0;
		++tx2_counter;
		++nchr;
		}
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
	return nchr;
	}

//-----------------------------------------------------------------------------
int _read(int fd, char *pBuffer, int size)
	{
	uint32_t nchr = 0;
	if (rx2_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
		for (int i = 0; i < size; i++)
			{
			pBuffer[i] = rx2_buffer[rx2_rd_index];
			++nchr;
			if (++rx2_rd_index == RX2_BUFFER_SIZE)
				rx2_rd_index = 0;
			if (--rx2_counter == 0)
				break;
			}
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t COMReadStr(char *buf, uint32_t buflen)		// USART1, Communication Connector
	{
	uint32_t nchr = 0;
	char c;
	if (rx1_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				c = *buf++ = rx1_buffer[rx1_rd_index];
				++nchr;
				if (++rx1_rd_index == RX1_BUFFER_SIZE)
					rx1_rd_index = 0;
				if (--rx1_counter == 0 || c == rx1_EOF_chr)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		}
	*buf = '\0';
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t COMSendStr(char *s)	//  USART1, Communication Connector
	{
	uint32_t nchr = 0;
	while (tx1_counter == TX1_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
	while (*s != '\0')
		{
		tx1_buffer[tx1_wr_index] = *s++;
		if (++tx1_wr_index == TX1_BUFFER_SIZE)
			tx1_wr_index = 0;
		++tx1_counter;
		++nchr;
		}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t COMSendBuf(uint8_t *buf, uint32_t size)
	{
	uint32_t i;
	while (tx1_counter == TX1_BUFFER_SIZE);
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
	for (i = 0; i < size; i++)
		{
		tx1_buffer[tx1_wr_index] = buf[i];
		if (++tx1_wr_index == TX1_BUFFER_SIZE)
			tx1_wr_index = 0;
		++tx1_counter;
		}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
	return size;
	}

//-----------------------------------------------------------------------------
uint32_t COMSendByte(uint8_t data)
	{
	if (tx1_counter == TX1_BUFFER_SIZE)
		return 0;	// when buffer is full returns 0
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
		tx1_buffer[tx1_wr_index] = (char)data;
		if (++tx1_wr_index == TX1_BUFFER_SIZE)
			tx1_wr_index = 0;
		++tx1_counter;
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
	return 1;	// always only 1 byte
	}

#endif


/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
// HAL_UART_IRQHandler(&huart1);
char c;
uint16_t SRflags;
SRflags = USART1->SR; 	// sequence for clearing error flags
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
	{                  // read interrupt
   c = USART1->DR & 0x00FF;
	rx1_buffer[rx1_wr_index] = c;
	if (c == rx1_EOF_chr)
		rx1_EOF_rcv = true;
	if (++rx1_wr_index == RX1_BUFFER_SIZE)
		rx1_wr_index = 0;
	if (++rx1_counter == RX1_BUFFER_SIZE)
		{
		rx1_counter = 0;
		rx1_buffer_overflow = true;
		}
	}
if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
	{   
	USART1->DR = tx1_buffer[tx1_rd_index];
	if (++tx1_rd_index == TX1_BUFFER_SIZE)
		tx1_rd_index = 0;
	if (--tx1_counter == 0)
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
	}
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
// HAL_UART_IRQHandler(&huart2);
char c;
uint16_t SRflags;
// AUX1_ON; // Minimum 0.87 us
SRflags = USART2->SR;
if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
	{
	c = USART2->DR & 0x00FF;
	rx2_buffer[rx2_wr_index] = c;
	if (c == rx2_EOF_chr)
		rx2_EOF_rcv = true;
	if (++rx2_wr_index == RX2_BUFFER_SIZE)
		rx2_wr_index = 0;
	if (++rx2_counter == RX2_BUFFER_SIZE)
		{
		rx2_counter = 0;
		rx2_buffer_overflow = true;
		}
	}

if (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE))
	{   
	USART2->DR = tx2_buffer[tx2_rd_index];
	if (++tx2_rd_index == TX2_BUFFER_SIZE)
		tx2_rd_index = 0;
	if (--tx2_counter == 0)
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
	}
// AUX1_OFF;
}

//-----------------------------------------------------------------------------
uint32_t RS485ReadStr(char *buf, uint32_t buflen)	// USART6
	{
	uint32_t nchr = 0;
	char c;
	if (rx6_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				c = *buf++ = rx6_buffer[rx6_rd_index];
				++nchr;
				if (++rx6_rd_index == RX6_BUFFER_SIZE)
					rx6_rd_index = 0;
				if (--rx6_counter == 0 || c == rx6_EOF_chr)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
		}
	*buf = '\0';
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t RS485SendStr(char *s)	// USART6
	{
	uint32_t nchr = 0;
	while (tx6_counter == TX6_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_TXE);
	while (*s != '\0')
		{
		tx6_buffer[tx6_wr_index] = *s++;
		if (++tx6_wr_index == TX6_BUFFER_SIZE)
			tx6_wr_index = 0;
		++tx6_counter;
		++nchr;
		}
	if (nchr > 0)
		{
		SETPIN(RS485_TXE_GPIO_Port, RS485_TXE_Pin);
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_TXE);
		}
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t RS485ReadBuf(uint8_t *buf, uint32_t buflen)	// USART6
	{
	uint32_t nchr = 0;
	if (rx6_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				*buf++ = rx6_buffer[rx6_rd_index];
				++nchr;
				if (++rx6_rd_index == RX6_BUFFER_SIZE)
					rx6_rd_index = 0;
				if (--rx6_counter == 0)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
		}
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t RS485SendBuf(uint8_t *buf, uint32_t size)
	{
	uint32_t i;
	while (tx6_counter == TX6_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_TXE);
	for (i = 0; i < size; i++)
		{
		tx6_buffer[tx6_wr_index] = buf[i];
		if (++tx6_wr_index == TX6_BUFFER_SIZE)
			tx6_wr_index = 0;
		++tx6_counter;
		}
	SETPIN(RS485_TXE_GPIO_Port, RS485_TXE_Pin);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TXE);
	return size;
	}

//-----------------------------------------------------------------------------
void RS485ReadFlush(void)
	{
	rx6_wr_index = rx6_rd_index = rx6_counter = 0;	
	}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
	{
	// HAL_UART_IRQHandler(&huart6);
	char c;
	uint16_t SRflags;
	SRflags = USART6->SR;
	// TO DO: error handling

#ifdef MODBUS	
	if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE)))
		{
		c = USART6->DR;  		// to complete the clear sequence
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
		rx6_EOF_rcv = true;
		}
#endif

	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
		{
		c = USART6->DR & 0x00FF;   	// 2nd part of sequence for clearing error flags
		rx6_buffer[rx6_wr_index] = c;
	#ifdef MODBUS	
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	#else
		if (c == rx6_EOF_chr)
			rx6_EOF_rcv = true;
	#endif
		if (++rx6_wr_index == RX6_BUFFER_SIZE)
			rx6_wr_index = 0;
		if (++rx6_counter == RX6_BUFFER_SIZE)
			{
			rx6_counter = 0;
			rx6_buffer_overflow = true;
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE))
		{   
		USART6->DR = tx6_buffer[tx6_rd_index];
		if (++tx6_rd_index == TX6_BUFFER_SIZE)
			tx6_rd_index = 0;
		if (--tx6_counter == 0)
			{
			__HAL_UART_DISABLE_IT(&huart6, UART_IT_TXE);
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);	// to wait for transmitt completion
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_TC) && __HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC))
		{
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_TC);
		CLRPIN(RS485_TXE_GPIO_Port, RS485_TXE_Pin);
		}
	}

//-----------------------------------------------------------------------------
uint32_t IOL2ReadStr(char *buf, uint32_t buflen)	// UART4
	{
	uint32_t nchr = 0;
	char c;
	if (rx4_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				c = *buf++ = rx4_buffer[rx4_rd_index];
				++nchr;
				if (++rx4_rd_index == RX4_BUFFER_SIZE)
					rx4_rd_index = 0;
				if (--rx4_counter == 0 || c == rx4_EOF_chr)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
		}
	*buf = '\0';
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t IOL2SendStr(char *s)		// UART4
	{
	uint32_t nchr = 0;
	while (tx4_counter == TX4_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
	while (*s != '\0')
		{
		tx4_buffer[tx4_wr_index] = *s++;
		if (++tx4_wr_index == TX4_BUFFER_SIZE)
			tx4_wr_index = 0;
		++tx4_counter;
		++nchr;
		}
	if (nchr > 0)
		{
		SETPIN(IOLINK2_TXE_GPIO_Port, IOLINK2_TXE_Pin);
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
		}
	return nchr;
	}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)	// IOL2
	{
	// HAL_UART_IRQHandler(&huart4);
	char c;
	uint16_t SRflags;
	SRflags = UART4->SR;
	if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE))
		{
		c = UART4->DR & 0x00FF;
		rx4_buffer[rx4_wr_index] = c;
		if (c == rx4_EOF_chr)
			rx4_EOF_rcv = true;
		if (++rx4_wr_index == RX4_BUFFER_SIZE)
			rx4_wr_index = 0;
		if (++rx4_counter == RX4_BUFFER_SIZE)
			{
			rx4_counter = 0;
			rx4_buffer_overflow = true;
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE))
		{   
		UART4->DR = tx4_buffer[tx4_rd_index];
		if (++tx4_rd_index == TX4_BUFFER_SIZE)
			tx4_rd_index = 0;
		if (--tx4_counter == 0)
			{
			__HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
			__HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);  	// to wait for transmitt completion
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TC) && __HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC))
		{
		__HAL_UART_DISABLE_IT(&huart4, UART_IT_TC);
		CLRPIN(IOLINK2_TXE_GPIO_Port, IOLINK2_TXE_Pin);
		}

	}

//-----------------------------------------------------------------------------
uint32_t IOL1ReadStr(char *buf, uint32_t buflen)	// UART5
	{
	uint32_t nchr = 0;
	char c;
	if (rx5_counter != 0)
		{
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
		for (; buflen > 1; --buflen)		// last char in buffer for '\0'
				{
				c = *buf++ = rx5_buffer[rx5_rd_index];
				++nchr;
				if (++rx5_rd_index == RX5_BUFFER_SIZE)
					rx5_rd_index = 0;
				if (--rx5_counter == 0 || c == rx5_EOF_chr)
					break;
				}
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
		}
	*buf = '\0';
	return nchr;
	}

//-----------------------------------------------------------------------------
uint32_t IOL1SendStr(char *s)		// UART5
	{
	uint32_t nchr = 0;
	while (tx5_counter == TX5_BUFFER_SIZE) ;
	__HAL_UART_DISABLE_IT(&huart5, UART_IT_TXE);
	while (*s != '\0')
		{
		tx5_buffer[tx5_wr_index] = *s++;
		if (++tx5_wr_index == TX5_BUFFER_SIZE)
			tx5_wr_index = 0;
		++tx5_counter;
		++nchr;
		}
	if (nchr > 0)
		{
		SETPIN(IOLINK1_TXE_GPIO_Port, IOLINK1_TXE_Pin);
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_TXE);
		}
	return nchr;
	}

/**
* @brief This function handles UART5 global interrupt.
*/
void UART5_IRQHandler(void)	// IOL1
	{
	// HAL_UART_IRQHandler(&huart4);
	char c;
	uint16_t SRflags;
	SRflags = UART5->SR;   	// sequence for clearing error flags
	if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE))
		{
		c = UART5->DR & 0x00FF;
		rx5_buffer[rx5_wr_index] = c;
		if (c == rx5_EOF_chr)
			rx5_EOF_rcv = true;
		if (++rx5_wr_index == RX5_BUFFER_SIZE)
			rx5_wr_index = 0;
		if (++rx5_counter == RX5_BUFFER_SIZE)
			{
			rx5_counter = 0;
			rx5_buffer_overflow = true;
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE))
		{   
		UART5->DR = tx5_buffer[tx5_rd_index];
		if (++tx5_rd_index == TX5_BUFFER_SIZE)
			tx5_rd_index = 0;
		if (--tx5_counter == 0)
			{
			__HAL_UART_DISABLE_IT(&huart5, UART_IT_TXE);
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_TC);  	// to wait for transmitt completion
			}
		}

	if (__HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_TC) && __HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC))
		{
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_TC);
		CLRPIN(IOLINK1_TXE_GPIO_Port, IOLINK1_TXE_Pin);
		}
	}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
// AUX1_ON;
HAL_DMA_IRQHandler(&hdma_adc1);	// Duration usually 1.6-3.4 us, max. 9.5 us
// AUX1_OFF;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
