//-----------------------------------------------------------------------------
uint32_t UART1ReadStr(char *buf, uint32_t buflen)
{
uint32_t nchr = 0;
if (rx1_counter != 0)
	{
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
	for ( ; buflen > 0; --buflen)
		{
		*buf++ = rx1_buffer[rx1_rd_index];
		if (++rx1_rd_index == RX1_BUFFER_SIZE)
			rx1_rd_index = 0;
		++nchr;
		if (--rx1_counter == 0)
			break;
		}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	}
return nchr;
}

//-----------------------------------------------------------------------------
uint32_t UART1SendStr(char *s)
{
uint32_t nchr = 0;
while (tx1_counter == TX1_BUFFER_SIZE);
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
