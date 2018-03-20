/*
 * File:		MQTT_Gsm.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Madhumadi Dhanasingh, ed. Ivan Dolezal
 * Date:		12. 2. 2018, 23. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:

Communication with quection module.
getting responses.
Sending MQTT messages.
 */
#include "mqtt_gsm.h"
#include "edge.h"

#define _AT_ "AT\r"
#define _CLOSEPORT_ "AT+QICLOSE=0\r"
#define _TCPCONFIG_ "AT+QICSGP=1,1,\"internet.t-mobile.cz\",\"\",\"\",1\r"
#define _OPENPORT_ "AT+QIOPEN=1,0,\"TCP\",\"playground.g4t.io\",1883,0,1\r"
#define _PUBLISH_ "Publish error"
int error;
extern struct MQTT_TAB mqtt[];
//bool localdata;
extern volatile bool rx1_EOF_rcv;	// [ID]
//extern char channel, op;
extern void IOL_DigOut(uint32_t, uint32_t);
extern bool reconnect_flag;
int error;
char main_rx_buf[MAIN_RX_BUF_LEN];	// [ID] moved from MQTT_GSM.h

bool Init_quectel(void)
{
	char b;
	COMSendStr("AT+IPR=115200\r");
	
	flush_buf();
	COMSendStr("AT&W\r");
	flush_buf();
	COMSendByte(0x1A);
	COMSendStr("ATE0\r");
	flush_buf();
	b=Test_AT_send(15000);
	b = Configure_TCP();
	b = close_socket();
 	b = Open_Port();
return true; 	// [ID]
}


bool flush_buf(void)	// [ID] was bool
{
if (rx1_EOF_rcv)
	{
		COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
		readURC();
	}
	memset(main_rx_buf, 0, MAIN_RX_BUF_LEN);
return true; // [ID]
}


bool gsm_timeout(uint32_t tickstart, uint32_t timeout)
{
	
	if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout))
	{
		if (error == 0)
		{
			error = -1;
		}
		rx1_EOF_rcv = false;
		return false;	
	}
	else
	{
		return true;
	}

}
char Test_AT_send(uint32_t time)
{
	uint32_t Timeout_int = time;
	uint32_t Tickstart_int = 0U;
	Tickstart_int = HAL_GetTick();
	unsigned char localdata;     // non zero value
	
while(gsm_timeout(Tickstart_int, Timeout_int))
	{
	
		COMSendStr(_AT_);
		HAL_Delay(50);
		if (rx1_EOF_rcv)
		{
			error = -2;
			COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
			localdata = strcmp("\r\nOK\r\n", main_rx_buf);
			rx1_EOF_rcv = false;
			flush_buf();
			if (!localdata)
			{
				error = 0;
				break;
			}
		}
	}
	return error;
	
}

int close_socket()	// [ID] int
{
	uint32_t Timeout_int = 15000;
	uint32_t Tickstart_int = 0U;
	int error = 0;	// [ID]
	unsigned char localdata = 1, er = 0;     // non zero value
	for (int i = 0 ; i < 10 ; i++)
	{
		COMSendStr("AT+QICLOSE=0\r");
		HAL_Delay(100);
		Tickstart_int = HAL_GetTick();
		while (gsm_timeout(Tickstart_int, Timeout_int))
		{
		
	
			if (rx1_EOF_rcv)
			{
				HAL_Delay(300);
				error = -4;
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				localdata = strcmp("\r\nOK\r\n", main_rx_buf);
				rx1_EOF_rcv = false;
				flush_buf();
				if (!localdata)
				{
					error = 0;
					break;
					
				}
			}
		}
		if ((localdata == 0)&&(error == 0))
		{
			break;
		}
	}
	return error;
}

int Configure_TCP()
{
	uint32_t Timeout_int = 1000;
	uint32_t Tickstart_int = 0U;
	int error = 0;
	unsigned char localdata;      // non zero value
	for(int i = 0 ; i < 10 ; i++)
	{
		COMSendStr("AT+QICSGP=1,1,\"internet.t-mobile.cz\",\"\",\"\",1\r");
		Tickstart_int = HAL_GetTick();
		while (gsm_timeout(Tickstart_int, Timeout_int))
		{   
			if (rx1_EOF_rcv)
			{	
				HAL_Delay(200);
				error = -3;
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				localdata = strcmp("\r\nOK\r\n", main_rx_buf);
				rx1_EOF_rcv = false;
				flush_buf();
				if (!localdata)
				{
					error = 0;
					break;
				}
			}
		}
		if ((localdata == 0)&&(error == 0))
		{
			break;
		}
	}	
	return error;	
}

bool Mqtt_connect(char *clientID)
	{
	char protocol_name[] = "MQTT";
	COMSendStr("AT+QISEND=0\r");
	
	int len = 8 + strlen(protocol_name) + strlen(clientID);
	unsigned char arr1[3] = { 0x10, len, 0x00 }; 
	unsigned char arr2[5] = { 0x04, 0x02, 0x00, 0x3C, 0x00 };
	HAL_Delay(10);	
	COMSendBuf(arr1, 3);
	COMSendByte(strlen(protocol_name));
	HAL_Delay(5);
	COMSendBuf((uint8_t*)protocol_name, strlen(protocol_name));
	COMSendBuf(arr2, 5);
	COMSendByte(strlen(clientID));
	HAL_Delay(5);
	COMSendBuf((uint8_t*)clientID, strlen(clientID));
	COMSendByte(0x1A);
	HAL_Delay(100);
	return true;	// [ID]
}

int Mqtt_publish(unsigned char i)	// [ID] int
{

	uint32_t Timeout_int = 25000;
	uint32_t Tickstart_int = 0U;
	int error = 0;	// [ID]
	char *topic;
	char *message;
	char *p, *q; 
	topic = mqtt[i].topic;
	message = mqtt[i].message;
	unsigned char *mess;
	unsigned char tsize = strlen(topic);
	unsigned char len = 2 + tsize + strlen(message);
	unsigned char N = tsize + strlen(message);
	unsigned char localdata = 1;
	unsigned char arr1[4] = { 0x30, len, 0x00, tsize };
	Tickstart_int = HAL_GetTick();
	mess = malloc((N + 1) * sizeof(char));	// [ID] missing free() ???
	for (int i = 0; i < tsize; i++) 
	{
		mess[i] = topic[i];
	}
	for (int i = 0; i < strlen(message); i++) 
	{
		mess[tsize + i] = message[i];
	}
	mess[N] = 0x00;	
	
	while (gsm_timeout(Tickstart_int, Timeout_int))
	{
			
		COMSendStr("AT+QISEND=0\r");
		HAL_Delay(5);
		//flush_buf();
		COMSendBuf(arr1, 4);
		COMSendBuf(mess, N);	
		COMSendByte(0x1A);
		free(mess);
		HAL_Delay(10);
			
		if (rx1_EOF_rcv)
		{
			error = -6; 			// Publishing error
			COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
			readURC();
			p = strstr(main_rx_buf, "\r\nSEND OK\r\n");
			q = strstr(main_rx_buf, "\r\nERROR\r\n");		
			rx1_EOF_rcv = false;
			flush_buf();
			if (p != 0)
			{
				localdata = 0;
				error = 0;  
			}
					
			if (q != 0)
			{
				localdata = 0;
				reconnect_flag = true;	
				error = -8;  
			}
					
			
			if (!localdata)
			{
				break;
			}
		}
	}
	return error;
}  

// ***************************************************************************************
//|1 |2 |3 |4 |5 |6 |7 |8 |9 |10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|  |  |  |  |  |  |  |  |
//|\r|\n|+ |Q |U |I |R |C |: |\ |" |r |e |c |v |" | ,|0 |, |2 |3 |\r|\n|0 |  |  |  |  |  |  |  |  |
//******************************************************************************************        	

bool readURC()
{
	char i, cmd_len, closed, recv;
	char tot_len, meslen, message[20], urc[20];
	char *p, *q; 
	char *temp, *temp1, *temp2, *temp3, *temp4;
	char channel = 0;
	uint8_t op;
	char *sub_messages[4];
	int tempt[4];
	p = memchr(main_rx_buf, '+', MAIN_RX_BUF_LEN); 	
	//int error;
	
	if (p != NULL)												//checks for +
		{		
			q = strstr(p, "+QIURC: \"");
			if (q != NULL)											//	checks for "recv"/ "closed" keyword
				{
					q = q + 9;
					temp = memchr((q), '"', 8);
					//temp = memchr(temp, '"', 8);
					cmd_len = temp - (q);
					memcpy(urc, q, cmd_len); 
					urc[cmd_len] = '\0';
					recv = strcmp(urc, "recv");
					closed = strcmp(urc, "closed");
				} 
		
			if (recv == 0)											//if message is recv 
				{
					recv = 1;
					temp = memchr((p + 17 + 2), '\n', 6); 					 //check for /n befre strart of frame  '0' = 0x30(received subscribe messages)
					if(temp != NULL)									
					{
						if (* ++temp == '0')									//if next is start of frame =0							// \r\n + QIURC : \"recv\",0,23\r\n0
							{
																				//var = temp - (p+17);
							for(i = 0 ; i < 4 ; i++)
								{
								tempt[i] = memcmp((uint8_t*)(temp + 4), (uint8_t*)(mqtt[N_MQTT_OUT + i].topic), strlen(mqtt[N_MQTT_OUT + i].topic));
								}	// [ID] !!!
						// [ID] why 2 identical for cycles ?
								for (i = 0; i < 4; i++)
								{
								if (tempt[i] == 0)
									{
									temp = (temp + 4 + strlen(mqtt[N_MQTT_OUT + i].topic));
									temp1 = memchr(temp, '\r', 10);  
									meslen = temp1 - temp;
									memcpy(message, temp, meslen);
									message[meslen] = '\0';
									strcpy(mqtt[N_MQTT_OUT + i].message, message);
									channel = i + 1;
									op = operation(message);
										IOL_DigOut(channel, op);
									}
									
								}
							}		
					}
				}
		
			if (closed == 0)											//if message is closed
				{
					closed = 1;
					reconnect_flag = true;
				}
		}
	HAL_Delay(10);
return true; // [ID]
}



bool Mqtt_subscribe(char *topic)
{
	COMSendStr("AT+QISEND=0\r");
	HAL_Delay(50);
	unsigned char *mess;
	unsigned char tsize = strlen(topic);
	unsigned char len = 5 + tsize;
	unsigned char arr1[6] = { 0x82, len, 0x00, 0x0A, 0x00, tsize }; 
	COMSendBuf(arr1, 6);
	for (int i = 0; i < tsize; i++) 
	{	
		COMSendByte(topic[i]);
		HAL_Delay(5);
	} 
	COMSendByte(0x00);
	HAL_Delay(5);
	COMSendByte(0x1A);
	HAL_Delay(200);
return true; // [ID]
}

int Open_Port()
{
	uint32_t Timeout_int = 25000;
	uint32_t Tickstart_int = 0U;
	Tickstart_int = HAL_GetTick();
	//unsigned char localdata = 1, localdata1 = 1;        // non zero value
	int error = 0;	// [ID] int
// uint32_t *ld, ld1; // // [ID] !!!
	char *ld, *ld1; // [ID]
	ld = ld1 = NULL;
	
	for (int i = 0; i < 10; i++)
	{
		COMSendStr("AT+QIOPEN=1,0,\"TCP\",\"4tech.horcica.cz\",1883,0,1\r");
		while (gsm_timeout(Tickstart_int, Timeout_int))
		{
		
			HAL_Delay(200);
			if (rx1_EOF_rcv)
			{
				HAL_Delay(100);
				error = -5;
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				ld = strstr(main_rx_buf,"\r\nOK\r\n\r\n+QIOPEN: 0,0\r\n"); 
				ld1 = strstr(main_rx_buf,"\r\nOK\r\n\r\n+QIOPEN: 0,563\r\n");   //\r\nOK\r\n\r\n+QIOPEN: 0,563\r\n    already port open b etter to close and open
				rx1_EOF_rcv = false;
				flush_buf();
				HAL_Delay(100);
				if (ld!=NULL)
				{
					error = 0;
					break;
				}
				if (ld1 != NULL)
				{
					close_socket();
					break;
				}
			}
		}
		if ((ld!=NULL)&&(error == 0))
		{
			break;
		}
	}
	
return error;
}

uint8_t operation(char *message)
{

	if (!strcmp(message, "1"))
	{
		return 1;
	}
	if (!strcmp(message, "0"))
	{
		return 0;
	}
	if (!strcmp(message, "2"))
	{
		return 2;
	}
return 0;  // [ID]
}

	/*
	
	
	
	
bool  MQTTPublish(unsigned char i)
{
	{

		uint32_t Timeout_int = 25000;
		uint32_t Tickstart_int = 0U;
		Tickstart_int = HAL_GetTick();
		unsigned char *topic;
		unsigned char *message;
		topic = mqtt[i].topic;
		message = mqtt[i].message;
		unsigned char localdata = 1;
		while (gsm_timeout(_PUBLISH_, Tickstart_int, Timeout_int))
		{
			
			COMSendStr("AT+QISEND=0\r");
			HAL_Delay(30);
			unsigned char *mess;
			unsigned char tsize = strlen(topic);
			unsigned char msize = strlen(message);
			unsigned char len = 2 + tsize + msize;
			unsigned char N = tsize + msize;
			flush_buf();
			mess = malloc((N + 1) * sizeof(char));
			for (int i = 0; i < tsize; i++) {
				mess[i] = topic[i];
			}
			for (int i = 0; i < msize; i++) {
				mess[tsize + i] = message[i];
			}
			mess[N] = 0X00;
			COMSendByte(0x30);
			HAL_Delay(1);
			COMSendByte(len);
			HAL_Delay(1);
			COMSendByte(0x00);
			HAL_Delay(1);
			COMSendByte(tsize);
			HAL_Delay(1);
			for (int i = 0; i < N; i++) {
		
				COMSendByte(mess[i]);
				HAL_Delay(1);
			} 
			COMSendByte(0x1A);
			HAL_Delay(50);
			
			if (rx1_EOF_rcv)
			{
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				localdata = strcmp("\r\nSEND OK\r\n", main_rx_buf);
				rx1_EOF_rcv = false;
				flush_buf();
				if (!localdata)
				{
					error = false;
					break;
				}
			}
		}
		return error;
	}
	
}

char Open_Port()
{
	uint32_t Timeout_int = 25000;
	uint32_t Tickstart_int = 0U;
	Tickstart_int = HAL_GetTick();
	unsigned char localdata = 1, localdata1 = 1;        // non zero value
	error = 0;

	
	for (int i = 0; i < 10; i++)
	{
		COMSendStr("AT+QIOPEN=1,0,\"TCP\",\"4tech.horcica.cz\",1883,0,1\r");
		while (gsm_timeout(Tickstart_int, Timeout_int))
		{
		
			HAL_Delay(200);
			if (rx1_EOF_rcv)
			{
				HAL_Delay(1000);
				error = -5;
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				localdata = strcmp(main_rx_buf,"\r\nOK\r\n\r\n+QIOPEN: 0,0\r\n"); 
				//LD1 = strstr("\r\nOK\r\n\r\n+QIOPEN: 0,563\r\n", main_rx_buf);  //\r\nOK\r\n\r\n+QIOPEN: 0,563\r\n
				rx1_EOF_rcv = false;
				flush_buf();
				HAL_Delay(100);
				if (!localdata)
				{
					error = 0;
					break;
				}
		
			}
		}
		if ((localdata ==0)&&(error == 0))
		{
			//break;
		}
	}
	

	return error;
}

}*/