#pragma once
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#define		Domain_name									"\"4tech.horcica.cz\","
#define		Portno										"1883,"
#define	 MAIN_RX_BUF_LEN									150
// #define	 main_rx_buf									Mainrxbuf

bool Init_quectel(void);
bool Gsm_SendRaw(uint8_t *, uint16_t);
bool Gsm_SendString(char *);
bool flush_buf(void);
char Test_AT_send(uint32_t time);
int close_socket();	// [ID] int
int Configure_TCP();	// [ID] int
int Open_Port();	// [ID] int
bool Mqtt_connect(char *clientID);
int Mqtt_publish(unsigned char i); 	// [ID] int
//char Mqtt_publish(unsigned char i);
bool Mqtt_subscribe(char *topic);
bool gsm_timeout(uint32_t tickstart, uint32_t timeout);
bool readURC();
uint8_t operation(char *message);