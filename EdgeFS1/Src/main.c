/*
 * File:		main.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		12. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
USART operation (circullar buffers, ISR, STDIO redirection)
Single 1Wire thermometer DS12B20
ADC averaging & calibration
Timing Scheduler
USART2 (STDOUT)
USART6 (RS485)
IO-Link IC output both CQ & L+
IO-Link UART4,5
MQTT structure & assignment
MODBUS: UART message end detection, protocol functions, SmartMeter reading
SPI3 on COM connector, read/write registers of RFM9x
STDIN command parsing
* Doing:
Agilent DMM value reception - RTC/CTS handshaking
Set command via console for Digital Outputs
Incl. GSM/LTE Quectel & MQTT publishing/subscription
* To Do:
* Bugs:
 */

#define	VERSION			"V1.14"

#define	DEBUG_PRINT	
#define	PRINT_INFO		dip1		// set by reading of DIP1-2 switches		
// DIP1 sets initial message sending and console command readings, turn off DMM reception
#define	PRINT_VALUES	(dip1 && dip2)
#define	PRINTI(...)		if (PRINT_INFO) printf(__VA_ARGS__)
#define	PRINTV(...)		if (PRINT_VALUES) printf(__VA_ARGS__)
#define	TST(i)			ts[i].elapsed			// Timing Scheduler test
#define	TSC(i)			ts[i].elapsed=false

/* Includes ------------------------------------------------------------------*/
#include "edge.h"
#include "main.h"
#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_ds18b20.h"
#include "mqtt_gsm.h"

/* Private function declaration ----------------------------------------------*/

/* Private definitons --------------------------------------------------------*/
#define INSTR_LEN				100

// ADC
#define ADI_DIV_RATIO		0.250	// resistor divider on inputs
#define ADI_THR_VOLT			2.5	// voltage for H level
#define ADI_CURR_RATIO		0.1	// shunt ration on inputs
#define N_ADI_CHAN			4
#define N_ADC_CHAN			(N_ADI_CHAN + 1)	// incl. REF voltage channel
#define ADC_BUF_LEN			N_ADC_CHAN
#define ADC_FCLK				21000000	// (APBCLK=84 MHz)/4
#define ADC_CONVCYC			(144+12)
#define N_ADC_AVG				(ADC_FCLK/(50 * N_ADC_CHAN * ADC_CONVCYC))	// 538.46 (for 20 ms averaging i.e. 50 Hz supression)
#define M_OVERSAMP			10		// to be precised
#define N_AVG_OVERSAMP		(N_ADC_AVG/M_OVERSAMP)
#define ADI_THR_VAL			(ADI_THR_VOLT * ADI_DIV_RATIO * M_OVERSAMP * 4095.0 / 3.3)	// threshold of H level 2.5 V

// Parser error codes
#define	ERR_CMD_EMPTY			-11
#define	ERR_CMD_UNKNOWN		-12
#define	ERR_FEW_PAR   			-13
#define	ERR_WRONG_PAR 			-14

// Privat types
struct TIMESCHED 
{
	uint32_t ticks0;
	uint32_t	period;
	bool enabled;
	bool elapsed;
	bool repeat;
}
;

// Extern variables
extern volatile bool rx1_EOF_rcv, rx2_EOF_rcv, rx6_EOF_rcv, rx4_EOF_rcv, rx5_EOF_rcv;
extern uint8_t mbdata[MB_DATA_SIZE], mberrslave;

// Global variables
TM_OneWire_t OneWire1;
bool SMpresent = false;
int rtv, smquery = 0;
enum TS_ITEMS { TS_SIG, TS_FLASH, TS_ADC, TS_1WIRE, TS_IOL_OUT, TS_BUZ, TS_MB, TS_AUX, END_MARK };
#define N_TIME_SCHED	END_MARK	// time scheduler items No.
struct TIMESCHED ts[N_TIME_SCHED];

char ClientID[] = "EdgeFS1";
struct MQTT_TAB mqtt[N_MQTT_TOPIC] = {
	{ "EdgeFS1/1wire/temperature/th1", "", "%+07.2f", 0.0 },
	{ "EdgeFS1/1wire/temperature/th2", "", "%+07.2f", 0.0 },
	{ "EdgeFS1/1wire/temperature/th3", "", "%+07.2f", 0.0 },
	{ "EdgeFS1/ADI/chan1/voltage", "", "%06.3f", 0.0 },
	{ "EdgeFS1/ADI/chan2/current", "", "%06.3f", 0.0 },
	{ "EdgeFS1/ADI/chan3/logical", "", "%1.0f", 0.0 },
	{ "EdgeFS1/ADI/chan4/logical", "", "%1.0f", 0.0 },
	{ "EdgeFS1/RS485/SM/voltage", "", "%05.1f", 0.0 },
	{ "EdgeFS1/RS485/SM/power", "", "%05.3f", 0.0 }, 
		// in kW
	{ "EdgeFS1/RS485/SM/pfactor", "", "%05.3f", 0.0 },
	{ "EdgeFS1/DO/chan1", "", "%1.0f", 0.0 },
	 	// relay
	{ "EdgeFS1/DO/chan2", "", "%1.0f", 0.0 },
	 	// LED
	{ "EdgeFS1/DO/chan3", "", "%1.0f", 0.0 },
	 	// bulb
	{ "EdgeFS1/DO/chan4", "", "%1.0f", 0.0 }	// buzzer
};
volatile bool adcnewval;
char in1str[INSTR_LEN], in2str[INSTR_LEN], cmdstr[INSTR_LEN];
uint32_t adcbuf[ADC_BUF_LEN], adcsum[ADC_BUF_LEN], adcval[ADC_BUF_LEN];
bool diginp[N_ADI_CHAN], inpmode[N_ADI_CHAN] = { false, true, false, false }; 	// TRUE means the current input
bool dip1, dip2, buzzer;
uint16_t vrefcal;
volatile uint32_t iadcavg;
const char delim[] = { ' ', '\r', '\n', ':', '/', '\0' };
int error, npar, inpar1, inpar2, inpar3;
#ifdef GSM_MQTT
bool reconnect_flag;
extern char main_rx_buf[MAIN_RX_BUF_LEN];
#endif

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief  Delays for amount of micro seconds
 * @param  micros: Number of microseconds for delay
 * @retval None
 */
//------------------------------------------------------------------------------
void Delay_us(__IO uint32_t micros) 
{
	// !!! DWT counter does not run without J-Link debugging
	/*
	#if !defined(STM32F0xx)
		uint32_t start = DWT->CYCCNT;    
		micros *= (HAL_RCC_GetHCLKFreq() / 1000000);   	// Go to number of cycles for system
		while((DWT->CYCCNT - start) < micros);   			// Delay till end
		#else
		*/
		    micros *= (SystemCoreClock / 1000000) / 8;    	// Go to clock cycles	// originally /5 for STM32F0xx
		    while(micros--);    									// Wait till done
		    // #endif
}

//------------------------------------------------------------------------------
static void TimingScheduler(struct TIMESCHED ts[])
{
	uint32_t i, ticks;
	ticks = HAL_GetTick();

	for (i = 0; i < N_TIME_SCHED; i++)
	{ 
		if (ts[i].enabled && (ticks - ts[i].ticks0) >= ts[i].period)
		{
			ts[i].elapsed = true;
			if (ts[i].repeat)
				ts[i].ticks0 += ts[i].period;
			else
				ts[i].enabled = false;
		}
	}
}

//------------------------------------------------------------------------------
static void TimingSchedulerInit(struct TIMESCHED ts[])
{
	uint32_t t0 = HAL_GetTick();
	for (int i = 0; i < N_TIME_SCHED; i++)
	{
		ts[i].enabled = true;	
		ts[i].repeat = true;	
		ts[i].ticks0 = t0;	
	}
}

//------------------------------------------------------------------------------
static void TimingOneShot(struct TIMESCHED ts[], int i, uint32_t delay)
{
	ts[i].ticks0 = HAL_GetTick();
	ts[i].period = delay;
	ts[i].repeat = false;	
	ts[i].enabled = true;
}

//------------------------------------------------------------------------------
int main(void)
{
	uint32_t onewire_count, iadc = 0, imainloop = 0;
	int i, j, n, rtv;
	uint8_t onewire_devices;
	uint8_t onewire_code[N_ONEWIRE_SENSORS][8];
	float DS12B20_th[N_DS12B20];
	float calib0, voltcoef, currcoef, ADIval[N_ADI_CHAN], SMvoltage, SMpower, SMpfactor, SMfrequency, SMcurrent;
	bool supply24V = false, SMpresent = false;
	
	HAL_Init();
	MX_GPIO_Init();          			// to be LED outputs set for Error_Handler
	SystemClock_Config();
	//	DWT->CTRL |= 0x1;	// starting DWT could not help to Delay_us() working
	MX_SPI2_Init();       				// for early IOL switching off
	rtv = IOL_Init();  					// also for print of IOL IC working
	supply24V = (rtv == 1);

	// MX_RTC_Init();
	MX_DMA_Init();       	// for ADC
	MX_ADC1_Init();
	// MX_CAN1_Init();
	MX_UART4_Init();    			// IOL2
	MX_UART5_Init();    			// IOL1
	MX_USART2_UART_Init();      // RS232
	MX_USART6_UART_Init();      // RS485

#ifdef COM_SPI
		MX_SPI3_Init();      		// shared COM connector
#else
		MX_USART1_UART_Init();   // shared COM connector
#endif

	/* Initialize OneWire on pin PD0 */
	TM_OneWire_Init(&OneWire1, ONEWIRE_GPIO_Port, ONEWIRE_Pin);

	// Initializing some values
	vrefcal = *(uint16_t *)0x1FFF7A2A;           	// from the manufacturer in the system memory
	
	dip2 = !HAL_GPIO_ReadPin(DIP0_GPIO_Port, DIP0_Pin);
	dip1 = !HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin);

	// Signalization of a start
	// HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin | LEDB_Pin, GPIO_PIN_SET);
	SETPIN(LEDR_GPIO_Port, LEDR_Pin | LEDB_Pin);
	HAL_Delay(2000);
	// Delay_us(1500000);
	//	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin | LEDB_Pin, GPIO_PIN_RESET);
	CLRPIN(LEDR_GPIO_Port, LEDR_Pin | LEDB_Pin);
	HAL_Delay(1000);

	// UART additiona init
	setvbuf(stdout, NULL, _IONBF, 0);         	// switch off a buffering, so output does not wait for \n
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	__HAL_UART_FLUSH_DRREGISTER(&huart2);

#ifdef	DEBUG_PRINT
	PRINTI("\r\nEdgeNodeFS Debug " VERSION ", STDOUT @ RS232 (UART2)\r\n");
	if (rtv < 0)	// from IOLInit()
		{ PRINTI("IO-Link IC failed!\r\n"); }
	else
		PRINTI("Supply 24 V: %s\r\n", (supply24V) ? "ON" : "OFF");
	// UART1SendStr("\r\nEdgeNodeFS Debug " VERSION " on UART1\r\n");
#endif
	// Checks for any device on 1-wire
	onewire_count = 0;
	TM_OneWire_Reset(&OneWire1);
	onewire_devices = TM_OneWire_First(&OneWire1);
	while (onewire_devices)
	{
		onewire_count++;
		TM_OneWire_GetFullROM(&OneWire1, onewire_code[onewire_count - 1]);         	// Get 1-wire device codes/addresses
		onewire_devices = TM_OneWire_Next(&OneWire1);
	}
	PRINTI("1Wire devices number: %lu\r\n", onewire_count);
	for (i = 0; i < onewire_count; i++) 
		TM_DS18B20_SetResolution(&OneWire1, onewire_code[i], TM_DS18B20_Resolution_12bits);

	if (!supply24V)	// missing 24 V supply
		SETPIN(LEDR_GPIO_Port, LEDR_Pin);
	if (onewire_count > 0)
		SETPIN(LEDB_GPIO_Port, LEDB_Pin);
	HAL_Delay(500);
	CLRPIN(LEDR_GPIO_Port, LEDR_Pin | LEDB_Pin);

	// Initialization before Main Loop
	HAL_ADC_Start_DMA(&hadc1, adcbuf, ADC_BUF_LEN);

	// MODBUS SmartMeter
	PRINTI("MODBUS SmartMeter: ");
	rtv = SMReadVal(MB_METER_REG_VOLT, &SMvoltage);
	if (rtv == 2)
		SMpresent = true;
	else if (rtv == -1)
		{ PRINTI("has not connected or responded!\r\n"); }
	else
		{ PRINTI("has responded with error %d, slave error %d!\r\n", -rtv, mberrslave); }
	if (SMpresent)
	{
		rtv = SMReadVal(MB_METER_REG_FREQ, &SMfrequency);
		if (rtv == 2)
			{ PRINTI("Voltage = %5.1f V, Frequency = %5.2f Hz\r\n", SMvoltage, SMfrequency); }
	}	
	
	ts[TS_MB].period = 2000;      			// MODBUS query (SmartMeter)
	/*	// MODBUS SmartMeter
	PRINTI("MODBUS SmartMeter:");
	MBSendQuery(MB_METER_ADDR, MB_FUNC_READ, MB_METER_REG_SN, 4, NULL);
	HAL_Delay(150);  	// at least 110 ms
	if(rx6_EOF_rcv)	// RS485	
	{
		rx6_EOF_rcv = false;
		rtv = MBProcResponse(mbdata, MB_DATA_SIZE);			
		// printf("MODBUS Response Status: %i\r\n", rtv);
		if(rtv == 2)
		{
			SMpresent = true;
#ifdef	DEBUG_PRINT
			PRINTI("\r\nSerial No. ");
			for (i = 0; i < 8; i++)
				PRINTI("%01u", mbdata[i]);   	// BCD Serial Number in mbdata
			PRINTI("\r\n");
#endif
		}
	}
#ifdef	DEBUG_PRINT
	if (SMpresent)
	{
		rtv = SMReadVal(MB_METER_REG_VOLT, &SMvoltage);
		if (rtv > 0)
			PRINTI("Voltage:   %5.1f V\r\n", SMvoltage);
		rtv = SMReadVal(MB_METER_REG_FREQ, &SMfrequency);
		if (rtv > 0)
			PRINTI("Frequency: %5.2f Hz\r\n", SMfrequency);
				
	//	rtv = SMReadVal(MB_METER_REG_CURRENT, &SMcurrent);
	//	if (rtv > 0)
	//		PRINTI("Current:   %5.2f A\r\n", SMcurrent);
	//	rtv = SMReadVal(MB_METER_REG_POWER, &SMpower);
	//	if (rtv > 0)
	//		PRINTI("Power:    %6.3f kW\r\n", SMpower);
		
	}	
	if (!SMpresent)
		PRINTI(" has not responded or with an error\r\n");
#endif
*/
#ifdef GSM_MQTT	// Powering Quectel
	rtv = Test_AT_send(500);  	// was 10000 but this is a test only of turned on module !
	if(rtv != 0)  // Pulse for powering Quectel module
	{
		SETPIN(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin);
		SETPIN(LEDB_GPIO_Port, LEDB_Pin);
		HAL_Delay(500);
		CLRPIN(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin);
		CLRPIN(LEDB_GPIO_Port, LEDB_Pin);
		HAL_Delay(5000);
	}	
	Init_quectel();	 
	Mqtt_connect(ClientID); 	// missing real return value (always TRUE for now)
	HAL_Delay(200);
	for (i = 0; i < 4; i++)
	{
		Mqtt_subscribe(mqtt[N_MQTT_OUT + i].topic);
	}
#endif 			

	// set Timing Scheduler
	TimingSchedulerInit(ts);
	ts[TS_ADC].period = 1000;     		// ADC
	ts[TS_1WIRE].period = 3000;  		// 1-wire
	ts[TS_MB].period = 2000;   			// MODBUS query (SmartMeter)
	ts[TS_SIG].period = 1500;  			// LED signalization
	ts[TS_FLASH].enabled = false; 		// DISABLED - overwrite common initialization
#ifdef ADI_IOL_OUT
	ts[TS_IOL_OUT].period = 100;  		// IO-Link direct output test by DigIn levels
#else
	ts[TS_IOL_OUT].enabled = false; 	// DISABLED - overwrite common initialization
#endif	
	ts[TS_BUZ].enabled = false;  		// DISABLED - overwrite common initialization
	// Extra settings
	
	// Once run area
	/*
	rtv = RFM_Read1Reg(0x33);
	HAL_Delay(3);
	rtv = RFM_Write1Reg(0x33,0x11);
	HAL_Delay(3);
	rtv = RFM_Read1Reg(0x33);
	*/
	TimingOneShot(ts, TS_AUX, 10000);  	// for Main Loop duration measurement

//........................................................	
// MAIN LOOP - average duration 4.2 us (only ADC and 1 1-Wire device working, with prints)
while(true)
	{
		imainloop++;	
		TimingScheduler(ts);

		if (TST(TS_AUX))	// check the number of loops after 10 s
			{
				TSC(TS_AUX);
				printf("iLoopMain = %lu\r\n", imainloop);
			}

#ifdef GSM_MQTT
		if (reconnect_flag)
		{
			rtv = close_socket();
			rtv = Configure_TCP();
			rtv = Open_Port();
			Mqtt_connect(ClientID);
			for (i = 0; i < 4; i++)
			{
				Mqtt_subscribe(mqtt[N_MQTT_OUT + i].topic);
			}	
			reconnect_flag = false;
		}	
#endif	
	
#ifdef GSM_MQTT
		if (rx1_EOF_rcv)	// from COM
			{
				rx1_EOF_rcv = false;
				COMReadStr(main_rx_buf, MAIN_RX_BUF_LEN);
				readURC();
			}
#else
		if (rx1_EOF_rcv)	// DEBUG from COM
			{
				rx1_EOF_rcv = false;
				gets(in1str);
				// puts(in1str);
				npar = ParseMessage(in1str);
				if (npar < 0)
					error = ERR_CMD_EMPTY;
				else
					error = ProcMessage();
			}
#endif

		// Console command or value from Agilent DMM (TX still as STDOUT)
		if(rx2_EOF_rcv)
		{
			rx2_EOF_rcv = false;
			gets(in2str);
#ifdef RS232_DMM	// DMM
			if (dip1)	// console
				{
					npar = ParseMessage(in2str);
					if (npar < 0)
						error = ERR_CMD_EMPTY;
					else
						error = ProcMessage();
				}
			else			// DMM
				{
					mqtt[7].value = (float)atof(in2str);
					sprintf(mqtt[7].message, mqtt[7].format, mqtt[7].value);
					PRINTV("DMM value: %6.2f\r\n", mqtt[7].value);
#ifdef GSM_MQTT
					rtv = Mqtt_publish(7);
					if (rtv < 0)
						PRINTV("MQTT Publish Error No. %d, Topic 7\r\n", -rtv);
#endif				}
#else
			// puts(in2str);
			// Commands from console
			npar = ParseMessage(in2str);
			if (npar < 0)
				error = ERR_CMD_EMPTY;
			else
				error = ProcMessage();
#endif		}

		/*	
		// IO-Link serial DEBUG (CQ1 & CQ2 connected together)
		if (rx4_EOF_rcv)	
			{
			rx4_EOF_rcv = false;
			IOL2ReadStr(in2str, INSTR_LEN);
			puts("IOL2: ");
			puts(in2str);
			}

		if (rx5_EOF_rcv)	
			{
			rx5_EOF_rcv = false;
			IOL1ReadStr(in1str, INSTR_LEN);
			puts("IOL1: ");
			puts(in1str);
			}
		*/
	
		// Query for SmartMeter over RS485/MODBUS
	if(SMpresent && smquery == 0 && TST(TS_MB))
		{
			TSC(TS_MB);
			smquery = 1;
			MBSendQuery(MB_METER_ADDR, MB_FUNC_READ, MB_METER_REG_VOLT, 2, NULL);
		}


		// RS485/MODBUS
		if(rx6_EOF_rcv)
		{
			rx6_EOF_rcv = false;
			rtv = MBProcResponse(mbdata, MB_DATA_SIZE);
			if (rtv == 2)
			{
				n = __REV(*(uint32_t*)mbdata);       	// Endian swap
				switch(smquery)
				{
				case 1:
					SMvoltage = *((float*)&n);
					MBSendQuery(MB_METER_ADDR, MB_FUNC_READ, MB_METER_REG_POWER, 2, NULL);
					smquery = 2;
					break;
				case 2:
					SMpower = *((float*)&n);
					MBSendQuery(MB_METER_ADDR, MB_FUNC_READ, MB_METER_REG_PFACTOR, 2, NULL);
					smquery = 3;
					break;
				case 3:
					SMpfactor = *((float*)&n);
					PRINTV("SmartMeter: Voltage = %5.1f V, Power = %6.3f kW, Power Factor = %5.3f\r\n", SMvoltage, SMpower, SMpfactor);
					smquery = 0;
					
					// Insert conversion of data into the structure and MQTT publishing here
					
					break;
				}
			}
			else
			{
				PRINTV("RS485/SmartMeter has responded with error No. %u\r\n", mberrslave);
				smquery = 0;
			}
		}



		if (TST(TS_SIG))		// Blue LED flashing
			{
				TSC(TS_SIG);
				SETPIN(LEDB_GPIO_Port, LEDB_Pin);
				if (buzzer)
					IOL_DigOut(4, 1);
				TimingOneShot(ts, TS_FLASH, 100);
			}
		if (TST(TS_FLASH))
		{
			TSC(TS_FLASH);
			CLRPIN(LEDB_GPIO_Port, LEDB_Pin);
			if (buzzer)
				IOL_DigOut(4, 0);
		}

#ifdef ADI_IOL_OUT	// periodical IOL output test by DigIn
		if (dip1 && TST(TS_IOL_OUT))
		{
			TSC(TS_IOL_OUT);
			IOL_DigOut(1, (uint32_t)diginp[0]); 		// DO1 CQ3
			IOL_DigOut(2, (uint32_t)diginp[1]);  	// DO2 CQ4
			IOL_DigOut(3, (uint32_t)diginp[2]); 		// DO3 L+3
			IOL_DigOut(4, (uint32_t)diginp[3]);  	// DO4 L+4
			// IOL_DigOut(5, DO_TOGGLE);	// IOL1 L+1
			// IOL_DigOut(6, DO_TOGGLE);	// IOL2 L+2
		}
#endif
	
		// ADC value operation
		if(TST(TS_ADC) && adcnewval)
		{
			TSC(TS_ADC);
			adcnewval = false;
			calib0 = (3.300 / 4095.0) * (float)vrefcal / (float)adcval[N_ADC_CHAN - 1]; 
			voltcoef = (1.0 / ADI_DIV_RATIO) * calib0;
			currcoef = (1.0 / ADI_CURR_RATIO) * calib0;
			// oversampling and input divider included; ADCVAL last channel is REF voltage
		PRINTV("ADI AVG values (log. levels):");
			for (i = 0; i < N_ADI_CHAN; i++)	// input channels only
				{
					j = 3 + i;  	// as in the structure
					ADIval[i] = (inpmode[i] ? currcoef : voltcoef) * (float)adcval[i];      	// [V]
					if(i < 2)
						mqtt[j].value = ADIval[i];
					else
						mqtt[j].value = (float)diginp[i];
					sprintf(mqtt[j].message, mqtt[j].format, mqtt[j].value);
#ifdef GSM_MQTT
					rtv = Mqtt_publish(j);
					if (rtv < 0)
						PRINTV("MQTT Publish Error No. %d, Topic %u\r\n", -rtv, j);			
#endif					PRINTV("  %6.3f %s (%c)", ADIval[i], (inpmode[i] ? "mA" : "V "), (diginp[i]) ? 'H' : 'L');
				}
			PRINTV("\r\n");
		}
		
		// Start temperature conversion on all devices on one 1-wire bus
		if(TST(TS_1WIRE) && onewire_count > 0)
		{
			TSC(TS_1WIRE);
			while (!TM_DS18B20_AllDone(&OneWire1)) ;	// Wait until all are done on one onewire port
			PRINTV("1-Wire temperatures:");	
			for (i = 0; i < onewire_count; i++) 		// Read temperature from each device separatelly
				{
					// Read temperature from ROM address and store it to temps variable
					if(TM_DS18B20_Read(&OneWire1, onewire_code[i], &DS12B20_th[i]))
					{
						PRINTV("  %6.2f", DS12B20_th[i]);	
						mqtt[i].value = DS12B20_th[i];
						sprintf(mqtt[i].message, mqtt[i].format, mqtt[i].value);
#ifdef GSM_MQTT				
						rtv = Mqtt_publish(i);
						if (rtv < 0)
							PRINTV("MQTT Publish Error No. %d, Topic %d\r\n", -rtv, i);
#endif				
					}
				}
			PRINTV("  degC\r\n");
			TM_DS18B20_StartAll(&OneWire1); 	// Start measuring on all devices
		}

		// HAL_Delay(1000);
	}
}

//------------------------------------------------------------------------------
static int ParseMessage(char *istr)
{
	char *sp;
	inpar1 = inpar2 = inpar3 = 0;
	sp = istr;
	while (*sp == ' ')	sp++;
	strcpy(cmdstr, sp);
	sp = strtok(cmdstr, delim);
	if (sp == NULL)
		return -1;
	sp = strtok(NULL, delim);
	if (sp == NULL)
		return 0;   // command only
	else
		// inpar1 = atoi(sp);
		inpar1 = (int)strtol(sp, NULL, 0);
	sp = strtok(NULL, delim);
	if (sp == NULL)
		return 1;   // only 1 parameter
	else
		// inpar2 = atoi(sp);
		inpar2 = (int)strtol(sp, NULL, 0);
	sp = strtok(NULL, delim);
	if (sp == NULL)
		return 3;
	else
		// inpar3 = atoi(sp);
		inpar3 = (int)strtol(sp, NULL, 0);
	return 4;      // 3 parametry
}

//------------------------------------------------------------------------------
static int ProcMessage(void)
{
	int rtv, msgerror = 0;
	char *sp, c1 = toupper(cmdstr[0]), c2 = toupper(cmdstr[1]);

	switch (c1)
	{
	case 'L':		// LoRa RFM85 register test
		{
			switch (c2)
			{
			case 'W':	// Write register
				if(npar > 1)
				{
					if (inpar1 >= 0 && inpar1 < 0x64)
					{
						rtv = (int)RFM_Write1Reg((uint8_t)inpar1, (uint8_t)inpar2);
						printf("RFM register #0x%02X written %02X, returned %02X\r\n", inpar1, inpar2, rtv);
					}
					else
						msgerror = ERR_WRONG_PAR;
				}
				else
					msgerror = ERR_FEW_PAR;
				break;

			case 'R':	// Read byte
				if(npar > 0)
				{
					if (inpar1 >= 0 && inpar1 < 0x64)
					{
						rtv = (int)RFM_Read1Reg((uint8_t)inpar1);
						printf("RFM register #0x%02X = 0x%02X\r\n", inpar1, rtv);
					}
					else
						msgerror = ERR_WRONG_PAR;
				}
				else
					msgerror = ERR_FEW_PAR;
				break;

			default:
				msgerror = ERR_CMD_UNKNOWN;
			}
		}
		break;
	
	case 'S':
		if (npar > 1)
		{
			if (inpar1 >= 1 && inpar1 <= 6)
			{
				if (inpar2 >= 0 && inpar2 <= 2)
					IOL_DigOut(inpar1, inpar2);	
				else
					msgerror = ERR_WRONG_PAR;
			}
			else
				msgerror = ERR_WRONG_PAR;
			if (inpar1 == 7)
				buzzer = (inpar2 == 1);
		}
		else
			msgerror = ERR_FEW_PAR;
		break;

	case '*':
		printf("\r\n: %s\r\n", cmdstr + 2);
		break;

	}
	return msgerror;
}

// RFM95 functions
//------------------------------------------------------------------------------
uint8_t RFM_Read1Reg(uint8_t regno)
{
	uint8_t rtv;
	regno &= 0x7F; 	// read
	CS_COM_ACTIVE;
	SPI3_SendByte(regno);
	rtv = SPI3_SendByte(0x00);
	CS_COM_IDLE;
	return rtv;
}

//------------------------------------------------------------------------------
uint8_t RFM_Write1Reg(uint8_t regno, uint8_t data)
{
	uint8_t rtv;
	regno |= 0x80;  	// write
	CS_COM_ACTIVE;
	rtv = SPI3_SendByte(regno);
	SPI3_SendByte(data); 	// previous content of the register
	CS_COM_IDLE;
	return rtv;
}

//------------------------------------------------------------------------------
uint8_t SPI3_SendByte(uint8_t data)
{
	while (!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE)) ;
	hspi3.Instance->DR = data;
	while (!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_RXNE)) ;
	return hspi3.Instance->DR;
}

/*
//------------------------------------------------------------------------------
void MQTTDigOut(void)
{
for (int i=0; i < N_MQTT_IN; i++)
	IOL_DigOut(i+1, (uint32_t)mqtt[8+i].value);
}
*/

//------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	uint32_t i;
	for (i = 0; i < N_ADC_CHAN; i++)
		adcsum[i] += adcbuf[i];
	if (++iadcavg == N_ADC_AVG)
	{
		iadcavg = 0;
		for (i = 0; i < N_ADC_CHAN; i++)
		{
			adcval[i] = adcsum[i] / N_AVG_OVERSAMP;
			adcsum[i] = 0;
			diginp[i] = (adcval[i] > ADI_THR_VAL);
		}
		adcnewval = true;
	}
}

// *****************************************************************************


