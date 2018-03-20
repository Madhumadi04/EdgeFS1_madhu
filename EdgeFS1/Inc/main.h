/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define AUX3IN_Pin GPIO_PIN_13
#define AUX3IN_GPIO_Port GPIOC
#define LEDR_Pin GPIO_PIN_0
#define LEDR_GPIO_Port GPIOC
#define LEDB_Pin GPIO_PIN_1
#define LEDB_GPIO_Port GPIOC
#define DIP0_Pin GPIO_PIN_2
#define DIP0_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_3
#define DIP1_GPIO_Port GPIOC
#define CTS2_RS232_Pin GPIO_PIN_0
#define CTS2_RS232_GPIO_Port GPIOA
#define RTS2_RS232_Pin GPIO_PIN_1
#define RTS2_RS232_GPIO_Port GPIOA
#define TX2_RS232_Pin GPIO_PIN_2
#define TX2_RS232_GPIO_Port GPIOA
#define RX2_RS232_Pin GPIO_PIN_3
#define RX2_RS232_GPIO_Port GPIOA
#define DIGOUT1_Pin GPIO_PIN_4
#define DIGOUT1_GPIO_Port GPIOA
#define DIGOUT2_Pin GPIO_PIN_5
#define DIGOUT2_GPIO_Port GPIOA
#define ADC1_IN6_ADI1_Pin GPIO_PIN_6
#define ADC1_IN6_ADI1_GPIO_Port GPIOA
#define ADC1_IN7_ADI2_Pin GPIO_PIN_7
#define ADC1_IN7_ADI2_GPIO_Port GPIOA
#define ADC1_IN14_ADI3_Pin GPIO_PIN_4
#define ADC1_IN14_ADI3_GPIO_Port GPIOC
#define ADC1_IN15_ADI4_Pin GPIO_PIN_5
#define ADC1_IN15_ADI4_GPIO_Port GPIOC
#define ADC1_IN8_ADI5_AUX1_Pin GPIO_PIN_0
#define ADC1_IN8_ADI5_AUX1_GPIO_Port GPIOB
#define ADC1_IN9_ADI6_AUX2_Pin GPIO_PIN_1
#define ADC1_IN9_ADI6_AUX2_GPIO_Port GPIOB
#define IOLINK_INTR_Pin GPIO_PIN_2
#define IOLINK_INTR_GPIO_Port GPIOB
#define IOLINK1_TXE_Pin GPIO_PIN_10
#define IOLINK1_TXE_GPIO_Port GPIOB
#define IOLINK2_TXE_Pin GPIO_PIN_11
#define IOLINK2_TXE_GPIO_Port GPIOB
#define CS_IOLINK_Pin GPIO_PIN_12
#define CS_IOLINK_GPIO_Port GPIOB
#define SCK2_IOLINK_Pin GPIO_PIN_13
#define SCK2_IOLINK_GPIO_Port GPIOB
#define MIS02_IOLINK_Pin GPIO_PIN_14
#define MIS02_IOLINK_GPIO_Port GPIOB
#define MOSI2_IOLINK_Pin GPIO_PIN_15
#define MOSI2_IOLINK_GPIO_Port GPIOB
#define TX6_RS485_Pin GPIO_PIN_6
#define TX6_RS485_GPIO_Port GPIOC
#define RX6_RS485_Pin GPIO_PIN_7
#define RX6_RS485_GPIO_Port GPIOC
#define RS485_TXE_Pin GPIO_PIN_8
#define RS485_TXE_GPIO_Port GPIOC
#define GSM_RESET_Pin GPIO_PIN_9
#define GSM_RESET_GPIO_Port GPIOC
#define GSM_PWRKEY_Pin GPIO_PIN_8
#define GSM_PWRKEY_GPIO_Port GPIOA
#define TX1_GSM_Pin GPIO_PIN_9
#define TX1_GSM_GPIO_Port GPIOA
#define RX1_GSM_Pin GPIO_PIN_10
#define RX1_GSM_GPIO_Port GPIOA
#define CTS1_GSM_Pin GPIO_PIN_11
#define CTS1_GSM_GPIO_Port GPIOA
#define RTS1_GSM_Pin GPIO_PIN_12
#define RTS1_GSM_GPIO_Port GPIOA
#define CS1_COM_Pin GPIO_PIN_15
#define CS1_COM_GPIO_Port GPIOA
#define TX4_IOLINK2_Pin GPIO_PIN_10
#define TX4_IOLINK2_GPIO_Port GPIOC
#define RX4_IOLINK2_Pin GPIO_PIN_11
#define RX4_IOLINK2_GPIO_Port GPIOC
#define TX5_IOLINK1_Pin GPIO_PIN_12
#define TX5_IOLINK1_GPIO_Port GPIOC
#define RX5_IOLINK1_Pin GPIO_PIN_2
#define RX5_IOLINK1_GPIO_Port GPIOD
#define SCK3_COM_Pin GPIO_PIN_3
#define SCK3_COM_GPIO_Port GPIOB
#define MISO3_COM_Pin GPIO_PIN_4
#define MISO3_COM_GPIO_Port GPIOB
#define MOSI3_COM_Pin GPIO_PIN_5
#define MOSI3_COM_GPIO_Port GPIOB
#define GSM_INTR_Pin GPIO_PIN_6
#define GSM_INTR_GPIO_Port GPIOB
#define ONEWIRE_Pin GPIO_PIN_7
#define ONEWIRE_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
