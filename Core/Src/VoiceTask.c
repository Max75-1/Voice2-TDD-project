#include "VoiceTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#ifndef TEST
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t UART1_RxBuf[40];
extern uint8_t MainBuf[40];
#else
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
uint8_t UART1_RxBuf[40];
uint8_t MainBuf[40];
#endif

void prvVoiceTask   ( void *pvParameters )
{

	vTaskSuspend(NULL); // HAL_UARTEx_RxEventCallback

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT\r\n",6,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATZ0\r\n",7,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		/*memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+DDET=0\r\n",12,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);*/

		/*memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATT\r\n",6,1000);
		HAL_UART_Receive(&huart1,MainBuf,40,1000);
		HAL_UART_Transmit(&huart2,MainBuf,40,1000);*/

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+COLP=0\r\n",12,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,1000);
		HAL_UART_Transmit(&huart2,MainBuf,40,1000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATD0548385644;\r\n",17,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		//HAL_Delay(6000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+DTAM=2\r\n",12,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=1,1,0,600,0,0\r\n",24,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		/*memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=4,1,0,50,1\r\n",21,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);*/

		vTaskSuspend(NULL);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=2\r\n",12,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATH\r\n",6,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=4,1,0,80,1\r\n",21,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		HAL_Delay(10000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=5\r\n",12,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		/*memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATH\r\n",6,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);*/

		vTaskSuspend(NULL);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT\r\n",5,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		/*memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATT\r\n",6,1000);
		HAL_UART_Receive(&huart1,MainBuf,40,1000);
		HAL_UART_Transmit(&huart2,MainBuf,40,1000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+COLP=1\r\n",12,1000);
		HAL_UART_Receive(&huart1,MainBuf,40,1000);
		HAL_UART_Transmit(&huart2,MainBuf,40,1000);*/

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATD0548385644;\r\n",17,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		HAL_Delay(10000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+DTAM=1\r\n",12,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CREC=4,1,0,100,1\r\n",21,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		HAL_Delay(10000);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"ATH\r\n",6,2000);
		HAL_UART_Receive(&huart1,MainBuf,40,2000);
		HAL_UART_Transmit(&huart2,MainBuf,40,2000);

		vTaskSuspend(NULL);

		/*memset(MainBuf,'\0',39);
		HAL_UART_Transmit(&huart1,"AT\r\n",5,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);

		memset(MainBuf,'\0',39);
		HAL_UART_Transmit(&huart1,"AT+CSCS=\"GSM\"\r\n",16,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);

		memset(MainBuf,'\0',39);
		HAL_UART_Transmit(&huart1,"AT+CMGF=1\r\n",12,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"AT+CMGS=\"0548385644\"\r\n",23,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);

		memset(MainBuf,'\0',40);
		HAL_UART_Transmit(&huart1,"Alarm !!! Priza from STM32 !!!\r\n",33,100);
		HAL_UART_Receive(&huart1,MainBuf,40,100);
		HAL_UART_Transmit(&huart2,MainBuf,40,100);*/

		/*while (!(huart1.Instance->SR & 0x0080));
		huart1.Instance->SR & 0x0000;
		huart1.Instance->DR = 0x1A;    //sending CtrlZ command*/

		//vTaskSuspend(NULL);

		//vTaskSuspend(NULL);

	while(1){

  	}
}

