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

	while(1){

		/*HAL_UART_Transmit(&huart1,"AT\r\n",5,10);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART1_RxBuf,40);
		HAL_UART_Transmit(&huart2,MainBuf,40,10);

		HAL_UART_Transmit(&huart1,"AT+CSCS=\"GSM\"\r\n",16,10);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART1_RxBuf,40);
		HAL_UART_Transmit(&huart2,MainBuf,40,10);

		HAL_UART_Transmit(&huart1,"AT+CMGF=1\r\n",12,10);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART1_RxBuf,40);
		HAL_UART_Transmit(&huart2,MainBuf,40,10);

		HAL_UART_Transmit(&huart1,"AT+CMGS=\"0548385644\"\r\n",23,10);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART1_RxBuf,40);
		HAL_UART_Transmit(&huart2,MainBuf,40,10);

		HAL_UART_Transmit(&huart1,"Alarm !!! Priza from STM32 !!!\r\n",33,10);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,UART1_RxBuf,40);
		HAL_UART_Transmit(&huart2,MainBuf,40,10);

		while (!(huart1.Instance->SR & 0x0080));
		huart1.Instance->SR & 0x0000;
		huart1.Instance->DR = 0x1A;    //sending CtrlZ command

		vTaskSuspend(NULL);

		vTaskSuspend(NULL);*/
  	}
}

