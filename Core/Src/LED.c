#include "LED.h"
#include "stm32l1xx_hal.h"

STATUS_T LED_On(uint16_t led)
{
	HAL_GPIO_WritePin(GPIOC,led,GPIO_PIN_SET);

	return STATUS_SET;
}

STATUS_T LED_Off(uint16_t led)
{
	HAL_GPIO_WritePin(GPIOC,led,GPIO_PIN_RESET);

	return STATUS_RESET;
}

STATUS_T LED_Init(void)
{
	LED_Off(LED_1);
	LED_Off(LED_2);

	return STATUS_OK;
}

STATUS_T LED_ToggleTwoLEDs(void)
{
	if(HAL_GPIO_ReadPin(GPIOC,LED_1)==GPIO_PIN_SET){
		LED_Off(LED_1);
		LED_On(LED_2);
	}else{
		LED_On(LED_1);
		LED_Off(LED_2);
	}

	return STATUS_OK;
}

STATUS_T LED_GetStatus(uint16_t led)
{
	if(HAL_GPIO_ReadPin(GPIOC,led)==GPIO_PIN_SET)
		return STATUS_SET;
	else
		return STATUS_RESET;
}

void Power_On(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Power on of SIM808 shield module
}
