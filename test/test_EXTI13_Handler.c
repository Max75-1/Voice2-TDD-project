/*#include "unity.h"
#include "mock_FreeRTOS.h"
#include "mock_task.h"
#include "mock_stm32l1xx_hal_tim.h"
#include "mock_stm32l1xx_hal_gpio.h"
#include "mock_LED.h"
#include "stm32l1xx_it.h"
#include "Utils.h"

extern GPIO_PinState ButtonState;
extern int Timer1;
extern TaskHandle_t xHandle_LED;
extern BaseType_t checkIfYieldRequired;

void setUp(void)
{
}

void tearDown(void)
{
}*/

/*void test_EXTI13_Handler_should_GetButtonState(void)
{
	Timer1=30;
	HAL_GPIO_ReadPin_ExpectAnyArgsAndReturn(GPIO_PIN_SET);

	EXTI15_10_IRQHandler();

	TEST_ASSERT_EQUAL_INT(GPIO_PIN_SET,ButtonState);
	TEST_ASSERT_EQUAL_INT(0,Timer1);
}*/
