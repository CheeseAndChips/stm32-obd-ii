#include <stdio.h>
#include <string.h>

#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"

UART_HandleTypeDef h_uart3;

static void uart_init(void);
static void error_handler(void);
static void SystemClock_Config(void);
static void wait_for_input(void);

int main(void)
{
	SCB_EnableICache();
	SCB_EnableDCache();
	HAL_Init();
	SystemClock_Config();
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	uart_init();

	while(1) {
		printf("Waiting for some input...\n");
		wait_for_input();
	}

	return 0;
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&h_uart3, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}

static void wait_for_input(void) {
	uint8_t buffer[16];
	do {
		memset(buffer, 0, sizeof(buffer));
		HAL_UART_Receive(&h_uart3, buffer, sizeof(buffer), 100);
	} while(!buffer[0]);
}

static void uart_init(void) {
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef gpio;

	gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_HIGH;
	gpio.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &gpio);

	h_uart3.Instance = USART3;
	h_uart3.Init.BaudRate = 115200;
	h_uart3.Init.WordLength = UART_WORDLENGTH_8B;
	h_uart3.Init.StopBits = UART_STOPBITS_1;
	h_uart3.Init.Parity = UART_PARITY_NONE;
	h_uart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	h_uart3.Init.Mode = UART_MODE_TX_RX;
	h_uart3.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&h_uart3) != HAL_OK) {
		error_handler();
	}

	//HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
	//HAL_NVIC_EnableIRQ(USART3_IRQn);
	//__HAL_UART_ENABLE_IT(&h_uart3, UART_IT_RXNE);
}

static void SystemClock_Config(void) {
	/*
	System Clock source            = PLL (HSE)
	SYSCLK(Hz)                     = 216000000
	HCLK(Hz)                       = 216000000
	HSE Frequency(Hz)              = 8000000
	VDD(V)                         = 3.3
	Main regulator output voltage  = Scale1 mode
	*/

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	// enable HSE Oscillator and activate PLL with HSE as source
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	RCC_OscInitStruct.PLL.PLLR = 7;

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		while(1) {};
	}

	// activate the OverDrive to reach the 216 Mhz Frequency
	if(HAL_PWREx_EnableOverDrive() != HAL_OK) {
		while(1) {};
	}

	// select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		while(1) {};
	}
}

static void error_handler(void) {
	while(1) {
		BSP_LED_Toggle(LED3);
		HAL_Delay(1000);
	}
}
