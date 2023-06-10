#include <stdio.h>
#include <string.h>

#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal.h"

UART_HandleTypeDef h_uart3;
CAN_HandleTypeDef h_can1;

static void uart_init(void);
static void error_handler(void);
static void SystemClock_Config(void);
static void wait_for_input(void);
static void can_init(void);

uint32_t TxMailbox;
uint32_t RxMailbox;

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
	can_init();

	HAL_CAN_Start(&h_can1);
	while(1) {
		CAN_TxHeaderTypeDef header;
		header.StdId = 0x69;
		header.IDE = CAN_ID_STD;
		header.RTR = CAN_RTR_DATA;
		header.DLC = 2;
		header.TransmitGlobalTime = DISABLE;

		uint8_t data[0x10];
		for(int i = 0; i < 0x10; i++) {
			data[i] = 0x10 | i;
		}
		//printf("RX fifo %u before\n", HAL_CAN_GetRxFifoFillLevel(&h_can1, RxMailbox));
		printf("VWaiting for input...\n");
		wait_for_input();
		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&h_can1, &header, data, &TxMailbox);
		printf("Pending: %u\n", HAL_CAN_IsTxMessagePending(&h_can1, TxMailbox));
		HAL_Delay(10);
		printf("Pending: %u\n", HAL_CAN_IsTxMessagePending(&h_can1, TxMailbox));
		if(status != HAL_OK) {
			printf("Failed sending\n");
			error_handler();
		}
		//printf("RX fifo %u after\n", HAL_CAN_GetRxFifoFillLevel(&h_can1, ));
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

static void can_init(void) {
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

	h_can1.Instance = CAN1;
	h_can1.Init.Prescaler = 8;
	h_can1.Init.Mode = CAN_MODE_NORMAL;
	h_can1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	h_can1.Init.TimeSeg1 = CAN_BS1_1TQ;
	h_can1.Init.TimeSeg2 = CAN_BS2_2TQ;
	h_can1.Init.TimeTriggeredMode = DISABLE;
	h_can1.Init.AutoBusOff = DISABLE;
	h_can1.Init.AutoWakeUp = DISABLE;
	h_can1.Init.AutoRetransmission = DISABLE;
	h_can1.Init.ReceiveFifoLocked = DISABLE;
	h_can1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&h_can1) != HAL_OK)
	{
		error_handler();
	} 

	CAN_FilterTypeDef config;
	config.FilterActivation = DISABLE;
	HAL_CAN_ConfigFilter(&h_can1, &config);
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
