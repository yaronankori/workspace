/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define FALSE 0
#define TRUE  1
#define NOT_PRESSED FALSE
#define PRESSED TRUE

//Function prototype
static void prvSetupUART(void);
static void prvSetupHardware(void);
void printmsg(char *msg);

// task prototypes
void led_task_handler(void *params);
void button_task_handler(void *params);
void prvSetupGpio(void);
void rtos_delay(uint32_t delay_in_ms);

uint8_t button_status_flag = NOT_PRESSED;
char usr_msg[250];
TaskHandle_t xTaskHandle1,xTaskHandle2;
int main(void)
{

	//1) HSI ON, PLL OFF, HSE OFF, System clock = 16MHz, CPU clock 16MHz
	RCC_DeInit();

	//2) Update the system clock variable(to 16MHz)
	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"This is hellow world application starting\r\n");
	printmsg(usr_msg);

	//create a led task
	xTaskCreate(led_task_handler,"LED_TASK",500,NULL,2,&xTaskHandle1);
	//create a button task
	xTaskCreate(button_task_handler,"BUTTON_TASK",500,NULL,2,&xTaskHandle2);

	//start the scheduler
	vTaskStartScheduler();


	for(;;);
}

void led_task_handler(void *params)
{
	uint32_t current_notification_value =0;

	while(1)
	{
		// lets wait until we receive any notification event from button

		if(xTaskNotifyWait(0,0,&current_notification_value,portMAX_DELAY) == pdTRUE)
		{
			GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
			sprintf(usr_msg,"Notification is received: Button press count : %ld\r\n",current_notification_value);
			printmsg(usr_msg);

		}
	}
}

void button_task_handler(void *params)
{
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13))
		{
			// Buton is not pressed on the nucleo board
			//button_status_flag = NOT_PRESSED;

		}
		else
		{
			// Buton is pressed on the nucleo board
			rtos_delay(100); //Debounce
			xTaskNotify(xTaskHandle1,0,eIncrement);
			//xTaskHandle1
		}
	}
}


void rtos_delay(uint32_t delay_in_ms)
{
	uint32_t current_tick_count = xTaskGetTickCount();

	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ)/1000; // ticks to [ms]

	while(xTaskGetTickCount() < current_tick_count + delay_in_ms);

}
static void prvSetupHardware(void)
{
	//Setup the Button and LED
	prvSetupGpio();

	//setup UART2
	prvSetupUART();

}
static void prvSetupUART(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef huart2_init;

	//1) Enable the UART2 Peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//PA2 is UART2_TX , PA3 is UART2_RX

	//2) Alternate function to work as UART2
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));
	gpio_uart_pins.GPIO_Pin =GPIO_Pin_2;
	gpio_uart_pins.GPIO_Mode =GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio_uart_pins);

	//3) AF mode settings for the pins

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);// PA2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);// PA3

	//4) UART parameter initializations
	memset(&huart2_init,0,sizeof(huart2_init));
	huart2_init.USART_BaudRate = 115200;
	huart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	huart2_init.USART_Mode = USART_Mode_Tx;
	huart2_init.USART_Parity = USART_Parity_No;
	huart2_init.USART_StopBits = USART_StopBits_1;
	huart2_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&huart2_init);

	USART_Cmd(USART2,ENABLE);


}

void prvSetupGpio(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitTypeDef led_init,button_init;

	led_init.GPIO_Mode =  GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_5;
	led_init.GPIO_PuPd = GPIO_Low_Speed;
	led_init.GPIO_Speed = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&led_init);

	button_init.GPIO_Mode =  GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_13;
	button_init.GPIO_PuPd = GPIO_Low_Speed;
	button_init.GPIO_Speed = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&button_init);
}

void printmsg(char *msg)
{

	for(int i= 0 ;i<strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET); // if SET means the UART TX is busy
		USART_SendData(USART2,msg[i]);
	}


}
