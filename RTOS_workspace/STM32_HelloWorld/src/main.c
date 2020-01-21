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

			
void vTask1_handler(void *param );
void vTask2_handler(void *param );

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
#ifdef USE_SEMIHOSTING
	extern void initialise_monitor_handles();
#endif


#define TRUE 1
#define FALSE 0
#define AVAILABLE TRUE
#define NOT_AVAILABLE FALSE

static void prvSetupHardware(void);

static void prvSetupUART2(void);
void printmsgUART2(char *msg);

static void prvSetupUART3(void);
void printmsgUART3(char *msg);

char usr_msg[250];


uint8_t UART_ACCESS_KEY = AVAILABLE;

int main(void)
{

#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("This is hello world example code\n");
#endif

	DWT->CTRL |= (1 << 0);// ENABLE the CYCCNT in DWT_CTRL (Time couner for the tracking of the system view)5

	//1) HSI ON, PLL OFF, HSE OFF, System clock = 16MHz, CPU clock 16MHz
	RCC_DeInit();

	//2) Update the system clock variable(to 16MHz)
	SystemCoreClockUpdate();

	prvSetupHardware();
	sprintf(usr_msg,"This is hellow world application starting\r\n");
	printmsgUART2(usr_msg);
	//while(1)
	//printmsgUART3(usr_msg);

	//Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

    //3) Create 2 tasks
	 xTaskCreate(vTask1_handler,//Task function
			 	 "Task-1",      //Task name
				 configMINIMAL_STACK_SIZE, //Task stack size
				 NULL,          // Task function argument to transfer
				 2,             // Priority
				 &xTaskHandle1);// Tasks handle

	 xTaskCreate(vTask2_handler,//Task function
			 	 "Task-2",//Task name
				 configMINIMAL_STACK_SIZE,//Task stack size
				 NULL,	// Task function argument to transfer
				 2,  	// Priority
				 &xTaskHandle2);// Tasks handle

	 //4) Starts the scheduler
	 vTaskStartScheduler();


	for(;;);
}


void vTask1_handler(void *param )
{
	while(1)
	{
		if(UART_ACCESS_KEY == AVAILABLE)
		{
			UART_ACCESS_KEY = NOT_AVAILABLE;
			printmsgUART2("This is hello world from TASK1\r\n");
			UART_ACCESS_KEY = AVAILABLE;
			taskYIELD();
		}
	}
}

void vTask2_handler(void *param )
{
	while(1)
	{
		if(UART_ACCESS_KEY == AVAILABLE)
		{
			UART_ACCESS_KEY = NOT_AVAILABLE;
			printmsgUART2("This is hello world from TASK2\r\n");
			UART_ACCESS_KEY = AVAILABLE;
			taskYIELD();
		}
	}
}
static void prvSetupHardware(void)
{

	prvSetupUART2();
	//prvSetupUART3();

}

static void prvSetupUART3(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef huart3_init;

	//1) Enable the UART2 Peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//PA2 is UART2_TX , PA3 is UART2_RX

	//2) Alternate function to work as UART2
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));
	gpio_uart_pins.GPIO_Pin =GPIO_Pin_10;
	gpio_uart_pins.GPIO_Mode =GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&gpio_uart_pins);

	//3) AF mode settings for the pins

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);// PA2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);// PA3

	//4) UART parameter initializations
	memset(&huart3_init,0,sizeof(huart3_init));
	huart3_init.USART_BaudRate = 115200;
	huart3_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	huart3_init.USART_Mode = USART_Mode_Tx;
	huart3_init.USART_Parity = USART_Parity_No;
	huart3_init.USART_StopBits = USART_StopBits_1;
	huart3_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3,&huart3_init);

	USART_Cmd(USART3,ENABLE);


}
void printmsgUART3(char *msg)
{

	for(int i= 0 ;i<strlen(msg); i++)
	{
		//while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) != SET); // if SET means the UART TX is busy
		USART_SendData(USART3,msg[i]);
	}

}

static void prvSetupUART2(void)
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

void printmsgUART2(char *msg)
{

	for(int i= 0 ;i<strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET); // if SET means the UART TX is busy
		USART_SendData(USART2,msg[i]);
	}

}




