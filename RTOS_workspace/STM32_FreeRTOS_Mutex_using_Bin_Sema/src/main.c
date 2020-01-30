/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-02-2020
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



#define FALSE 0
#define TRUE  1
#define NOT_PRESSED FALSE
#define PRESSED TRUE

//Function prototype
static void prvSetupUART(void);
static void prvSetupHardware(void);
void printmsg(char *msg);
void EmployeeDoWork(unsigned char TicketId);

// task prototypes
void vTask1(void *params);
void vTask2(void *params);

void prvSetupGpio(void);
void rtos_delay(uint32_t delay_in_ms);
void printmsg(char *msg);

char usr_msg[250];
uint8_t switch_prio = FALSE;
void button_handler(void *params);

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

#define USE_MUTEX
#ifdef USE_MUTEX
	xSemaphoreHandle xMutex;
#else
	xSemaphoreHandle xBinarySemaphore;
#endif


int main(void)
{

	DWT->CTRL |= (1 << 0);// ENABLE the CYCCNT in DWT_CTRL (Time couner for the tracking of the system view)

	//1) HSI ON, PLL OFF, HSE OFF, System clock = 16MHz, CPU clock 16MHz
	RCC_DeInit();

	//2) Update the system clock variable(to 16MHz)
	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"This is a Mutual exclusion using binary semaphore Demo\r\n");
	printmsg(usr_msg);


	#ifdef USE_MUTEX
		xMutex = xSemaphoreCreateMutex(); // Option2, using Mutex
	#else
		vSemaphoreCreateBinary(xBinarySemaphore); // create binary semaphore
	#endif

	//Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	#ifdef USE_MUTEX
		if(xMutex != NULL)
	#else
		if(xBinarySemaphore != NULL)
	#endif
	{
		//create Task1
		xTaskCreate(vTask1,"ManagerTask",500,NULL,1,&xTaskHandle1);
		//create Task2
		xTaskCreate(vTask2,"EmployeeTask",500,NULL,1,&xTaskHandle2);

		#ifdef USE_MUTEX
			xSemaphoreGive(xMutex); // First time give
		#else
			xSemaphoreGive(xBinarySemaphore); // First time give
		#endif

	}

	sprintf(usr_msg,"Queue or Sema create failed\r\n");
	printmsg(usr_msg);
	//start the scheduler
	vTaskStartScheduler();


	for(;;);
}


void vTask1(void *params)
{
   const char *pcTaskName = "Task 1 is running \r\n";

	while(1)
	{
		#ifdef USE_MUTEX
			xSemaphoreTake(xMutex,portMAX_DELAY);

			sprintf(usr_msg,"%s",pcTaskName);
			printmsg(usr_msg);

			xSemaphoreGive(xMutex);
		#else
			xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);

			sprintf(usr_msg,"%s",pcTaskName);
			printmsg(usr_msg);

			xSemaphoreGive(xBinarySemaphore);
		#endif


		vTaskDelay(500 / portTICK_RATE_MS);
	}
}
void vTask2(void *params)
{
	const char *pcTaskName = "Task 2 is running \r\n";

	while(1)
	{
		#ifdef USE_MUTEX
			xSemaphoreTake(xMutex,portMAX_DELAY);

			sprintf(usr_msg,"%s",pcTaskName);
			printmsg(usr_msg);

			xSemaphoreGive(xMutex);
		#else
			xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);

			sprintf(usr_msg,"%s",pcTaskName);
			printmsg(usr_msg);

			xSemaphoreGive(xBinarySemaphore);
		#endif

		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void EmployeeDoWork(unsigned char TicketId)
{
	sprintf(usr_msg,"Employee task: working on Ticket id: %d\r\n",TicketId);
	printmsg(usr_msg);
	vTaskDelay(TicketId);
}


void rtos_delay(uint32_t delay_in_ms)
{
	uint32_t current_tick_count = xTaskGetTickCount();

	//uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ)/1000; // ticks to [ms]

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

	button_init.GPIO_Mode =  GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_1;
	button_init.GPIO_PuPd = GPIO_Low_Speed;
	button_init.GPIO_Speed = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&button_init);

	//********CONFIG INTERRUPT PA1**************************
	//Interrupt configuration for the button (PA1)
	//Config the External interrupt controller, ST
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource1);

	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line = EXTI_Line1;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	//Config the Internal interrupt controller, ARM
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn,5); //0 in the highest
	//********************************************************
}

void printmsg(char *msg)
{
	for(int i= 0 ;i<strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET); // if SET means the UART TX is busy
		USART_SendData(USART2,msg[i]);
	}
}

void EXTI1_IRQHandler(void)
{
	traceISR_ENTER();
	EXTI_ClearITPendingBit(EXTI_Line1);
	button_handler(NULL);
	traceISR_EXIT();
	// Random code
}
void button_handler(void *params)
{
	switch_prio = TRUE;
}


void vApplicationIdleHook()
{


}


