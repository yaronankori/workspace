/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    26-01-2020
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define FALSE 0
#define TRUE  1
#define NOT_PRESSED FALSE
#define PRESSED TRUE

// Commands
#define LED_ON_COMMAND     1
#define LED_OFF            2
#define LED_TOGGLE_COMMAND 3
#define LED_TOGGLE_STOP    4
#define LED_READ_STATUS    5
#define LED_READ_DATE_TIME 6

//Function prototype
static void prvSetupHardware(void);
void printmsg(char *msg);
static void prvSetupUART(void);
void prvSetupGpio(void);
void rtos_delay(uint32_t delay_in_ms);
uint8_t getCommandCode(uint8_t *buffer);
uint8_t getArguments(uint8_t *buffer);


// task prototypes
void vTask1_menu_display(void *params);
void vTask2_cmd_handling(void *params);
void vTask3_cmd_processing(void *params);
void vTask4_uart_write(void *params);

void make_led_on(void);
void make_led_off(void);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void read_led_status(char* task_msg);
void read_rtc_status(char* task_msg);
void print_error_message(char* task_msg);

void led_toggle(TimerHandle_t xTimer);



char usr_msg[250];
uint8_t switch_prio = FALSE;


TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;
TaskHandle_t xTaskHandle4 = NULL;

// command structure
typedef struct APP_CMD
{
		uint8_t COMMAND_NUM;
		uint8_t COMMAND_ARGS[10];
}APP_CMD_t;

QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

TimerHandle_t led_timer_handle = NULL ;


uint8_t command_buffer[20];
uint8_t command_len=0;

char menu[] ={"\
		\r\nLED_ON                ----->1\
		\r\nLED_OFF               ----->2\
		\r\nLED_TOGGLE            ----->3\
		\r\nLED_TOFFLE_OFF        ----->4\
		\r\nLED_READ_STATUS       ----->5\
		\r\nRTC_PRINT_DATETIME    ----->6\
		\r\nEXIT_APP              ----->0\
		\r\nType your option here:\
		"};

int main(void)
{

	DWT->CTRL |= (1 << 0);// ENABLE the CYCCNT in DWT_CTRL (Time couner for the tracking of the system view)

	//1) HSI ON, PLL OFF, HSE OFF, System clock = 16MHz, CPU clock 16MHz
	RCC_DeInit();

	//2) Update the system clock variable(to 16MHz)
	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"This is QUEUE Demo\r\n");
	printmsg(usr_msg);
	//Start recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	//lets create command queue
	command_queue = xQueueCreate(10,sizeof(APP_CMD_t*));
	uart_write_queue = xQueueCreate(10,sizeof(char*));

	if((command_queue != NULL) && (uart_write_queue != NULL))
	{
		//create task1
		xTaskCreate(vTask1_menu_display,"TASK1-MENU",500,NULL,1,&xTaskHandle1);
		//create task2
		xTaskCreate(vTask2_cmd_handling,"TASK2-CMD-HANDLING",500,NULL,2,&xTaskHandle2);
		//create task3
		xTaskCreate(vTask3_cmd_processing,"TASK3-CMD-PROCESS",500,NULL,2,&xTaskHandle3);
		//create task4
		xTaskCreate(vTask4_uart_write,"TASK4-UART-WRITE",500,NULL,2,&xTaskHandle4);

		//start the scheduler
		vTaskStartScheduler();
	}
	else
	{
		sprintf(usr_msg,"Queue creation failed \r\n");
		printmsg(usr_msg);
	}



	for(;;);
}

void vApplicationIdleHook()
{


}

void vTask1_menu_display(void *params)
{
	char *pData = menu;

	while(1)
	{
		xQueueSend(uart_write_queue,&pData,portMAX_DELAY);
		// lets wait here until someone notifies
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Wake up in notification !!!!!!!!!!!!!!!!!!!!!!!
	}
}
void vTask2_cmd_handling(void *params)
{
	uint8_t command_code=0;
	APP_CMD_t *new_cmd;

	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);  // Wake up in notification!!!!!!!!!!!!!!!!!!!!!

		taskENTER_CRITICAL();
		command_code = getCommandCode(command_buffer);
		new_cmd = (APP_CMD_t*) pvPortMalloc(sizeof(APP_CMD_t));
		new_cmd ->COMMAND_NUM = command_code;
		getArguments(new_cmd->COMMAND_ARGS);
		taskEXIT_CRITICAL();


		xQueueSend(command_queue,&new_cmd,portMAX_DELAY);


	}
}

void vTask3_cmd_processing(void *params)
{
	APP_CMD_t *new_cmd;
	char task_msg[50];
	uint32_t toggle_duration = pdMS_TO_TICKS(500); // 500ms

	while(1)
	{
		xQueueReceive(command_queue,(void*)&new_cmd,portMAX_DELAY);//Waiting for command_queue filling!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		if(new_cmd -> COMMAND_NUM == LED_ON_COMMAND)
		{
			make_led_on();
		}
		else if(new_cmd -> COMMAND_NUM == LED_OFF)
		{
			make_led_off();
		}
		else if(new_cmd -> COMMAND_NUM == LED_TOGGLE_COMMAND)
		{
			led_toggle_start(toggle_duration);
		}
		else if(new_cmd -> COMMAND_NUM == LED_TOGGLE_STOP)
		{
			led_toggle_stop();
		}
		else if(new_cmd -> COMMAND_NUM == LED_READ_STATUS)
		{
			read_led_status(task_msg);
		}
		else if(new_cmd -> COMMAND_NUM == LED_READ_DATE_TIME)
		{
			read_rtc_status(task_msg);
		}
		else
		{
			print_error_message(task_msg);
		}

		vPortFree(new_cmd);

	}
}

void vTask4_uart_write(void *params)
{
	char *pData = NULL;
	while(1)
	{
		xQueueReceive(uart_write_queue,&pData,portMAX_DELAY); //Waiting for uart_write_queue filling!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		printmsg(pData);
	}
}




void make_led_on(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
}
void make_led_off(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
}
void led_toggle_start(uint32_t duration)
{
	if(led_timer_handle == NULL)
	{
		// Create the software timer
		led_timer_handle = xTimerCreate("LED-TIMER",duration,pdTRUE,NULL,led_toggle);
		// start the software timer
		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
	else
	{
		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
}
void led_toggle_stop(void)
{
	xTimerStop(led_timer_handle,portMAX_DELAY);
}
void read_led_status(char* task_msg)
{
	sprintf(task_msg,"\r\nLED status is %d\r\n",GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5));
	xQueueSend(uart_write_queue,&task_msg,portMAX_DELAY);
}
void read_rtc_status(char* task_msg)
{
	RTC_TimeTypeDef RTC_time;
	RTC_DateTypeDef RTC_date;

	RTC_GetTime(RTC_Format_BIN,&RTC_time);
	RTC_GetDate(RTC_Format_BIN,&RTC_date);

	sprintf(task_msg,"\r\nTime: %02d:%02d:%02d Date: %02d-%02d-%02d\r\n",RTC_time.RTC_Hours,RTC_time.RTC_Minutes,RTC_time.RTC_Seconds\
																		,RTC_date.RTC_Date,RTC_date.RTC_Month,RTC_date.RTC_Year\
																		);

	xQueueSend(uart_write_queue, &task_msg,portMAX_DELAY);

}
void print_error_message(char* task_msg)
{
	sprintf(task_msg,"\r\nERROR MESSAGE\r\n");
	xQueueSend(uart_write_queue,&task_msg,portMAX_DELAY);
}


void led_toggle(TimerHandle_t xTimer)
{
	GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
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
	gpio_uart_pins.GPIO_Pin =GPIO_Pin_2 |GPIO_Pin_3;
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
	huart2_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	huart2_init.USART_Parity = USART_Parity_No;
	huart2_init.USART_StopBits = USART_StopBits_1;
	huart2_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&huart2_init);



	//********CONFIG UART2 Interrupt**************************
	//Config the uart2 byte reception interrupt
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	//Config the Internal interrupt controller, ARM
	NVIC_SetPriority(USART2_IRQn,5); //0 in the highest

	NVIC_EnableIRQ(USART2_IRQn);
	//********************************************************

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
	traceISR_EXIT();
	// Random code
}

uint8_t getCommandCode(uint8_t *buffer)
{
	return buffer[0] - 0x30;
}

uint8_t getArguments(uint8_t *buffer)
{
	// empty function for now
	return 0;
}


void USART2_IRQHandler(void)
{
	uint16_t data_byte;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(USART_GetFlagStatus(USART2,USART_IT_RXNE))
	{
		EXTI_ClearITPendingBit(USART_IT_RXNE);

		//A data byte received from the user
		data_byte = USART_ReceiveData(USART2);

		command_buffer[command_len++] = data_byte & 0xFF;


		if(data_byte == '\r')
		{
			command_len = 0;
			// user finished the type
			//lets notify the command handling task
			xTaskNotifyFromISR(xTaskHandle2,0,eNoAction, &xHigherPriorityTaskWoken);

			// Lets notify to the menu display
			xTaskNotifyFromISR(xTaskHandle1,0,eNoAction, &xHigherPriorityTaskWoken);

		}
	}

	//if the above freertos apis wake up any hkgher priority task then yeild the processor to eht higher priority task which is just woken up.
	if(xHigherPriorityTaskWoken)
	{
		taskYIELD();
	}

}






