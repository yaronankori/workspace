/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdarg.h>
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#define D_UART &huart2
#define C_UART &huart3

// Bootloader commands

//This command is used to read the bootlaoder version from the MCU
#define BL_GET_VER 0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP 0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID 0x53

//This command is used to read the FLASH read protection level
#define BL_GET_RDP_STATUS 0x54

//This command is used to jump bootloader to specified address
#define BL_GO_TO_ADDR 0x55

//This command is used to mass erase or sector erase of the user flash
#define BL_FLASH_ERASE 0x56

//This command is used to write datainto differeint memories of the MCU
#define BL_MEM_WRITE 0x57

//This command is used to enable or disable read/write protect on different secotr of the
#define BL_EN_R_W_PROTECT 0x58

//This command is used to read data from differeint memories of the microcontroller
#define BL_MEM_READ 0x59

//This command is used to read all the sector protection status
#define BL_READ_SECTOR_STATUS 0x5A

//This command is used to read the OTP content
#define BL_OTP_READ 0x5B

//This command is used to disable all protections from the chip
#define BL_DIS_R_W_PROTECT 0x5C




static void printmsg(char *format,...);

void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
char somedata[] = "Hello from bootloader\r\n"; 
#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
	 HAL_UART_Transmit(&huart2,(uint8_t*)somedata,sizeof(somedata),100); 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			
	 //HAL_UART_Transmit(&huart3,(uint8_t*)somedata,sizeof(somedata),100);
		
	 uint32_t current_tick = HAL_GetTick();		
	 //printmsg("current_tick = %d\n\r",current_tick);

	 while(HAL_GetTick() <= current_tick+500);
			
	 static char OpEnable = 1; // for 1 time enable of the operation, or going to boot or going to user application
    		
	 if(HAL_GPIO_ReadPin(B1_GPIO_Port ,B1_Pin) == GPIO_PIN_RESET)	
	 {
		  if(OpEnable==1)
			{
				bootloader_uart_read_data();
				printmsg("BL_DEBUG_MSG:Button is pressed.. going to BL mode\r\n");
							
				OpEnable = 0;
			}
	 }
	 else
	 {
		  if(OpEnable==1)
			{
				bootloader_jump_to_user_app();
				printmsg("BL_DEBUG_MSG:Button is not pressed.. executing user app\r\n");
			 
				OpEnable = 0;
			}
	 }
	
	
		
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;
	
	while(1)
	{
		memset(bl_rx_buffer,0,200);
		// here we will read and decode the commands comming from host
		// first read only one byte from the host, which is the length field of the command packet
		HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		
		rcv_len = bl_rx_buffer[0];
		
		HAL_UART_Receive(C_UART,bl_rx_buffer,rcv_len,HAL_MAX_DELAY);
		
		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buffer);
				break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buffer);
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(bl_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
				bootloader_handle_goto_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				bootloader_handle_flasherase_cmd(bl_rx_buffer);
				break;
			case BL_MEM_WRITE:
				bootloader_handle_memwrite_cmd(bl_rx_buffer);
				break;
			case BL_EN_R_W_PROTECT:
				bootloader_handle_enrwprotect_cmd(bl_rx_buffer);
				break;
			case BL_MEM_READ:
				bootloader_handle_memread_cmd(bl_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_readsectorstatus_cmd(bl_rx_buffer);
				break;
			case BL_OTP_READ:
				bootloader_handle_otpread_cmd(bl_rx_buffer);
				break;
		  case BL_DIS_R_W_PROTECT:
				bootloader_handle_disrwprotect_cmd(bl_rx_buffer);
				break;
			default:
				printmsg("BL_DEBUG_MSG: no such command\r\n");
				break;
			
			
		}

		
	}
	
	
}
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
void bootloader_jump_to_user_app(void)
{
	void (*app_reset_handler) (void); // A function pointer to hold the address of the reset handler of the user app
	
	printmsg("BL_DEBUG_MSG: bootloader _jump_to_user_app\r\n");
	
	//1. configure the MSP by reading the value from the base address of sector 2
	uint32_t msp_value = *(volatile uint32_t*) FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG: MSP value : %#x\r\n",msp_value);
	
	//This function comes from CMSIS
	//ITS ONLY UPDATE THE REGISTER, NOT COPY THE DATA TO 0x080000000!!!!!
	__set_MSP(msp_value); // MSP -> main stack pointer (the firest stored data in the block), second is reset handler, and then , the vector table
	
	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS
	
	//2. Now fetch the reset handler address of the user application from the location FLASH_SECTOR2_BASE_ADDRESS
	// (this is the reset handler address)
	uint32_t resethandler_address = *(volatile uint32_t*)(FLASH_SECTOR2_BASE_ADDRESS + 4); 
	app_reset_handler = (void*)resethandler_address; 
	// copy the addres to as a pointer function custing YOU CAN SEE IT IN THE ST UTILITY, address 1 of 0x08000000 will have the same data as 0x08008000!!!!!
	printmsg("BL_DEBUG_MSG: app reset handler is %#x\r\n",app_reset_handler);
	
	//3. Jump to reset handler of the user applictaion
	app_reset_handler();
	
}

void printmsg(char *format,...)
{
	char str[80];
	
	//extract the argument list using VA apis (need stdarg.h libraray)
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t*)str,strlen(str),100);
	va_end(args);
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}
void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer)
{
	
}
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer)
{
	
}	

void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer)
{
	
}
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_goto_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_flasherase_cmd(uint8_t* bl_rx_buffer)
{
	
}
void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_enrwprotect_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_memread_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_readsectorstatus_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_otpread_cmd(uint8_t* bl_rx_buffer)
{
	
}	
void bootloader_handle_disrwprotect_cmd(uint8_t* bl_rx_buffer)
{
	
}	
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
