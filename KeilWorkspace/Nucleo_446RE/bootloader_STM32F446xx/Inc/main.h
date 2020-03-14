/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdint.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void bootloader_handle_getver_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_getrdp_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_goto_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_flasherase_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_memwrite_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_enrwprotect_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_memread_cmd(uint8_t* bl_rx_buffer);			
void bootloader_handle_readsectorstatus_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_otpread_cmd(uint8_t* bl_rx_buffer);
void bootloader_handle_disrwprotect_cmd(uint8_t* bl_rx_buffer);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

// Ack and Nack bytes
#define BL_ACK  0xA5
#define BL_NACK 0x7F

#define VERIFY_CRC_SUCESS 0x00
#define VERIFY_CRC_FAIL 0x01

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01


#define SRAM1_SIZE 112*1024
#define SRAM1_END  (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE 16*1024
#define SRAM2_END  (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE 512*1024
#define BKPSRAM_SIZE 4*1024
#define BKPSRAM_END  (SRAM2_BASE + SRAM2_SIZE)

#define INVALID_SECTOR 0xFF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
