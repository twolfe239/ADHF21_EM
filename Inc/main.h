/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "ssd1306.h"
#include "bme680.h"
#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
struct bme680_dev gas_sensor;
struct bme680_field_data data;
RTC_TimeTypeDef sTime;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

void BME680_Read(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AL_BUT_Pin GPIO_PIN_0
#define AL_BUT_GPIO_Port GPIOA
#define AL_BUT_EXTI_IRQn EXTI0_IRQn
#define RS_Pin GPIO_PIN_3
#define RS_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define MinUp_Pin GPIO_PIN_12
#define MinUp_GPIO_Port GPIOB
#define MinUp_EXTI_IRQn EXTI15_10_IRQn
#define HourUp_Pin GPIO_PIN_13
#define HourUp_GPIO_Port GPIOB
#define HourUp_EXTI_IRQn EXTI15_10_IRQn
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BUF ((struct uip_eth_hdr *)&uip_buf[UIP_LLH_LEN])
#define SPON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define SPOFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define intPin0 0x00
#define intPin12 0x0C
#define intPin13 0x0D
#define intPinDef 0xEF





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
