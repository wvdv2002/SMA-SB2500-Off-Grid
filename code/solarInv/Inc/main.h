/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GRIDRELAY_1_Pin GPIO_PIN_14
#define GRIDRELAY_1_GPIO_Port GPIOC
#define GRIDRELAY_0_Pin GPIO_PIN_15
#define GRIDRELAY_0_GPIO_Port GPIOC
#define ADC_IN0_Pin GPIO_PIN_0
#define ADC_IN0_GPIO_Port GPIOC
#define CURSENS_Pin GPIO_PIN_1
#define CURSENS_GPIO_Port GPIOC
#define TRAFO_Pin GPIO_PIN_2
#define TRAFO_GPIO_Port GPIOC
#define TEMPHEAT_Pin GPIO_PIN_1
#define TEMPHEAT_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define VDCBUS_Pin GPIO_PIN_4
#define VDCBUS_GPIO_Port GPIOA
#define RELAY_CHECK1_Pin GPIO_PIN_5
#define RELAY_CHECK1_GPIO_Port GPIOA
#define RELAY_CHECK0_Pin GPIO_PIN_6
#define RELAY_CHECK0_GPIO_Port GPIOA
#define IO_IN0_Pin GPIO_PIN_7
#define IO_IN0_GPIO_Port GPIOA
#define ADC_IN1_Pin GPIO_PIN_4
#define ADC_IN1_GPIO_Port GPIOC
#define ISOLATION_SENSE_Pin GPIO_PIN_5
#define ISOLATION_SENSE_GPIO_Port GPIOC
#define IO_IN2_Pin GPIO_PIN_0
#define IO_IN2_GPIO_Port GPIOB
#define LCD_MIC_Pin GPIO_PIN_1
#define LCD_MIC_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_10
#define LED_YELLOW_GPIO_Port GPIOB
#define DONOTUSECOMPATIBILITY_Pin GPIO_PIN_11
#define DONOTUSECOMPATIBILITY_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOB
#define BRIDGES_SHUTDOWN_Pin GPIO_PIN_13
#define BRIDGES_SHUTDOWN_GPIO_Port GPIOB
#define BRIDGE0_LIN_Pin GPIO_PIN_14
#define BRIDGE0_LIN_GPIO_Port GPIOB
#define BRIDGE1_LIN_Pin GPIO_PIN_15
#define BRIDGE1_LIN_GPIO_Port GPIOB
#define LED_HEARTBEAT_Pin GPIO_PIN_6
#define LED_HEARTBEAT_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_7
#define LCD_RW_GPIO_Port GPIOC
#define LCD_BACKL_Pin GPIO_PIN_8
#define LCD_BACKL_GPIO_Port GPIOC
#define LCD_SWB_Pin GPIO_PIN_9
#define LCD_SWB_GPIO_Port GPIOC
#define IO_OUT1_Pin GPIO_PIN_8
#define IO_OUT1_GPIO_Port GPIOA
#define BRIDGE0_HIN_Pin GPIO_PIN_9
#define BRIDGE0_HIN_GPIO_Port GPIOA
#define BRIDGE1_HIN_Pin GPIO_PIN_10
#define BRIDGE1_HIN_GPIO_Port GPIOA
#define IO_OUT0_Pin GPIO_PIN_11
#define IO_OUT0_GPIO_Port GPIOA
#define RS485_TXENA_Pin GPIO_PIN_12
#define RS485_TXENA_GPIO_Port GPIOA
#define IO_IN1_Pin GPIO_PIN_15
#define IO_IN1_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_10
#define LCD_D7_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_11
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOC
#define LCD_SWA_Pin GPIO_PIN_2
#define LCD_SWA_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define LCD_ENA_Pin GPIO_PIN_5
#define LCD_ENA_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_6
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_7
#define RS485_RX_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_8
#define LCD_D5_GPIO_Port GPIOB
#define RS485_RXENA_Pin GPIO_PIN_9
#define RS485_RXENA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
