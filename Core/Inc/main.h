/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define RESET_CAUSE_STACK_OVERFLOW 1u
#define RESET_CAUSE_MALLOC_FAILED  2u
#define RESET_CAUSE_HARDFAULT      3u

/* Diagnostic only: manually drain FIFO0 with HAL_CAN_GetRxMessage(), bypassing
 * HAL_CAN_IRQHandler(), the application callback, and all RTOS queue code. Set
 * to 0 after completing the staged CAN receive-path tests. */
#define CAN_RX_IRQ_ISOLATION_TEST 0

/* Stage reached after a CAN frame has been parsed:
 *   1 = construct ControllerCommand, then return before QueueCommand()
 *   2 = construct it and call the test sink inside QueueCommand()
 * Keep the IRQ isolation test enabled while using either stage. */
#define CAN_COMMAND_PATH_TEST_STAGE 2

extern volatile uint32_t can_rx_isolation_irq_count;
extern volatile uint32_t can_rx_isolation_read_count;
extern volatile uint32_t can_rx_isolation_error_count;
extern volatile uint32_t can_rx_isolation_queue_put_count;
extern volatile uint32_t can_rx_isolation_queue_get_count;
extern volatile uint32_t can_rx_isolation_queue_error_count;
extern volatile uint32_t can_rx_isolation_parse_count;
extern volatile uint32_t can_rx_isolation_command_count;
extern volatile uint32_t can_rx_isolation_queue_command_count;
extern volatile uint32_t can_rx_isolation_call_before_count;
extern volatile uint32_t can_rx_isolation_call_entry_count;
extern volatile uint32_t can_rx_isolation_call_after_count;
void RecordResetCauseAndReboot(uint32_t cause, const char *name);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_Enable_Pin GPIO_PIN_2
#define Z_Enable_GPIO_Port GPIOE
#define Y_CS_Pin GPIO_PIN_3
#define Y_CS_GPIO_Port GPIOE
#define Y_Dir_Pin GPIO_PIN_4
#define Y_Dir_GPIO_Port GPIOE
#define Y_Step_Pin GPIO_PIN_5
#define Y_Step_GPIO_Port GPIOE
#define X_CS_Pin GPIO_PIN_6
#define X_CS_GPIO_Port GPIOE
#define X_Dir_Pin GPIO_PIN_13
#define X_Dir_GPIO_Port GPIOC
#define X_Step_Pin GPIO_PIN_14
#define X_Step_GPIO_Port GPIOC
#define XY_Enable_Pin GPIO_PIN_15
#define XY_Enable_GPIO_Port GPIOC
#define SCLK_Pin GPIO_PIN_12
#define SCLK_GPIO_Port GPIOE
#define MISO_Pin GPIO_PIN_13
#define MISO_GPIO_Port GPIOE
#define MOSI_Pin GPIO_PIN_14
#define MOSI_GPIO_Port GPIOE
#define E4_Dir_Pin GPIO_PIN_12
#define E4_Dir_GPIO_Port GPIOD
#define E4_Step_Pin GPIO_PIN_13
#define E4_Step_GPIO_Port GPIOD
#define E3_CS_Pin GPIO_PIN_15
#define E3_CS_GPIO_Port GPIOD
#define E3_Dir_Pin GPIO_PIN_6
#define E3_Dir_GPIO_Port GPIOC
#define E3_Step_Pin GPIO_PIN_7
#define E3_Step_GPIO_Port GPIOC
#define E3_Enable_Pin GPIO_PIN_8
#define E3_Enable_GPIO_Port GPIOC
#define E2_Dir_Pin GPIO_PIN_1
#define E2_Dir_GPIO_Port GPIOD
#define E2_Step_Pin GPIO_PIN_2
#define E2_Step_GPIO_Port GPIOD
#define E2_Enable_Pin GPIO_PIN_3
#define E2_Enable_GPIO_Port GPIOD
#define E1_CS_Pin GPIO_PIN_4
#define E1_CS_GPIO_Port GPIOD
#define E1_Dir_Pin GPIO_PIN_5
#define E1_Dir_GPIO_Port GPIOD
#define E1_Step_Pin GPIO_PIN_6
#define E1_Step_GPIO_Port GPIOD
#define E1_Enable_Pin GPIO_PIN_7
#define E1_Enable_GPIO_Port GPIOD
#define E0_Dir_Pin GPIO_PIN_4
#define E0_Dir_GPIO_Port GPIOB
#define E0_Step_Pin GPIO_PIN_5
#define E0_Step_GPIO_Port GPIOB
#define E04_Enable_Pin GPIO_PIN_6
#define E04_Enable_GPIO_Port GPIOB
#define Z_CS_Pin GPIO_PIN_7
#define Z_CS_GPIO_Port GPIOB
#define Z_Dir_Pin GPIO_PIN_0
#define Z_Dir_GPIO_Port GPIOE
#define Z_Step_Pin GPIO_PIN_1
#define Z_Step_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
