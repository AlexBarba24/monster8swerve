/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "stream_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef *step_port;
	uint16_t step_pin;

	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;

	volatile uint8_t mode;

	volatile uint32_t accumulator;
	volatile int32_t target;
	volatile int32_t position;
	volatile uint32_t speed;
	volatile uint8_t enabled;
	volatile uint8_t step_high;
} StepperMotor;

typedef struct {
    uint8_t id;

    StepperMotor *stepper;

    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    GPIO_PinState enable_active_state;
} MotorControllerContext;

typedef enum {
    CMD_NONE = 0x000,
    CMD_MOVE_ABS = 0x001,
    CMD_MOVE_REL = 0x010,
    CMD_SET_SPEED = 0x011,
    CMD_STOP = 0x100,
    CMD_DISABLE = 0x101,
    CMD_STATUS = 0x110
} CommandType;

typedef enum {
    TRANSPORT_USB = 0,
    TRANSPORT_CAN = 1
} TransportType;

typedef struct {
    TransportType source;
    CommandType type;
    uint8_t motor_id;
    int32_t target;
    uint32_t speed;
    uint32_t flags;
} ControllerCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_CMD_RUN    0x01  // Normal running state
#define MOTOR_CMD_STOP   0x00  // Bit 0: Stop spinning command
#define MODE_SPEED 0
#define MODE_POS 1
#define DEFAULT_SPEED 50

#define NUM_STEPPERS 1
#define MAX_SPEED 1

#define USE_USB_COMMANDS 1
#define USE_CAN_COMMANDS 0

#define USB_LINE_MAX 256
#define USB_RX_STREAM_SIZE 256
#define USB_TX_STREAM_SIZE 256

#define NUM_ENCODERS 4
#define ENCODER_CHANNEL_OFFSET 4
#define INF 0x7FFFFFFF
#define NINF 0xFFFFFFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

/* Definitions for commandSchedule */
osThreadId_t commandScheduleHandle;
const osThreadAttr_t commandSchedule_attributes = {
  .name = "commandSchedule",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorController */
osThreadId_t motorControllerHandle;
const osThreadAttr_t motorController_attributes = {
  .name = "motorController",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for logger */
osThreadId_t loggerHandle;
const osThreadAttr_t logger_attributes = {
  .name = "logger",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usbCommand */
osThreadId_t usbCommandHandle;
const osThreadAttr_t usbCommand_attributes = {
  .name = "usbCommand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
StepperMotor steppers[NUM_STEPPERS];
MotorControllerContext motorCtx[NUM_STEPPERS];
osThreadId_t motorTaskHandles[NUM_STEPPERS];
StreamBufferHandle_t usbRxStream;
StreamBufferHandle_t usbTxStream;
osMessageQueueId_t schedulerCommandQueue;
volatile uint32_t usb_rx_callback_count = 0;
volatile uint32_t usb_rx_byte_count = 0;
volatile uint32_t usb_rx_drop_count = 0;
volatile uint32_t usb_command_lines = 0;

uint32_t encoder_adc[NUM_ENCODERS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * @brief Initialize StepperMotors and MotorControllerContexts
 * @retval None
 */
void MotorContexts_Init(void)
{
    steppers[0] = (StepperMotor) {
        .step_port = GPIOD,
        .step_pin = GPIO_PIN_13,
        .dir_port = GPIOD,
        .dir_pin = GPIO_PIN_12,
        .mode = MODE_POS,
        .position = 0,
        .target = 0,
        .speed = DEFAULT_SPEED,
        .enabled = 0,
        .step_high = 0
    };

    motorCtx[0] = (MotorControllerContext) {
        .id = 0,
        .stepper = &steppers[0],
        .enable_port = GPIOB,
        .enable_pin = GPIO_PIN_6,
        .enable_active_state = GPIO_PIN_RESET,
    };

    // Repeat for steppers[1], steppers[2], etc.
}

void HandleUsbCommand(const char *line)
{
    ControllerCommand cmd = {0};

    int motor_id;
    long target;
    long speed;
    printf("Parsing Command: %s\r\n", line);
    osDelay(5);
    if (sscanf(line, "MOVEABS %d %ld %ld", &motor_id, &target, &speed) == 3) {
        cmd.source = TRANSPORT_USB;
        cmd.type = CMD_MOVE_ABS;
        cmd.motor_id = motor_id;
        cmd.target = target;
        cmd.speed = speed;
        osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
        if (status == osOK) {
            printf("OK QUEUED MOVEABS %d %ld %ld\r\n", motor_id, target, speed);
        }
        else {
            printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
        }
        return;
    }
    if (sscanf(line, "MOVEREL %d %ld %ld", &motor_id, &target, &speed) == 3){
    	cmd.source = TRANSPORT_USB;
		cmd.type = CMD_MOVE_REL;
		cmd.motor_id = motor_id;
		cmd.target = target;
		cmd.speed = speed;
		osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
		if (status == osOK) {
			printf("OK QUEUED MOVEREL %d %ld %ld\r\n", motor_id, target, speed);
		}
		else {
			printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
		}
		return;
    }
    if (sscanf(line, "MOVESPEED %d %ld", &motor_id, &speed) == 2){
		cmd.source = TRANSPORT_USB;
		cmd.type = CMD_MOVE_REL;
		cmd.motor_id = motor_id;
		cmd.target = speed < 0 ? NINF : INF;
		cmd.speed = speed;
		osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
		if (status == osOK) {
			printf("OK QUEUED MOVESPEED %d %ld\r\n", motor_id, speed);
		}
		else {
			printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
		}
		return;
	}
    if (sscanf(line, "STOP %d", &motor_id) == 3){
		cmd.source = TRANSPORT_USB;
		cmd.type = CMD_STOP;
		cmd.motor_id = motor_id;
		osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
		if (status == osOK) {
			printf("OK QUEUED STOP %d\r\n", motor_id);
		}
		else {
			printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
		}
		return;
	}
    if (sscanf(line, "DISABLE %d", &motor_id) == 3){
    		cmd.source = TRANSPORT_USB;
    		cmd.type = CMD_DISABLE;
    		cmd.motor_id = motor_id;
    		osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
    		if (status == osOK) {
    			printf("OK QUEUED DISABLE %d\r\n", motor_id);
    		}
    		else {
    			printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
    		}
    		return;
    	}
    if (sscanf(line, "STATUS %d", &motor_id) == 3){
		cmd.source = TRANSPORT_USB;
		cmd.type = CMD_STATUS;
		cmd.motor_id = motor_id;
		osStatus_t status = osMessageQueuePut(schedulerCommandQueue, &cmd, 0, 0);
		if (status == osOK) {
			printf("OK QUEUED STOP %d\r\n", motor_id);
		}
		else {
			printf("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
		}
		return;
	}
    printf("Invalid Command Received.\r\n");
}


/**
  * @brief  Returns a uint32_t command for montor controller task notifications.
  *
  *
  * @param  target the target position to move stepper.
  * @param  dir the sign of the speed.
  *          This parameter can be one of GPIO_PIN_x where x can be (0..15).
  * @param  speed the speed for the command.
  * @retval uint32_t command.
  */
int encodeCommand(int16_t target, uint8_t dir, uint8_t speed, CommandType cmd){
	return (((((target << 1) + (dir & 0x1)) << 8) + speed) << 3) + cmd;
}

int16_t decodeSpeed(uint32_t input) {
	int dir = (input >> 11) & 0x1;
	return (dir * -1) * ((input >> 3) & 0xFF);
}

CommandType decodeCommand(uint32_t input) {
	return input & 0x7;
}

int16_t decodeTarget(uint32_t input) {
	return (input >> 12) & 0xFFFF;
}

/* USER CODE END 0 */

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
	steppers[0] = (StepperMotor) {
		.step_port = GPIOD,
		.step_pin = GPIO_PIN_13,
		.dir_port = GPIOD,
		.dir_pin = GPIO_PIN_12,
		.mode = MODE_POS,
		.position = 0,
		.step_high=0,
		.speed=DEFAULT_SPEED,
		.enabled = 0
	};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  usbRxStream = xStreamBufferCreate(USB_RX_STREAM_SIZE, 1);
  usbTxStream = xStreamBufferCreate(USB_TX_STREAM_SIZE, 1);

  if (usbRxStream == NULL || usbTxStream == NULL)
  {
      Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MotorContexts_Init();
  HAL_StatusTypeDef adc_status =
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)encoder_adc, NUM_ENCODERS);

  if (adc_status != HAL_OK)
  {
      printf("ADC DMA start failed: %d, HAL ADC error: 0x%08lx\r\n",
             adc_status,
             HAL_ADC_GetError(&hadc1));
      Error_Handler();
  }
  else
  {
      printf("ADC DMA started OK\r\n");
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of commandSchedule */
  commandScheduleHandle = osThreadNew(StartTask1, NULL, &commandSchedule_attributes);

  /* creation of motorController */
  motorControllerHandle = osThreadNew(StartTask2, NULL, &motorController_attributes);

  /* creation of logger */
  loggerHandle = osThreadNew(StartTask03, NULL, &logger_attributes);

  /* creation of usbCommand */
  usbCommandHandle = osThreadNew(StartTask04, NULL, &usbCommand_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  for (int i = 0; i < NUM_STEPPERS; i++){
	  motorTaskHandles[i] = osThreadNew(StartTask2, &motorCtx[i], &motorController_attributes);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  schedulerCommandQueue = osMessageQueueNew(16, sizeof(ControllerCommand), NULL);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	ControllerCommand cmd;
	HAL_TIM_Base_Start_IT(&htim2);

	for (;;)
	{
		osStatus_t status = osMessageQueueGet(
			schedulerCommandQueue,
			&cmd,
			NULL,
			osWaitForever
		);

		if (status == osOK)
		{
			printf("Received Command!\r\n");

			if (cmd.motor_id >= NUM_STEPPERS)
			{
				printf("ERR BAD MOTOR ID %d\r\n", cmd.motor_id);
				break;
			}

			if (motorTaskHandles[cmd.motor_id] == NULL)
			{
				printf("ERR MOTOR HANDLE NULL %d\r\n", cmd.motor_id);
				break;
			}

			uint32_t notifyValue = encodeCommand(cmd.target, 0, cmd.speed, cmd.type);

			BaseType_t result = xTaskNotify(
				motorTaskHandles[cmd.motor_id],
				notifyValue,
				eSetValueWithOverwrite
			);

			printf("Notify result=%ld\r\n", (long)result);

			break;


		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2 */
void StartTask2(void *argument)
{
  /* USER CODE BEGIN StartTask2 */
	uint32_t ulNotifiedValue;
	MotorControllerContext *context = (MotorControllerContext*) argument;
	StepperMotor* stepper = context->stepper;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	/* Infinite loop */
	for(;;)
	{
        if (xTaskNotifyWait(0, 0xFFFF, &ulNotifiedValue, portMAX_DELAY)==pdTRUE){
        	CommandType command = decodeCommand(ulNotifiedValue);
        	switch (command) {
        		case CMD_STATUS: {
        			printf(
						"Motor (%d) Status: target=%ld, speed=%lu, enabled=%d, mode=%s",
						context->id,
						stepper->target,
						stepper->speed,
						stepper->enabled,
						stepper->mode == MODE_POS ? "position" : "speed"
					);
        			break;
        		}
        		case CMD_STOP: {
        			stepper->speed = 0;
        			stepper->mode = MODE_SPEED;
        			stepper->enabled = 1;
        		}
        		case CMD_DISABLE: {
        			HAL_GPIO_WritePin(context->enable_port, context->enable_pin, 0);
        			break;
        		}
        		case CMD_SET_SPEED:
        		case CMD_MOVE_REL:
        		case CMD_MOVE_ABS: {
        			stepper->target = decodeTarget(ulNotifiedValue);
        			if (CMD_MOVE_REL) {
        				stepper->target += stepper->position;
        			}
        			uint8_t speed = decodeSpeed(ulNotifiedValue);
        			if(command == CMD_SET_SPEED && speed == 0)
        				stepper->speed = 0;
					else
						stepper->speed = speed == 0 ? DEFAULT_SPEED : speed;
        			stepper->mode = MODE_POS;
        			stepper->enabled = 1;
        			stepper->accumulator = 0;
        			HAL_GPIO_WritePin(context->enable_port, context->enable_pin, 0);
					HAL_GPIO_WritePin(
							context->stepper->dir_port,
							context->stepper->dir_pin,
							context->stepper->target < context->stepper->position
					);
        			break;
        		}
        		default:
        			printf("Motor (%d), recieved invalid command %lu.", context->id, ulNotifiedValue);
        	}
        	printf("Received Task Notification: (%ld)\r\n", ulNotifiedValue);
        	uint8_t enabled = ulNotifiedValue & MOTOR_CMD_RUN;
        	int32_t target = ulNotifiedValue >> 2;
        	uint8_t mode = (ulNotifiedValue>>1) & 1;
        	context->stepper->target = target * 16;
        	context->stepper->speed = DEFAULT_SPEED;
        	context->stepper->mode = mode;
        	context->stepper->enabled = enabled;
        	context->stepper->accumulator = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, !enabled);
			HAL_GPIO_WritePin(
					context->stepper->dir_port,
					context->stepper->dir_pin,
					context->stepper->target < context->stepper->position
			);

        	printf("New State: target: %ld, speed: %d, enabled: %d.\r\n", target, DEFAULT_SPEED, enabled);
        }
	}
  /* USER CODE END StartTask2 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the logger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
//	  printf("USB rx_cb=%lu rx_bytes=%lu rx_drop=%lu lines=%lu\r\n",
//	         usb_rx_callback_count,
//	         usb_rx_byte_count,
//	         usb_rx_drop_count,
//	         usb_command_lines);
	  printf("ADC: IN4=%lu IN5=%lu IN6=%lu IN7=%lu\r\n",
	         encoder_adc[0],
	         encoder_adc[1],
	         encoder_adc[2],
	         encoder_adc[3]);
	  osDelay(1000);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the usbCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	char line[USB_LINE_MAX];
	size_t line_len = 0;
	uint8_t ch;

	for (;;)
	{
		if (xStreamBufferReceive(usbRxStream, &ch, 1, portMAX_DELAY) == 1)
		{
			if (ch == '\r')
			{
				continue;
			}

			if (ch == '\n')
			{
				line[line_len] = '\0';

				if (line_len > 0)
				{
					HandleUsbCommand(line);
				}

				line_len = 0;
			}
			else
			{
				if (line_len < USB_LINE_MAX - 1)
				{
					line[line_len++] = (char)ch;
				}
				else
				{
					line_len = 0;
					printf("ERR LINE_TOO_LONG\r\n");
				}
			}
		}
	}
  /* USER CODE END StartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM2){
		for (int i = 0; i < NUM_STEPPERS; i++){
			if (!steppers[i].enabled) {
				continue;
			}
			if(steppers[i].mode==MODE_POS) {
				if (steppers[i].target==steppers[i].position) {
					steppers[i].enabled = 0;
					continue;
				}
				if (steppers[i].step_high) {
					steppers[i].step_port->BSRR = (uint32_t)steppers[i].step_pin << 16; // STEP low
					steppers[i].step_high = 0;
					continue;
				}
				steppers[i].accumulator += steppers[i].speed;
				if (steppers[i].accumulator > MAX_SPEED) {
					steppers[i].step_port->BSRR = steppers[i].step_pin; // STEP high
					steppers[i].step_high = 1;
					if (steppers[i].position < steppers[i].target)
						steppers[i].position++;
					else
						steppers[i].position--;
				}
			}
		}
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
