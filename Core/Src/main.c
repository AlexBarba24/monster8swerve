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
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
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
	volatile int16_t speed;
	volatile int16_t commanded_speed;
	volatile uint32_t ramp_accumulator;
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
    CMD_NONE = 0x1,
    CMD_MOVE_ABS = 0x2,
    CMD_MOVE_REL = 0x3,
    CMD_SET_SPEED = 0x4,
    CMD_STOP = 0x5,
    CMD_DISABLE = 0x6,
    CMD_STATUS = 0x7
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
#define DEFAULT_SPEED 80

#define NUM_STEPPERS 8
#define MAX_SPEED 250

// Speed ramp profile for MODE_SPEED moves (position moves are unaffected and
// snap straight to their commanded speed). Each TIM2 tick, ramp_accumulator
// increments by 1; once it reaches RAMP_ACCUMULATOR_THRESHOLD it resets to 0
// and stepper->speed is nudged by SPEED_RAMP_STEP toward commanded_speed. So
// the ramp takes RAMP_ACCUMULATOR_THRESHOLD ticks per SPEED_RAMP_STEP units of
// speed change - raise the threshold for a gentler ramp, lower it (or raise
// the step) for a snappier one.
#define SPEED_RAMP_STEP 1
#define RAMP_ACCUMULATOR_THRESHOLD 50

#define USE_USB_COMMANDS 1
#define USE_CAN_COMMANDS 0

#define USB_LINE_MAX 256
/* The CDC OUT endpoint is re-armed unconditionally in CDC_Receive_FS, so this
 * buffer is the only thing absorbing a host burst - there is no NAK
 * backpressure. At 64 bytes per packet and one packet per 1ms frame, 256 bytes
 * was barely four packets, and an overrun corrupts a line mid-flight rather
 * than losing a whole command cleanly. */
#define USB_RX_STREAM_SIZE 1024
#define USB_TX_STREAM_SIZE 256

#define NUM_ENCODERS 4
#define ENCODER_CHANNEL_OFFSET 4
#define INF  0x7FFF
#define NINF 0x8000

#define CMD_TYPE_BITS 4
#define CMD_SPEED_BITS 8
#define CMD_DIR_BITS 1
#define CMD_TARGET_BITS 16

#define CMD_TYPE_MASK 0xF
#define CMD_SPEED_MASK 0xFF
#define CMD_DIR_MASK 0x1
#define CMD_TARGET_MASK 0xFFFF

/* Deferred logging. A USB CDC write costs ~1ms (bulk IN transfers are scheduled
 * on 1ms frame boundaries) and _write busy-waits up to 50ms when the endpoint is
 * still busy, so printf() anywhere between "byte arrived" and "stepper struct
 * updated" dominates command latency. Hot-path code instead formats into a
 * fixed-size record and hands it to logQueue; the logger task does the slow
 * write. Records are dropped rather than waited on when the queue is full, so a
 * slow or absent host can never stall motion. */
#define LOG_MSG_MAX 160
#define LOG_QUEUE_DEPTH 32
#define DIAG_INTERVAL_MS 1000

/* Per-command trace ("Parsing Command", "OK QUEUED ..."). An 8-motor batch
 * emits 10 such lines and each one is a separate ~1ms USB write, so above a few
 * command lines per second the log queue overflows. Dropped log records look
 * exactly like dropped commands from the host's point of view, which is why
 * this is off by default; errors, batch summaries and the periodic counters
 * below are always logged and are the reliable signal. */
#define CMD_LOG_VERBOSE 0

#if CMD_LOG_VERBOSE
#define LogTrace(...) LogDeferred(__VA_ARGS__)
#else
#define LogTrace(...) ((void)0)
#endif

/* Echo received command lines back to the host. Off by default: it is purely an
 * interactive-terminal convenience and costs a USB transaction per line. */
#define USB_ECHO_ENABLED 0

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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorController */
osThreadId_t motorControllerHandle;
const osThreadAttr_t motorController_attributes = {
  .name = "motorController",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for logger */
osThreadId_t loggerHandle;
const osThreadAttr_t logger_attributes = {
  /* Stack raised from 256 words: this task now also holds a LOG_MSG_MAX drain
   * buffer on top of newlib's vfprintf frame. */
  .name = "logger",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for outputWriter */
osThreadId_t outputWriterHandle;
const osThreadAttr_t outputWriter_attributes = {
  /* The only task permitted to write to stdout. It sits above the logger so
   * that a command acknowledgement preempts the logger's multi-second TMC5160
   * register reads, and below usbCommand so it can never delay parsing. */
  .name = "outputWriter",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for usbCommand */
osThreadId_t usbCommandHandle;
const osThreadAttr_t usbCommand_attributes = {
  .name = "usbCommand",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */
StepperMotor steppers[NUM_STEPPERS];
MotorControllerContext motorCtx[NUM_STEPPERS];
osThreadId_t motorTaskHandles[NUM_STEPPERS];
StreamBufferHandle_t usbRxStream;
StreamBufferHandle_t usbTxStream;
osMessageQueueId_t schedulerCommandQueue;
osMessageQueueId_t logQueue;
volatile uint32_t usb_rx_callback_count = 0;
volatile uint32_t usb_rx_byte_count = 0;
volatile uint32_t usb_rx_drop_count = 0;
volatile uint32_t usb_command_lines = 0;
volatile uint32_t log_drop_count = 0;
volatile uint32_t cmd_invalid_count = 0;
volatile uint32_t cmd_queue_drop_count = 0;
volatile uint32_t cmd_queue_peak = 0;

/* Notifications handed to each motor task vs. notifications it actually woke up
 * for. xTaskNotify uses eSetValueWithOverwrite, so a second command arriving
 * before the motor task runs silently replaces the first; a persistent gap
 * between these two totals is that coalescing, and it is invisible otherwise.
 * Each slot has exactly one writer, so no atomics are needed. */
volatile uint32_t notify_sent[NUM_STEPPERS];
volatile uint32_t notify_recv[NUM_STEPPERS];

volatile uint16_t encoder_adc[NUM_ENCODERS];

/* Crash/reset report carried across a reset via a no-init RAM section. */
#define RESET_REPORT_MAGIC 0xB16B0CA5u
typedef struct {
    uint32_t magic;
    uint32_t cause;
    char task[20];
} ResetReport;
__attribute__((section(".noinit"))) ResetReport g_reset_report;

extern USBD_HandleTypeDef hUsbDeviceFS;
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
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */
extern bool ConfigureSPIControllers(void);
uint32_t tmc5160_read(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t reg,
    uint8_t *status_out);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * @brief Initialize StepperMotors and MotorControllerContexts
 * @retval None
 */
void MotorContexts_Init(void)
{
	// Front Left Drive
    steppers[0] = (StepperMotor) {
        .step_port = GPIOC,
        .step_pin = GPIO_PIN_14,
        .dir_port = GPIOC,
        .dir_pin = GPIO_PIN_13,
        .mode = MODE_POS,
        .position = 0,
        .target = 0,
        .speed = 0,
        .enabled = 0,
        .step_high = 0
    };
    motorCtx[0] = (MotorControllerContext) {
        .id = 0,
        .stepper = &steppers[0],
        .enable_port = GPIOC,
        .enable_pin = GPIO_PIN_15,
        .enable_active_state = GPIO_PIN_RESET,
    };
    // Front Left Steer
	steppers[1] = (StepperMotor) {
		.step_port = GPIOE,
		.step_pin = GPIO_PIN_5,
		.dir_port = GPIOE,
		.dir_pin = GPIO_PIN_4,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[1] = (MotorControllerContext) {
		.id = 1,
		.stepper = &steppers[1],
		.enable_port = GPIOC,
		.enable_pin = GPIO_PIN_15,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Front Right Drive
	steppers[2] = (StepperMotor) {
		.step_port = GPIOE,
		.step_pin = GPIO_PIN_1,
		.dir_port = GPIOE,
		.dir_pin = GPIO_PIN_0,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[2] = (MotorControllerContext) {
		.id = 2,
		.stepper = &steppers[2],
		.enable_port = GPIOE,
		.enable_pin = GPIO_PIN_2,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Front Right Steer
	steppers[3] = (StepperMotor) {
		.step_port = GPIOB,
		.step_pin = GPIO_PIN_5,
		.dir_port = GPIOB,
		.dir_pin = GPIO_PIN_4,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[3] = (MotorControllerContext) {
		.id = 3,
		.stepper = &steppers[3],
		.enable_port = GPIOB,
		.enable_pin = GPIO_PIN_6,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Back Left Drive
	steppers[4] = (StepperMotor) {
		.step_port = GPIOD,
		.step_pin = GPIO_PIN_6,
		.dir_port = GPIOD,
		.dir_pin = GPIO_PIN_5,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[4] = (MotorControllerContext) {
		.id = 4,
		.stepper = &steppers[4],
		.enable_port = GPIOD,
		.enable_pin = GPIO_PIN_7,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Back Left Steer
	steppers[5] = (StepperMotor) {
		.step_port = GPIOD,
		.step_pin = GPIO_PIN_2,
		.dir_port = GPIOD,
		.dir_pin = GPIO_PIN_1,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[5] = (MotorControllerContext) {
		.id = 5,
		.stepper = &steppers[5],
		.enable_port = GPIOD,
		.enable_pin = GPIO_PIN_3,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Back Right Drive
	steppers[6] = (StepperMotor) {
		.step_port = GPIOC,
		.step_pin = GPIO_PIN_7,
		.dir_port = GPIOC,
		.dir_pin = GPIO_PIN_6,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[6] = (MotorControllerContext) {
		.id = 6,
		.stepper = &steppers[6],
		.enable_port = GPIOC,
		.enable_pin = GPIO_PIN_8,
		.enable_active_state = GPIO_PIN_RESET,
	};
	// Back Right Steer
	steppers[7] = (StepperMotor) {
		.step_port = GPIOD,
		.step_pin = GPIO_PIN_13,
		.dir_port = GPIOD,
		.dir_pin = GPIO_PIN_12,
		.mode = MODE_POS,
		.position = 0,
		.target = 0,
		.speed = 0,
		.enabled = 0,
		.step_high = 0
	};
	motorCtx[7] = (MotorControllerContext) {
		.id = 7,
		.stepper = &steppers[7],
		.enable_port = GPIOB,
		.enable_pin = GPIO_PIN_6,
		.enable_active_state = GPIO_PIN_RESET,
	};
}

//static uint8_t tmc_sw_spi_transfer(uint8_t data_out)
//{
//    uint8_t data_in = 0;
//
//    for (int i = 7; i >= 0; i--) {
//        // SCLK idle high. First pull low, set MOSI while low.
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
//
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,
//            (data_out & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
//
//        // Short delay may be needed if running very fast.
//        __NOP(); __NOP(); __NOP();
//
//        // Rising edge: TMC samples MOSI; MCU samples MISO.
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
//
//        __NOP(); __NOP(); __NOP();
//
//        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)) {
//            data_in |= (1 << i);
//        }
//    }
//
//    return data_in;
//}
//
//void tmc5160_write(GPIO_TypeDef *cs_port, uint16_t cs_pin,
//                   uint8_t reg, uint32_t value)
//{
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
//
//    tmc_sw_spi_transfer(reg | 0x80);        // write command
//    tmc_sw_spi_transfer(value >> 24);
//    tmc_sw_spi_transfer(value >> 16);
//    tmc_sw_spi_transfer(value >> 8);
//    tmc_sw_spi_transfer(value);
//
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
//}
//
///*
// * Read a TMC5160 register. Reads are pipelined: the first datagram latches the
// * address, and the *second* datagram returns its contents. We always issue two
// * frames so the value is valid on a cold read. The first byte clocked back on
// * any frame is the SPI status byte (returned via status_out).
// */
//uint32_t tmc5160_read(GPIO_TypeDef *cs_port, uint16_t cs_pin,
//                      uint8_t reg, uint8_t *status_out)
//{
//    uint8_t addr = reg & 0x7F;   // read access: MSB clear
//
//    // Frame 1: latch the address (returned data is stale, discard it).
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
//    tmc_sw_spi_transfer(addr);
//    tmc_sw_spi_transfer(0);
//    tmc_sw_spi_transfer(0);
//    tmc_sw_spi_transfer(0);
//    tmc_sw_spi_transfer(0);
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
//
//    // Frame 2: same address, now the value comes back.
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
//    uint8_t status = tmc_sw_spi_transfer(addr);
//    uint32_t value = 0;
//    value |= (uint32_t)tmc_sw_spi_transfer(0) << 24;
//    value |= (uint32_t)tmc_sw_spi_transfer(0) << 16;
//    value |= (uint32_t)tmc_sw_spi_transfer(0) << 8;
//    value |= (uint32_t)tmc_sw_spi_transfer(0);
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
//
//    if (status_out != NULL) {
//        *status_out = status;
//    }
//    return value;
//}
//
///*
// * Read back a few key registers from every TMC5160 so we can tell whether SPI
// * comms are actually working. On a healthy chip IOIN[31:24] (VERSION) reads
// * 0x30; a value of 0x00 or 0xFF means the SPI link (MISO/CS/wiring) is dead.
// */
//void DiagnoseSPIControllers(void)
//{
//    GPIO_TypeDef* cs_ports[5] = {GPIOE, GPIOE, GPIOB, GPIOD, GPIOD};
//    uint16_t cs_pins[5] = {GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_15};
//    const char *names[5] = {"X/s0", "Y/s1", "Z/s2", "E1/s4", "E3/s6"};
//
//    for (uint8_t i = 0; i < 5; i++) {
//        uint8_t st = 0;
//        // Clear GSTAT *now* (USB is up, so VS has had seconds to stabilise),
//        // then read it back. Any flag still set here is a live condition, not a
//        // stale latch left over from the power-up ramp.
//        tmc5160_write(cs_ports[i], cs_pins[i], 0x01, 0x00000007);
//
//        uint32_t ioin  = tmc5160_read(cs_ports[i], cs_pins[i], 0x04, &st);
//        uint32_t ihold  = tmc5160_read(cs_ports[i], cs_pins[i], 0x10, NULL);
//        uint32_t gconf = tmc5160_read(cs_ports[i], cs_pins[i], 0x00, NULL);
//        uint32_t gstat = tmc5160_read(cs_ports[i], cs_pins[i], 0x01, NULL);
//        uint32_t chop  = tmc5160_read(cs_ports[i], cs_pins[i], 0x6C, NULL);
//        uint32_t drv   = tmc5160_read(cs_ports[i], cs_pins[i], 0x6F, NULL);
//        printf("TMC %-5s VERSION=0x%02lX status=0x%02X IHOLD_IRUN=%08lX GCONF=0x%08lX "
//               "GSTAT=0x%lX CHOPCONF=0x%08lX DRV_STATUS=0x%08lX\r\n",
//               names[i], (ioin >> 24) & 0xFF, st, ihold,
//               gconf, gstat, chop, drv);
//    }
//}
//
//void ConfigureSPIControllers() {
//	// TODO: Dynamic Configuration
//	GPIO_TypeDef* cs_ports[5] = {GPIOE, GPIOE, GPIOB, GPIOD, GPIOD};
//	uint16_t cs_pins[5] = {GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_15};
//
//	// Put the shared SPI bus in its idle state before talking to any driver:
//	// SCLK high (mode 3 idle) and every CS de-asserted. This matters because at
//	// boot E3_CS (PD15) is driven low by MX_GPIO_Init, which would leave that
//	// chip selected and latching every byte meant for the other drivers.
//	HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);
//	for (uint8_t i = 0; i < 5; i++){
//		HAL_GPIO_WritePin(cs_ports[i], cs_pins[i], GPIO_PIN_SET);
//	}
//
//	for (uint8_t i = 0; i < 5; i++){
//		// CHOPCONF: MRES=16 microsteps (bits 27:24 = 0x4) plus intpol (bit 28)
//		// so the driver interpolates 16 usteps up to 256 internally -> smoother
//		// and much quieter than plain 16-ustep spreadCycle.
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x6C, 0x140100C3); // CHOPCONF (MRES=16, intpol)
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x0B, 0x00000080);
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x10, 0x00060701);
////		tmc5160_write(cs_ports[i], cs_pins[i], 0x0B, 0x00000080); // GLOBALSCALER=160
////		tmc5160_write(cs_ports[i], cs_pins[i], 0x10, 0x00061001); // IRUN=16, IHOLD=1
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x11, 0x0000000A);		// PWMCONF: reset-default value with pwm_autoscale + pwm_autograd enabled,
//		// required for stealthChop to self-tune. Must be set before en_pwm_mode.
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x70, 0xC40C001E); // PWMCONF
//		// GCONF: en_pwm_mode (bit 2) enables stealthChop -> near-silent chopper.
//		// With TPWMTHRS at its reset default (0) stealthChop stays active at all
//		// speeds. If you later need more high-speed torque, raise TPWMTHRS so the
//		// driver hands off to spreadCycle above that velocity.
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x00, 0x00000008); // GCONF (spreadCycle)
//		// Clear the latched GSTAT flags (reset / drv_err / uv_cp) by writing 1s.
//		// After this, any flag that reads back set is a *live* condition.
//		tmc5160_write(cs_ports[i], cs_pins[i], 0x01, 0x00000007); // GSTAT
//	}
//}

/*
 * Hand a message to the logger task instead of writing it here. Safe to call
 * from any task (message queues tolerate multiple writers) but NOT from an ISR.
 * Messages longer than LOG_MSG_MAX are truncated. Before logQueue exists - i.e.
 * during init, prior to osKernelStart - this falls through to a direct write,
 * which is fine because nothing is time-critical yet.
 */
static void LogDeferred(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    if (logQueue == NULL)
    {
        vprintf(fmt, args);
        va_end(args);
        return;
    }

    char msg[LOG_MSG_MAX];
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    if (osMessageQueuePut(logQueue, msg, 0, 0) != osOK)
    {
        log_drop_count++;
    }
}

/*
 * Single choke point for the scheduler queue so every producer is accounted
 * for. A nonzero drop count is a definite, unambiguous loss of a command.
 *
 * Queue depth is deliberately NOT sampled here. StartTask1 runs at a higher
 * priority than every producer, so it preempts and drains inside the put
 * itself - sampling on this side always reads back zero. The backlog is
 * measured from the consumer instead.
 */
static osStatus_t QueueCommand(const ControllerCommand *cmd)
{
    osStatus_t status = osMessageQueuePut(schedulerCommandQueue, cmd, 0, 0);

    if (status != osOK)
    {
        cmd_queue_drop_count++;
    }

    return status;
}

void HandleUsbCommand(const char *line)
{
    ControllerCommand cmd = {0};

    int motor_id;
    LogTrace("Parsing Command: %s\r\n", line);
    if (strncmp(line, "MOVEABS", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        // Batched absolute move: "MOVEABS <id> <pos> <speed> [<id> <pos> <speed> ...]".
        // Each triple is scanned with a trailing %n so we know how far the
        // cursor advanced, then queued as its own independent ControllerCommand
        // (same as a standalone single-motor MOVEABS) - the batch is
        // intentionally NOT atomic, so under queue pressure some motors can
        // be queued while others in the same line are dropped.
        const char *cursor = line + 7;
        int motors_queued = 0;
        int motors_failed = 0;

        for (;;) {
            int id;
            long pos;
            long spd;
            int consumed = 0;
            if (sscanf(cursor, " %d %ld %ld%n", &id, &pos, &spd, &consumed) != 3) {
                break;
            }
            cursor += consumed;

            cmd = (ControllerCommand){0};
            cmd.source = TRANSPORT_USB;
            cmd.type = CMD_MOVE_ABS;
            cmd.motor_id = id;
            cmd.target = pos;
            cmd.speed = spd;
            osStatus_t status = QueueCommand(&cmd);
            if (status == osOK) {
                LogTrace("OK QUEUED MOVEABS %d %ld %ld\r\n", id, pos, spd);
                motors_queued++;
            } else {
                LogDeferred("ERR QUEUE PUT FAILED status=%d motor=%d\r\n", (int)status, id);
                motors_failed++;
            }
        }

        if (motors_queued == 0 && motors_failed == 0) {
            LogDeferred("ERR INVALID MOVEABS SYNTAX\r\n");
        } else {
            LogDeferred("MOVEABS BATCH DONE queued=%d failed=%d\r\n", motors_queued, motors_failed);
        }
        return;
    }
    if (strncmp(line, "MOVEREL", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        // Batched relative move: "MOVEREL <id> <pos> <speed> [<id> <pos> <speed> ...]".
        // Each triple is scanned with a trailing %n so we know how far the
        // cursor advanced, then queued as its own independent ControllerCommand
        // (same as a standalone single-motor MOVEREL) - the batch is
        // intentionally NOT atomic, so under queue pressure some motors can
        // be queued while others in the same line are dropped.
        const char *cursor = line + 7;
        int motors_queued = 0;
        int motors_failed = 0;

        for (;;) {
            int id;
            long pos;
            long spd;
            int consumed = 0;
            if (sscanf(cursor, " %d %ld %ld%n", &id, &pos, &spd, &consumed) != 3) {
                break;
            }
            cursor += consumed;

            cmd = (ControllerCommand){0};
            cmd.source = TRANSPORT_USB;
            cmd.type = CMD_MOVE_REL;
            cmd.motor_id = id;
            cmd.target = pos;
            cmd.speed = spd;
            osStatus_t status = QueueCommand(&cmd);
            if (status == osOK) {
                LogTrace("OK QUEUED MOVEREL %d %ld %ld\r\n", id, pos, spd);
                motors_queued++;
            } else {
                LogDeferred("ERR QUEUE PUT FAILED status=%d motor=%d\r\n", (int)status, id);
                motors_failed++;
            }
        }

        if (motors_queued == 0 && motors_failed == 0) {
            LogDeferred("ERR INVALID MOVEREL SYNTAX\r\n");
        } else {
            LogDeferred("MOVEREL BATCH DONE queued=%d failed=%d\r\n", motors_queued, motors_failed);
        }
        return;
    }
    if (strncmp(line, "MOVESPEED", 9) == 0 && (line[9] == ' ' || line[9] == '\0')) {
        // Batched speed move: "MOVESPEED <id> <speed> [<id> <speed> ...]".
        // Each pair is scanned with a trailing %n so we know how far the
        // cursor advanced, then queued as its own independent ControllerCommand
        // (same as a standalone single-motor MOVESPEED) - the batch is
        // intentionally NOT atomic, so under queue pressure some motors can
        // be queued while others in the same line are dropped.
        const char *cursor = line + 9;
        int motors_queued = 0;
        int motors_failed = 0;

        for (;;) {
            int id;
            long spd;
            int consumed = 0;
            if (sscanf(cursor, " %d %ld%n", &id, &spd, &consumed) != 2) {
                break;
            }
            cursor += consumed;

            cmd = (ControllerCommand){0};
            cmd.source = TRANSPORT_USB;
            cmd.type = CMD_SET_SPEED;
            cmd.motor_id = id;
            cmd.target = spd < 0 ? NINF : INF;
            cmd.speed = spd;
            osStatus_t status = QueueCommand(&cmd);
            if (status == osOK) {
                LogTrace("OK QUEUED MOVESPEED %d %ld\r\n", id, spd);
                motors_queued++;
            } else {
                LogDeferred("ERR QUEUE PUT FAILED status=%d motor=%d\r\n", (int)status, id);
                motors_failed++;
            }
        }

        if (motors_queued == 0 && motors_failed == 0) {
            LogDeferred("ERR INVALID MOVESPEED SYNTAX\r\n");
        } else {
            LogDeferred("MOVESPEED BATCH DONE queued=%d failed=%d\r\n", motors_queued, motors_failed);
        }
        return;
    }
    if (strncmp(line, "STOP", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        // Batched stop: "STOP <id> [<id> ...]". Each id is scanned with a
        // trailing %n so we know how far the cursor advanced, then queued as
        // its own independent ControllerCommand (same as a standalone
        // single-motor STOP) - the batch is intentionally NOT atomic, so
        // under queue pressure some motors can be queued while others in the
        // same line are dropped.
        const char *cursor = line + 4;
        int motors_queued = 0;
        int motors_failed = 0;

        for (;;) {
            int id;
            int consumed = 0;
            if (sscanf(cursor, " %d%n", &id, &consumed) != 1) {
                break;
            }
            cursor += consumed;

            cmd = (ControllerCommand){0};
            cmd.source = TRANSPORT_USB;
            cmd.type = CMD_STOP;
            cmd.motor_id = id;
            osStatus_t status = QueueCommand(&cmd);
            if (status == osOK) {
                LogTrace("OK QUEUED STOP %d\r\n", id);
                motors_queued++;
            } else {
                LogDeferred("ERR QUEUE PUT FAILED status=%d motor=%d\r\n", (int)status, id);
                motors_failed++;
            }
        }

        if (motors_queued == 0 && motors_failed == 0) {
            LogDeferred("ERR INVALID STOP SYNTAX\r\n");
        } else {
            LogDeferred("STOP BATCH DONE queued=%d failed=%d\r\n", motors_queued, motors_failed);
        }
        return;
    }
    if (strncmp(line, "DISABLE", 7) == 0 && (line[7] == ' ' || line[7] == '\0')) {
        // Batched disable: "DISABLE <id> [<id> ...]". Each id is scanned with
        // a trailing %n so we know how far the cursor advanced, then queued
        // as its own independent ControllerCommand (same as a standalone
        // single-motor DISABLE) - the batch is intentionally NOT atomic, so
        // under queue pressure some motors can be queued while others in the
        // same line are dropped.
        const char *cursor = line + 7;
        int motors_queued = 0;
        int motors_failed = 0;

        for (;;) {
            int id;
            int consumed = 0;
            if (sscanf(cursor, " %d%n", &id, &consumed) != 1) {
                break;
            }
            cursor += consumed;

            cmd = (ControllerCommand){0};
            cmd.source = TRANSPORT_USB;
            cmd.type = CMD_DISABLE;
            cmd.motor_id = id;
            osStatus_t status = QueueCommand(&cmd);
            if (status == osOK) {
                LogTrace("OK QUEUED DISABLE %d\r\n", id);
                motors_queued++;
            } else {
                LogDeferred("ERR QUEUE PUT FAILED status=%d motor=%d\r\n", (int)status, id);
                motors_failed++;
            }
        }

        if (motors_queued == 0 && motors_failed == 0) {
            LogDeferred("ERR INVALID DISABLE SYNTAX\r\n");
        } else {
            LogDeferred("DISABLE BATCH DONE queued=%d failed=%d\r\n", motors_queued, motors_failed);
        }
        return;
    }
    if (sscanf(line, "STATUS %d", &motor_id) == 1){
		cmd.source = TRANSPORT_USB;
		cmd.type = CMD_STATUS;
		cmd.motor_id = motor_id;
		osStatus_t status = QueueCommand(&cmd);
		if (status == osOK) {
			LogTrace("OK QUEUED STATUS %d\r\n", motor_id);
		}
		else {
			LogDeferred("ERR QUEUE PUT FAILED status=%d\r\n", (int)status);
		}
		return;
	}
    cmd_invalid_count++;
    LogDeferred("Invalid Command Received.\r\n");
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
uint32_t encodeCommand(int16_t target, uint8_t dir, uint8_t speed, CommandType cmd){
	return (((((target << CMD_DIR_BITS) + (dir & CMD_DIR_MASK)) << CMD_SPEED_BITS) + speed) << CMD_TYPE_BITS) + cmd;
}

int16_t decodeSpeed(uint32_t input) {
	int dir = (input >> (CMD_SPEED_BITS + CMD_TYPE_BITS)) & CMD_DIR_MASK ? 1 : -1;
	return (dir * -1) * ((input >> CMD_TYPE_BITS) & CMD_SPEED_MASK);
}

CommandType decodeCommand(uint32_t input) {
	return input & CMD_TYPE_MASK;
}

int16_t decodeTarget(uint32_t input) {
	return (input >> (CMD_DIR_BITS + CMD_SPEED_BITS + CMD_TYPE_BITS)) & CMD_TARGET_MASK;
}

static void MotorController_HandlePosMove(
	MotorControllerContext *context,
	StepperMotor *stepper,
	uint32_t ulNotifiedValue,
	CommandType command,
	bool relative)
{
	int32_t target = decodeTarget(ulNotifiedValue);
	if (relative) {
		target += stepper->position;
	}
	stepper->target = target;
	stepper->mode = (command == CMD_SET_SPEED) ? MODE_SPEED : MODE_POS;
	stepper->ramp_accumulator = 0;
	if(command == CMD_SET_SPEED) {
		stepper->target *= 0xFFFF;
	}

	uint8_t speed = decodeSpeed(ulNotifiedValue);
	if (command == CMD_SET_SPEED && speed == 0) {
		stepper->commanded_speed = 0;
		stepper->target = stepper->position;
	} else {
		stepper->commanded_speed = speed == 0 ? DEFAULT_SPEED : abs(speed);
		if (command != CMD_SET_SPEED) {
			// Position moves snap straight to their commanded speed; only
			// speed-mode commands ramp gradually (see HAL_TIM_PeriodElapsedCallback).
			stepper->speed = stepper->commanded_speed;
		}
	}
	stepper->enabled = 1;
	stepper->accumulator = 0;
	HAL_GPIO_WritePin(context->enable_port, context->enable_pin, context->enable_active_state);
	HAL_GPIO_WritePin(
		stepper->dir_port,
		stepper->dir_pin,
		stepper->target < stepper->position ? GPIO_PIN_SET : GPIO_PIN_RESET
	);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* stdout is line-buffered (isatty()==1), so printf output without a trailing
   * '\n' would otherwise sit in the buffer and never reach USB. Make it
   * unbuffered so every printf is transmitted immediately. */
  // setvbuf(stdout, NULL, _IONBF, 0);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __set_BASEPRI(0U);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  MotorContexts_Init();

  /* Configure the TMC5160 drivers over SPI. Unlike the TMC2208s (which run
   * standalone from their pin/OTP defaults), the TMC5160 powers up with
   * CHOPCONF.TOFF = 0, i.e. the output stage disabled, so it ignores STEP/DIR
   * until we set the chopper + run current here. */
  ConfigureSPIControllers();

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

  /* Silence the ADC DMA completion interrupts.
   *
   * ADC1 free-runs (ContinuousConvMode) over 4 channels at 3-cycle sampling on
   * a 21 MHz ADC clock, so a full scan completes every ~2.9us. HAL_DMA_Start_IT
   * enables transfer-complete unconditionally and HAL_ADC_Start_DMA installs a
   * half-transfer callback, which together fired ~700k interrupts per second -
   * more than one per 240 CPU cycles, i.e. effectively the entire core spent
   * inside HAL_DMA_IRQHandler. Tasks were left with a fraction of a percent of
   * the CPU, which is why the logger's bit-banged TMC reads took ~0.9s apiece
   * instead of ~2ms, and why the same code is fast when it runs above (before
   * the ADC is started).
   *
   * Nothing needs these interrupts: the DMA writes encoder_adc[] circularly in
   * hardware and no conversion callback is implemented. Transfer-error and
   * direct-mode-error stay enabled so genuine faults are still caught. */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);
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
//  motorControllerHandle = osThreadNew(StartTask2, NULL, &motorController_attributes);

  /* creation of logger */
  loggerHandle = osThreadNew(StartTask03, NULL, &logger_attributes);

  /* creation of usbCommand */
  usbCommandHandle = osThreadNew(StartTask04, NULL, &usbCommand_attributes);

  /* creation of outputWriter */
  outputWriterHandle = osThreadNew(StartTask05, NULL, &outputWriter_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  for (int i = 0; i < NUM_STEPPERS; i++){
	  motorTaskHandles[i] = osThreadNew(StartTask2, &motorCtx[i], &motorController_attributes);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  schedulerCommandQueue = osMessageQueueNew(16, sizeof(ControllerCommand), NULL);
  logQueue = osMessageQueueNew(LOG_QUEUE_DEPTH, LOG_MSG_MAX, NULL);
  if (schedulerCommandQueue == NULL || logQueue == NULL)
  {
    Error_Handler();
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  htim2.Init.Period = 99;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Z_Enable_Pin|Y_CS_Pin|X_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Y_Dir_Pin|Y_Step_Pin|SCLK_Pin|MOSI_Pin
                          |Z_Dir_Pin|Z_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X_Dir_Pin|X_Step_Pin|E3_Dir_Pin|E3_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, XY_Enable_Pin|E3_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, E4_Dir_Pin|E4_Step_Pin|E2_Dir_Pin|E2_Step_Pin
                          |E1_Dir_Pin|E1_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, E3_CS_Pin|E2_Enable_Pin|E1_CS_Pin|E1_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E0_Dir_Pin|E0_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E04_Enable_Pin|Z_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Z_Enable_Pin */
  GPIO_InitStruct.Pin = Z_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Z_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_CS_Pin Y_Dir_Pin Y_Step_Pin X_CS_Pin
                           SCLK_Pin MOSI_Pin Z_Dir_Pin Z_Step_Pin */
  GPIO_InitStruct.Pin = Y_CS_Pin|Y_Dir_Pin|Y_Step_Pin|X_CS_Pin
                          |SCLK_Pin|MOSI_Pin|Z_Dir_Pin|Z_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : X_Dir_Pin X_Step_Pin E3_Dir_Pin E3_Step_Pin */
  GPIO_InitStruct.Pin = X_Dir_Pin|X_Step_Pin|E3_Dir_Pin|E3_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : XY_Enable_Pin E3_Enable_Pin */
  GPIO_InitStruct.Pin = XY_Enable_Pin|E3_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E4_Dir_Pin E4_Step_Pin E3_CS_Pin E2_Dir_Pin
                           E2_Step_Pin E1_CS_Pin E1_Dir_Pin E1_Step_Pin */
  GPIO_InitStruct.Pin = E4_Dir_Pin|E4_Step_Pin|E3_CS_Pin|E2_Dir_Pin
                          |E2_Step_Pin|E1_CS_Pin|E1_Dir_Pin|E1_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : E2_Enable_Pin E1_Enable_Pin */
  GPIO_InitStruct.Pin = E2_Enable_Pin|E1_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : E0_Dir_Pin E0_Step_Pin Z_CS_Pin */
  GPIO_InitStruct.Pin = E0_Dir_Pin|E0_Step_Pin|Z_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E04_Enable_Pin */
  GPIO_InitStruct.Pin = E04_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(E04_Enable_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * Record the reset cause into the no-init region and reboot. We deliberately do
 * NOT printf() from here: this hook runs in scheduler/exception context on an
 * already-corrupted stack, and USB transmit relies on IRQs we can't trust. The
 * cause is printed cleanly on the next boot once USB is healthy again.
 */
void RecordResetCauseAndReboot(uint32_t cause, const char *name)
{
    g_reset_report.magic = RESET_REPORT_MAGIC;
    g_reset_report.cause = cause;

    size_t i = 0;
    if (name != NULL)
    {
        for (; i < sizeof(g_reset_report.task) - 1 && name[i] != '\0'; i++)
        {
            g_reset_report.task[i] = name[i];
        }
    }
    g_reset_report.task[i] = '\0';

    __DSB();
    NVIC_SystemReset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    RecordResetCauseAndReboot(RESET_CAUSE_STACK_OVERFLOW, pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
    RecordResetCauseAndReboot(RESET_CAUSE_MALLOC_FAILED, "heap");
}

/* Print (and clear) any reset cause persisted from a previous crash. */
void ReportPreviousReset(void)
{
    if (g_reset_report.magic != RESET_REPORT_MAGIC)
    {
        return;
    }

    const char *cause_str = "UNKNOWN";
    if (g_reset_report.cause == RESET_CAUSE_STACK_OVERFLOW)
    {
        cause_str = "STACK OVERFLOW";
    }
    else if (g_reset_report.cause == RESET_CAUSE_MALLOC_FAILED)
    {
        cause_str = "MALLOC FAILED";
    }
    else if (g_reset_report.cause == RESET_CAUSE_HARDFAULT)
    {
        cause_str = "HARD FAULT";
    }

    LogDeferred("\r\n*** PREVIOUS RESET CAUSE: %s (task: '%s') ***\r\n",
                cause_str, g_reset_report.task);

    g_reset_report.magic = 0;
    g_reset_report.cause = 0;
    g_reset_report.task[0] = '\0';
}
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
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	ControllerCommand cmd;

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
			/* Backlog at the moment this consumer woke, counting the command
			 * just taken. Reads 1 whenever the consumer is keeping up; a
			 * climbing value means producers are outrunning dispatch. */
			uint32_t backlog =
				(uint32_t)osMessageQueueGetCount(schedulerCommandQueue) + 1U;
			if (backlog > cmd_queue_peak)
			{
				cmd_queue_peak = backlog;
			}

			/* A malformed id must not take the dispatcher down with it: this
			 * used to break out of the loop, which returns from the task and
			 * permanently stops all command dispatch. */
			if (cmd.motor_id >= NUM_STEPPERS)
			{
				LogDeferred("ERR BAD MOTOR ID %d\r\n", cmd.motor_id);
				continue;
			}

			if (motorTaskHandles[cmd.motor_id] == NULL)
			{
				LogDeferred("ERR MOTOR HANDLE NULL %d\r\n", cmd.motor_id);
				continue;
			}

			if (cmd.type == CMD_STATUS) {
				StepperMotor *stepper = motorCtx[cmd.motor_id].stepper;
				LogDeferred(
					"Motor (%d) Status: target=%ld, position=%ld, speed=%d, commanded_speed=%d, enabled=%d, mode=%s\r\n",
					cmd.motor_id,
					stepper->target,
					stepper->position,
					stepper->speed,
					stepper->commanded_speed,
					stepper->enabled,
					stepper->mode == MODE_POS ? "position" : "speed"
				);
				continue;
			}

			uint32_t notifyValue = encodeCommand(cmd.target, cmd.speed < 0, abs(cmd.speed), cmd.type);

			notify_sent[cmd.motor_id]++;

			BaseType_t result = xTaskNotify(
				motorTaskHandles[cmd.motor_id],
				notifyValue,
				eSetValueWithOverwrite
			);

			/* eSetValueWithOverwrite always succeeds, so only a failure is worth
			 * reporting - logging every notify just burned USB bandwidth. */
			if (result != pdPASS)
			{
				LogDeferred("ERR NOTIFY FAILED motor=%d\r\n", cmd.motor_id);
			}
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

	/* Infinite loop */
	for(;;)
	{
        if (xTaskNotifyWait(0, 0xFFFF, &ulNotifiedValue, osWaitForever)==pdTRUE){
        	notify_recv[context->id]++;
        	CommandType command = decodeCommand(ulNotifiedValue);
        	switch (command) {
        		case CMD_STOP: {
        			stepper->speed = 0;
        			stepper->commanded_speed = 0;
        			stepper->mode = MODE_SPEED;
        			stepper->enabled = 1;
        			HAL_GPIO_WritePin(context->enable_port, context->enable_pin, context->enable_active_state);
        			break;
        		}
        		case CMD_DISABLE: {
        			HAL_GPIO_WritePin(context->enable_port, context->enable_pin, !context->enable_active_state);
        			LogDeferred("Received Disable Command, set pin to: %d\r\n",
        			            !context->enable_active_state);
        			break;
        		}
        		case CMD_MOVE_REL: {
        			MotorController_HandlePosMove(
        				context, stepper, ulNotifiedValue, command, true);
        			break;
        		}
        		case CMD_SET_SPEED:
        		case CMD_MOVE_ABS: {
        			MotorController_HandlePosMove(
        				context, stepper, ulNotifiedValue, command, false);
        			break;
        		}
        		default:
        			break;
        	}
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
  /* Diagnostics only - this task performs no USB writes at all. Every line it
   * produces is handed to outputWriter, so the seconds it spends bit-banging
   * TMC5160 registers below can no longer hold up a command acknowledgement.
   * It is also the lowest-priority task in the system, so those register reads
   * only ever consume slack CPU. */
  /* Infinite loop */
  for(;;)
  {
	  if (log_drop_count != 0)
	  {
		  LogDeferred("WARN deferred log records dropped=%lu\r\n",
		              (unsigned long)log_drop_count);
	  }

	  /* Command-path accounting. Each stage that can lose a command reports
	   * here, so a drop can be attributed instead of guessed at:
	   *   rx_drop  - bytes the USB ISR could not fit into usbRxStream. Nonzero
	   *              means lines are being corrupted mid-flight, which shows up
	   *              as truncated batches or invalid syntax, not clean losses.
	   *   invalid  - lines that matched no command (a good corruption proxy).
	   *   qdrop    - commands rejected by a full scheduler queue.
	   *   qpeak    - deepest backlog seen by the dispatcher, out of 16. Stays
	   *              at 1 while dispatch keeps up with the producers.
	   *   coalesced- notifications overwritten before the motor task consumed
	   *              them, i.e. superseded commands. */
	  {
	    uint32_t sent_total = 0;
	    uint32_t recv_total = 0;
	    for (int i = 0; i < NUM_STEPPERS; i++)
	    {
	      sent_total += notify_sent[i];
	      recv_total += notify_recv[i];
	    }
	    LogDeferred("RX cb=%lu bytes=%lu rx_drop=%lu lines=%lu invalid=%lu\r\n",
	           (unsigned long)usb_rx_callback_count,
	           (unsigned long)usb_rx_byte_count,
	           (unsigned long)usb_rx_drop_count,
	           (unsigned long)usb_command_lines,
	           (unsigned long)cmd_invalid_count);
	    LogDeferred("CMD qdrop=%lu qpeak=%lu/16 notify_sent=%lu recv=%lu coalesced=%lu\r\n",
	           (unsigned long)cmd_queue_drop_count,
	           (unsigned long)cmd_queue_peak,
	           (unsigned long)sent_total,
	           (unsigned long)recv_total,
	           (unsigned long)(sent_total - recv_total));
	  }

	  /* 12-bit ADC (0..4095) mapped to a 0..360 degree angle. Buffer must be
	   * uint16_t to match the halfword DMA config in HAL_ADC_MspInit. */
	  uint32_t deg_x10[NUM_ENCODERS];
	  for (int i = 0; i < NUM_ENCODERS; i++)
	  {
	    deg_x10[i] = ((uint32_t)encoder_adc[i] * 3600U) / 4096U;
	  }
	  LogDeferred("ADC: IN4=%lu.%lu IN5=%lu.%lu IN6=%lu.%lu IN7=%lu.%lu\r\n",
	         deg_x10[0] / 10U, deg_x10[0] % 10U,
	         deg_x10[1] / 10U, deg_x10[1] % 10U,
	         deg_x10[2] / 10U, deg_x10[2] % 10U,
	         deg_x10[3] / 10U, deg_x10[3] % 10U);

	  /* Minimum free stack ever seen, in words (x4 = bytes). A value near 0
	   * means that task is about to overflow - bump its stack_size. */
	  LogDeferred("Stack free words: motor0=%lu logger=%lu usb=%lu out=%lu\r\n",
	         (unsigned long)uxTaskGetStackHighWaterMark(motorTaskHandles[0]),
	         (unsigned long)uxTaskGetStackHighWaterMark(NULL),
	         (unsigned long)uxTaskGetStackHighWaterMark(usbCommandHandle),
	         (unsigned long)uxTaskGetStackHighWaterMark(outputWriterHandle));

//	  DiagnoseSPIControllers();

	  /* Live TMC5160 microstep counter. MSCNT (0x6A) advances only when the
	   * driver actually receives STEP pulses, so if you command a move and the
	   * matching slot's value changes, STEP/DIR is reaching the driver; if it
	   * stays frozen, the pulses are not getting there (pin/enable/step-gen). */
	  {
	    GPIO_TypeDef* cs_ports[5] = {GPIOE, GPIOE, GPIOB, GPIOD, GPIOD};
	    uint16_t cs_pins[5] = {GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_15};
	    LogDeferred("MSCNT X=%lu Y=%lu Z=%lu E1=%lu E3=%lu\r\n",
	           tmc5160_read(cs_ports[0], cs_pins[0], 0x6A, NULL) & 0x3FF,
	           tmc5160_read(cs_ports[1], cs_pins[1], 0x6A, NULL) & 0x3FF,
	           tmc5160_read(cs_ports[2], cs_pins[2], 0x6A, NULL) & 0x3FF,
	           tmc5160_read(cs_ports[3], cs_pins[3], 0x6A, NULL) & 0x3FF,
	           tmc5160_read(cs_ports[4], cs_pins[4], 0x6A, NULL) & 0x3FF);

	    /* Decode the Y driver's DRV_STATUS (0x6F) so we can see what the driver
	     * itself thinks is happening while it is being stepped:
	     *   CS_ACTUAL  = actual current scale (should climb to IRUN=31 while
	     *                moving; if it stays low, run current is not applied)
	     *   ola/olb    = open load on coil A/B (broken/disconnected phase)
	     *   s2ga/s2gb/s2vsa/s2vsb = short to GND / supply on a coil
	     *   ot/otpw    = over-temp (shutdown / pre-warning)
	     *   stst       = standstill (no steps recently) */
	    uint32_t drv = tmc5160_read(cs_ports[1], cs_pins[1], 0x6F, NULL);
	    LogDeferred("Y DRV_STATUS=0x%08lX cs_actual=%lu stst=%lu ola=%lu olb=%lu "
	           "s2ga=%lu s2gb=%lu s2vsa=%lu s2vsb=%lu ot=%lu otpw=%lu\r\n",
	           drv,
	           (drv >> 16) & 0x1F,
	           (drv >> 31) & 0x1,
	           (drv >> 29) & 0x1,
	           (drv >> 30) & 0x1,
	           (drv >> 27) & 0x1,
	           (drv >> 28) & 0x1,
	           (drv >> 12) & 0x1,
	           (drv >> 13) & 0x1,
	           (drv >> 25) & 0x1,
	           (drv >> 26) & 0x1);
	  }

	  osDelay(DIAG_INTERVAL_MS);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the outputWriter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Sole owner of stdout. Everything else in the firmware queues text through
   * LogDeferred and returns immediately, so no task ever blocks on USB except
   * this one - and nothing depends on this one making progress.
   *
   * Being the only writer also matters for correctness: configUSE_NEWLIB_REENTRANT
   * is 0, so two tasks calling printf concurrently would share and corrupt a
   * single stdout buffer. */
  char msg[LOG_MSG_MAX];

  for(;;)
  {
	  if (osMessageQueueGet(logQueue, msg, NULL, osWaitForever) != osOK)
	  {
		  continue;
	  }

	  fputs(msg, stdout);
	  fflush(stdout);
  }
  /* USER CODE END StartTask05 */
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
	usbRxStream = xStreamBufferCreate(USB_RX_STREAM_SIZE, 1);
	usbTxStream = xStreamBufferCreate(USB_TX_STREAM_SIZE, 1);

	if (usbRxStream == NULL || usbTxStream == NULL)
	{
	  Error_Handler();
	}
	MX_USB_DEVICE_Init();

	/* Wait (up to ~3s) for the host to enumerate, then report any crash that
	 * caused the previous reset. _write drops output until USB is configured. */
	uint32_t enum_start = HAL_GetTick();
	while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED &&
	       (HAL_GetTick() - enum_start) < 3000)
	{
		osDelay(50);
	}
	osDelay(200);
	ReportPreviousReset();

	/* Dump each TMC5160's identity/status now that USB can carry the output. */
//	DiagnoseSPIControllers();

	char line[USB_LINE_MAX];
	size_t line_len = 0;
	uint8_t ch;
	for (;;)
	{
		/* Nothing in this loop may block on USB. The old per-character echo
		 * did exactly that - printf + fflush per byte meant one 1-byte USB
		 * packet, and so ~1ms, for every character of every command. */
		if (xStreamBufferReceive(usbRxStream, &ch, 1, portMAX_DELAY) == 1)
		{
			if (ch == '\n' || ch == '\r')
			{
				line[line_len] = '\0';

				if (line_len > 0)
				{
#if USB_ECHO_ENABLED
					LogDeferred("%s\r\n", line);
#endif
					usb_command_lines++;
					HandleUsbCommand(line);
				}

				line_len = 0;
			}
			else if (line_len < USB_LINE_MAX - 1)
			{
				line[line_len++] = (char)ch;
			}
			else
			{
				line_len = 0;
				LogDeferred("ERR LINE_TOO_LONG\r\n");
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
			if (steppers[i].target==steppers[i].position) {
				steppers[i].enabled = 0;
				continue;
			}
			if (steppers[i].mode == MODE_SPEED && steppers[i].speed != steppers[i].commanded_speed) {
				steppers[i].ramp_accumulator++;
				if (steppers[i].ramp_accumulator >= RAMP_ACCUMULATOR_THRESHOLD) {
					steppers[i].ramp_accumulator = 0;
					if (steppers[i].speed < steppers[i].commanded_speed) {
						steppers[i].speed += SPEED_RAMP_STEP;
						if (steppers[i].speed > steppers[i].commanded_speed)
							steppers[i].speed = steppers[i].commanded_speed;
					} else {
						steppers[i].speed -= SPEED_RAMP_STEP;
						if (steppers[i].speed < steppers[i].commanded_speed)
							steppers[i].speed = steppers[i].commanded_speed;
					}
				}
			} else {
				steppers[i].ramp_accumulator = 0;
			}
			if (steppers[i].step_high) {
				steppers[i].step_port->BSRR = (uint32_t)steppers[i].step_pin << 16; // STEP low
				steppers[i].step_high = 0;
				continue;
			}
			steppers[i].accumulator += steppers[i].speed;
			if (steppers[i].accumulator > MAX_SPEED) {
				steppers[i].accumulator -= MAX_SPEED;
				steppers[i].step_port->BSRR = steppers[i].step_pin; // STEP high
				steppers[i].step_high = 1;
				if (steppers[i].position < steppers[i].target)
					steppers[i].position++;
				else
					steppers[i].position--;
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
