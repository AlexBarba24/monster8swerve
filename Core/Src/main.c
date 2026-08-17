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
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
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
    CMD_STATUS = 0x7,
	CMD_POLL = 0x8
} CommandType;

typedef enum {
    TRANSPORT_USB = 0,
    TRANSPORT_CAN = 1
} TransportType;

/* `speed` is signed: for a jog its sign is the direction of travel, and for a
 * position move only its magnitude is used (direction comes from target versus
 * current position). Both fields are 32-bit all the way to the motor task, which
 * matters for `target` - one revolution of a 200-step motor at 256 microsteps is
 * already 51200 steps, so a 16-bit position cannot express even a single turn. */
typedef struct {
    TransportType source;
    CommandType type;
    uint8_t motor_id;
    int32_t target;
    int32_t speed;
    uint32_t flags;
} ControllerCommand;

/* One received CAN frame, as handed from the RX interrupt to canCommand. The
 * HAL's CAN_RxHeaderTypeDef is not used here because it is ~24 bytes of mostly
 * filter/timestamp metadata we discard, and this queue is copied per frame. */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;
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
/* Gates the CAN receive path. With this at 0 the filter is never configured and
 * HAL_CAN_Start is never called, so no frame can reach canCommand and no
 * telemetry is published - the tasks exist but stay idle. */
#define USE_CAN_COMMANDS 1

/* CAN control interface, using the FIRST/FRC addressing scheme. Identifiers are
 * 29-bit extended, partitioned into five fields:
 *
 *   bits 28:24  device type   (5 bits)
 *   bits 23:16  manufacturer  (8 bits)
 *   bits 15:10  API class     (6 bits)  \ together the 10-bit
 *   bits  9:6   API index     (4 bits)  / API/message identifier
 *   bits  5:0   device number (6 bits)
 *
 * Each motor is its own FRC device: the device number carries the motor id, so a
 * frame addresses exactly one motor and the entire 8-byte payload is free for a
 * 32-bit position and a 32-bit speed. That is how a Talon or SparkMax presents
 * itself, and it means an off-the-shelf FRC tool sees eight motors rather than
 * one opaque board.
 *
 * Every command therefore carries the same payload:
 *
 *   bytes 0:3  int32 target, little-endian
 *   bytes 4:7  int32 speed,  little-endian
 *
 * Commands that need only one of the two ignore the other, and commands that need
 * neither ignore the payload entirely. */
#define FRC_MAKE_ID(type, mfr, api, dev) \
    (((uint32_t)(type) << 24) | ((uint32_t)(mfr) << 16) | \
     ((uint32_t)(api) << 6) | (uint32_t)(dev))

#define FRC_DEVICE_TYPE(id)   (((id) >> 24) & 0x1Fu)
#define FRC_MANUFACTURER(id)  (((id) >> 16) & 0xFFu)
#define FRC_API_ID(id)        (((id) >> 6) & 0x3FFu)
#define FRC_API_CLASS(id)     (((id) >> 10) & 0x3Fu)
#define FRC_API_INDEX(id)     (((id) >> 6) & 0x0Fu)
#define FRC_DEVICE_NUMBER(id) ((id) & 0x3Fu)

/* Table 1 - CAN Device Types, and Table 2 - CAN Manufacturer Codes. */
#define FRC_DEVICE_TYPE_BROADCAST        0u
#define FRC_DEVICE_TYPE_MOTOR_CONTROLLER 2u
#define FRC_DEVICE_TYPE_ENCODER 		 7u
#define FRC_MANUFACTURER_BROADCAST       0u
#define FRC_MANUFACTURER_TEAM_USE        8u

/* This board's identity on the bus. It claims a block of eight consecutive device
 * numbers, one per motor, starting at CAN_MOTOR_ID_BASE: motor n answers to
 * device number CAN_MOTOR_ID_BASE + n.
 *
 * The base must be a multiple of 8 so that one hardware filter mask can accept
 * the whole block (see CAN_FILTER_MASK_MOTORS), which is what lets several of
 * these boards share a bus at bases 0, 8, 16 and so on. Base 0 is the FRC default.
 *
 * Device number 0x3F is reserved by the spec for device-specific broadcasts and is
 * also accepted, addressing every motor on the board at once. It is the block's
 * upper bound: a base of 56 would collide with it, so the last usable base is 48. */
#define CAN_DEVICE_TYPE   FRC_DEVICE_TYPE_MOTOR_CONTROLLER
#define CAN_MANUFACTURER  FRC_MANUFACTURER_TEAM_USE
#define CAN_MOTOR_ID_BASE 0u
#define FRC_DEVICE_NUMBER_BROADCAST 0x3Fu

/* Fixed payload layout shared by every command. */
#define CAN_PAYLOAD_LEN           8
#define CAN_PAYLOAD_TARGET_OFFSET 0
#define CAN_PAYLOAD_SPEED_OFFSET  4

/* API identifiers, as (class << 4) | index. Class numbering follows the
 * convention in the FRC motor-controller example so the device reads sensibly
 * to anyone familiar with the spec: 1 = speed control, 3 = position control,
 * 5 = status, 6 = periodic status. Class 0 holds the stop/disable controls.
 *
 * Lower identifiers win CAN arbitration, and because the API identifier sits
 * above the device number, this ordering means a stop beats motion traffic on a
 * saturated bus. */
#define CAN_API_STOP         0x000u /* class 0 index 0. Payload ignored */
#define CAN_API_DISABLE      0x001u /* class 0 index 1. Payload ignored */
#define CAN_API_SET_SPEED    0x010u /* class 1 index 0. Uses speed; target ignored */
#define CAN_API_MOVE_ABS     0x030u /* class 3 index 0. Uses target and speed */
#define CAN_API_MOVE_REL     0x031u /* class 3 index 1. Uses target and speed */
#define CAN_API_STATUS       0x050u /* class 5 index 0. Payload ignored */
#define CAN_API_ENCODER_REQ  0x051u /* class 5 index 1. Payload ignored, board-level */
#define CAN_API_TLM_ENCODERS 0x060u /* class 6 index 0. DLC 8: 4 x u16 ADC counts */
#define CAN_API_POLL_ENCODER 0x061u /* class 6 index 1. Frequency to report encoder values. */
/* Class 6 index 1 onward is reserved for per-motor status frames, which would
 * mirror the command payload: int32 position then int32 speed. */

/* Broadcast messages carry device type 0, manufacturer 0 and API class 0, so the
 * message number is the API index and Disable is arbitration ID 0x00000000 -
 * the lowest possible identifier, i.e. the highest priority frame on the bus.
 * The spec requires devices to disable immediately on this message. */
#define FRC_BCAST_DISABLE        0u
#define FRC_BCAST_SYSTEM_HALT    1u
#define FRC_BCAST_SYSTEM_RESET   2u
#define FRC_BCAST_DEVICE_ASSIGN  3u
#define FRC_BCAST_DEVICE_QUERY   4u
#define FRC_BCAST_HEARTBEAT      5u
#define FRC_BCAST_SYNC           6u
#define FRC_BCAST_UPDATE         7u
#define FRC_BCAST_FIRMWARE_VER   8u
#define FRC_BCAST_ENUMERATE      9u
#define FRC_BCAST_SYSTEM_RESUME 10u

/* Encoder telemetry covers all four steer channels in one frame, so it belongs to
 * the board rather than to any single motor. It goes out on the base device
 * number - motor 0's address - because a board-level frame still needs a device
 * number and that is the one that identifies this board's block. */
#define CAN_TX_ID_ENCODERS \
    FRC_MAKE_ID(CAN_DEVICE_TYPE, CAN_MANUFACTURER, CAN_API_TLM_ENCODERS, CAN_MOTOR_ID_BASE)

/* Receive filters, as {id, mask} pairs over the 29-bit identifier. A 1 in the
 * mask means the bit must match.
 *   0x1FFF0038 - device type + manufacturer + the top 3 bits of the device
 *                number, leaving the low 3 free: one bank accepts this board's
 *                whole aligned block of eight motors, any API identifier.
 *   0x1FFF003F - one exact device number, used for the all-motors address.
 *   0x1FFFFC00 - device type + manufacturer + API class, i.e. the whole
 *                broadcast class 0 regardless of message or device number. */
#define CAN_FILTER_MASK_MOTORS    0x1FFF0038u
#define CAN_FILTER_MASK_DEVICE    0x1FFF003Fu
#define CAN_FILTER_MASK_BROADCAST 0x1FFFFC00u

/* Frames buffered between the RX interrupt and canCommand. Eight is generous:
 * the hardware FIFO only holds three, and canCommand runs above every other
 * command producer so it drains the queue inside the put that filled it. */
#define CAN_RX_QUEUE_DEPTH 8

/* Encoder publish rate; 0 disables periodic publishing and leaves only the
 * on-request CAN_API_ENCODER_REQ path. At 100 Hz one 8-byte frame costs roughly
 * 135us of bus time per second (~1.4% of a 1 Mbit/s bus) and a few register
 * writes of CPU. That is orders of magnitude cheaper than publishing the same
 * data as a log line, which costs a vsnprintf, a 160-byte queue copy, and a
 * USB write that busy-waits up to 50ms. */
#define CAN_TLM_INTERVAL_MS 10

/* Set by canCommand to ask canTelemetry for an immediate extra publish. */
#define CAN_TLM_FLAG_PUBLISH 0x01u

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

/* Sentinel targets for jog commands, which run until countermanded. The step
 * generator stops a motor when target == position, so "forever" just means far
 * enough away that it never gets there: at the 5000 steps/s ceiling, 2.1e9 steps
 * is about five days of continuous running. The sign selects direction. */
#define TARGET_FOREVER_POS INT32_MAX
#define TARGET_FOREVER_NEG INT32_MIN

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

#define LOG_LEVEL 1

/* Per-command trace ("Parsing Command", "OK QUEUED ..."). An 8-motor batch
 * emits 10 such lines and each one is a separate ~1ms USB write, so above a few
 * command lines per second the log queue overflows. Dropped log records look
 * exactly like dropped commands from the host's point of view, which is why
 * this is off by default; errors, batch summaries and the periodic counters
 * below are always logged and are the reliable signal. */
#define CMD_LOG_VERBOSE 1

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

CAN_HandleTypeDef hcan1;

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
  .name = "logger",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usbCommand */
osThreadId_t usbCommandHandle;
const osThreadAttr_t usbCommand_attributes = {
  .name = "usbCommand",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for outputWriter */
osThreadId_t outputWriterHandle;
const osThreadAttr_t outputWriter_attributes = {
  .name = "outputWriter",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for canCommand */
osThreadId_t canCommandHandle;
const osThreadAttr_t canCommand_attributes = {
  .name = "canCommand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for canTelemetry */
osThreadId_t canTelemetryHandle;
const osThreadAttr_t canTelemetry_attributes = {
  .name = "canTelemetry",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* USER CODE BEGIN PV */
StepperMotor steppers[NUM_STEPPERS];
MotorControllerContext motorCtx[NUM_STEPPERS];
osThreadId_t motorTaskHandles[NUM_STEPPERS];
StreamBufferHandle_t usbRxStream;
StreamBufferHandle_t usbTxStream;
osMessageQueueId_t schedulerCommandQueue;
osMessageQueueId_t logQueue;
osMessageQueueId_t canRxQueue;

/* One depth-1 mailbox per motor, written with xQueueOverwrite. This replaces a
 * packed 32-bit task notification that could only carry a 16-bit target and an
 * 8-bit speed; the full ControllerCommand goes through instead, so the wire
 * format's 32-bit fields are honoured rather than silently truncated.
 *
 * Depth 1 with overwrite reproduces the old eSetValueWithOverwrite behaviour
 * exactly: a command arriving before the motor task has read the previous one
 * replaces it instead of queueing, so a motor always acts on the newest target
 * rather than working through a backlog of stale ones. Unlike a hand-rolled
 * shared struct it is also tear-free, which matters because the writer runs at a
 * higher priority than the reader and can preempt it mid-copy. */
QueueHandle_t motorCommandQueue[NUM_STEPPERS];
volatile uint32_t usb_rx_callback_count = 0;
volatile uint32_t usb_rx_byte_count = 0;
volatile uint32_t usb_rx_drop_count = 0;
volatile uint32_t usb_command_lines = 0;
volatile uint32_t log_drop_count = 0;
volatile uint32_t cmd_invalid_count = 0;
volatile uint32_t cmd_queue_drop_count = 0;
volatile uint32_t cmd_queue_peak = 0;

volatile uint32_t encoder_poll_rate = 0;
volatile uint32_t encoder_poll_acc = 0;

/* CAN command-path accounting, mirroring the USB counters above so a lost
 * command can be attributed to a stage rather than guessed at:
 *   rx_frame  - frames accepted by the filter and pulled out of FIFO0.
 *   rx_qdrop  - frames the ISR could not fit into canRxQueue. A definite loss.
 *   rx_invalid- unknown API identifier, bad DLC, bad motor id, or a standard-ID
 *               or remote frame that a misconfigured filter let through.
 *   queued    - per-motor commands successfully handed to the scheduler.
 *   tx_drop   - telemetry frames dropped because all mailboxes were busy.
 *   bcast     - FRC broadcast messages seen, handled or not.
 *   disable   - Disable/System Halt broadcasts acted on. A climbing count while
 *               motion is expected means something on the bus is holding the
 *               board disabled. */
volatile uint32_t can_rx_frame_count = 0;
/* Incremented by the CAN RX IRQ isolation path in stm32f4xx_it.c. This is the
 * storage definition; main.h exposes it to the interrupt translation unit. */
volatile uint32_t can_rx_isolation_irq_count = 0;
volatile uint32_t can_rx_isolation_read_count = 0;
volatile uint32_t can_rx_isolation_error_count = 0;
volatile uint32_t can_rx_isolation_queue_put_count = 0;
volatile uint32_t can_rx_isolation_queue_get_count = 0;
volatile uint32_t can_rx_isolation_queue_error_count = 0;
volatile uint32_t can_rx_isolation_parse_count = 0;
volatile uint32_t can_rx_isolation_command_count = 0;
volatile uint32_t can_rx_isolation_queue_command_count = 0;
volatile uint32_t can_rx_isolation_call_before_count = 0;
volatile uint32_t can_rx_isolation_call_entry_count = 0;
volatile uint32_t can_rx_isolation_call_after_count = 0;
volatile uint32_t can_rx_queue_drop_count = 0;
volatile uint32_t can_rx_invalid_count = 0;
volatile uint32_t can_cmd_queued_count = 0;
volatile uint32_t can_tx_drop_count = 0;
volatile uint32_t can_broadcast_count = 0;
volatile uint32_t can_disable_count = 0;

/* Commands handed to each motor task vs. commands it actually woke up for.
 * xQueueOverwrite replaces an unread command rather than queueing it, so a
 * persistent gap between these two totals is that coalescing, and it is
 * invisible otherwise. Each slot has exactly one writer, so no atomics are
 * needed. */
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
static void MX_CAN1_Init(void);
void StartTask1(void *argument);
void StartTask2(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);

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
#if CAN_RX_IRQ_ISOLATION_TEST && (CAN_COMMAND_PATH_TEST_STAGE == 2)
    /* Stage 5 isolation sink: the real ControllerCommand was constructed and
     * passed down the call chain. Inspect its fields without touching the
     * scheduler queue yet. */
    can_rx_isolation_call_entry_count++;
    if (cmd == NULL || cmd->source != TRANSPORT_CAN ||
        cmd->motor_id >= NUM_STEPPERS)
    {
        return osErrorParameter;
    }
    can_rx_isolation_queue_command_count++;
    return osOK;
#endif

	LogTrace(
	    "Command Issued: motor_id=%u type=%d target=%ld speed=%ld\r\n",
	    (unsigned)cmd->motor_id,
	    (int)cmd->type,
	    (long)cmd->target,
	    (long)cmd->speed
	);

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
            /* No target for a jog: the motor task derives both the direction and
             * the run-forever sentinel from the sign of the speed. */
            cmd.target = 0;
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

_Static_assert(NUM_ENCODERS * 2 <= 8,
               "encoder telemetry must fit one 8-byte CAN frame");

/* One filter mask covers an aligned block of eight device numbers, so the motor
 * block must be 8-aligned and must not run into the all-motors address 0x3F.
 * Together these limit the base to 0, 8, 16, 24, 32, 40 or 48. */
_Static_assert(NUM_STEPPERS <= 8,
               "one receive filter mask covers at most eight device numbers");
_Static_assert((CAN_MOTOR_ID_BASE % 8u) == 0u,
               "motor id base must be 8-aligned for the receive filter mask");
_Static_assert(CAN_MOTOR_ID_BASE + 8u <= FRC_DEVICE_NUMBER_BROADCAST,
               "motor id block must not reach the all-motors number 0x3F");

/*
 * Queue one command for one motor. Every CAN handler below funnels through here
 * so validation, counting and error reporting are identical across command types.
 */
static void CanQueueMotorCommand(
    CommandType type,
    uint8_t motor_id,
    int32_t target,
    int32_t speed)
{
    if (motor_id >= NUM_STEPPERS)
    {
        can_rx_invalid_count++;
        LogDeferred("ERR CAN BAD MOTOR ID %u\r\n", (unsigned)motor_id);
        return;
    }

#if CAN_RX_IRQ_ISOLATION_TEST
    /* HandleCanFrame successfully decoded and validated a motor command. The
     * deeper test sink is now inside QueueCommand(). */
    can_rx_isolation_command_count++;
#endif

    ControllerCommand cmd = {0};
    cmd.source = TRANSPORT_CAN;
    cmd.type = type;
    cmd.motor_id = motor_id;
    cmd.target = target;
    cmd.speed = speed;

#if CAN_RX_IRQ_ISOLATION_TEST && (CAN_COMMAND_PATH_TEST_STAGE == 1)
    /* Stage 5A: prove that constructing and populating the complete command is
     * safe. Volatile reads force the compiler to materialize every field before
     * returning, without entering QueueCommand(). */
    volatile uint32_t command_signature =
        (uint32_t)cmd.source ^ (uint32_t)cmd.type ^
        (uint32_t)cmd.motor_id ^ (uint32_t)cmd.target ^
        (uint32_t)cmd.speed;
    (void)command_signature;
    can_rx_isolation_queue_command_count++;
    return;
#endif

#if CAN_RX_IRQ_ISOLATION_TEST
    can_rx_isolation_call_before_count++;
#endif
    osStatus_t status = QueueCommand(&cmd);
#if CAN_RX_IRQ_ISOLATION_TEST
    can_rx_isolation_call_after_count++;
#endif
    if (status == osOK)
    {
        can_cmd_queued_count++;
        LogTrace("OK QUEUED CAN cmd=%d motor=%u\r\n", (int)type, (unsigned)motor_id);
    }
    else
    {
        LogDeferred("ERR CAN QUEUE PUT FAILED status=%d motor=%u\r\n",
                    (int)status, (unsigned)motor_id);
    }
}

static void CanReportBadFrame(const CanFrame *frame, const char *reason)
{
    can_rx_invalid_count++;
    /* The raw identifier is printed alongside the decoded API identifier because
     * the raw form is what a bus trace shows, and the API identifier is what the
     * protocol document lists. */
    LogDeferred("ERR CAN %s id=0x%08lX api=0x%03lX dlc=%u\r\n",
                reason, (unsigned long)frame->id,
                (unsigned long)FRC_API_ID(frame->id), (unsigned)frame->dlc);
}

static int32_t CanReadInt32(const uint8_t *bytes)
{
    /* Assembled byte by byte rather than by casting the buffer to an int32_t*.
     * CanFrame puts `data` at offset 5, so neither field is 4-byte aligned, and a
     * byte-wise read also states the wire's little-endian order outright instead
     * of inheriting it from the host. */
    return (int32_t)((uint32_t)bytes[0] |
                     ((uint32_t)bytes[1] << 8) |
                     ((uint32_t)bytes[2] << 16) |
                     ((uint32_t)bytes[3] << 24));
}

/*
 * Map an API identifier to an internal command, reporting whether the command
 * reads the payload at all. Split out so HandleCanFrame can reject an unknown
 * identifier before doing any payload validation.
 */
static bool CanCommandForApi(uint32_t api, CommandType *type, bool *needs_payload)
{
    switch (api)
    {
        case CAN_API_STOP:      *type = CMD_STOP;      *needs_payload = false; return true;
        case CAN_API_DISABLE:   *type = CMD_DISABLE;   *needs_payload = false; return true;
        case CAN_API_STATUS:    *type = CMD_STATUS;    *needs_payload = false; return true;
        case CAN_API_SET_SPEED: *type = CMD_SET_SPEED; *needs_payload = true;  return true;
        case CAN_API_MOVE_ABS:  *type = CMD_MOVE_ABS;  *needs_payload = true;  return true;
        case CAN_API_MOVE_REL:  *type = CMD_MOVE_REL;  *needs_payload = true;  return true;
        default: return false;
    }
}

/*
 * Publish the encoder channels as one frame of little-endian raw ADC counts.
 * Raw counts rather than degrees so the client owns the scaling and this stays a
 * handful of register writes: no formatting, no allocation, and nothing that can
 * block the command path.
 */
static void CanPublishEncoders(void)
{
    CAN_TxHeaderTypeDef header = {
        .StdId = 0,
        .ExtId = CAN_TX_ID_ENCODERS,
        .IDE = CAN_ID_EXT,
        .RTR = CAN_RTR_DATA,
        .DLC = NUM_ENCODERS * 2,
        .TransmitGlobalTime = DISABLE,
    };

    uint8_t payload[NUM_ENCODERS * 2];
    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        uint16_t raw = encoder_adc[i];
        payload[i * 2] = (uint8_t)(raw & 0xFFu);
        payload[i * 2 + 1] = (uint8_t)(raw >> 8);
    }

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox) != HAL_OK)
    {
        /* All three mailboxes busy, or the peripheral is bus-off. Telemetry is
         * expendable, so drop it and let the counter show the rate - the same
         * bargain the log queue makes. Retrying here would put bus health on the
         * critical path of a task that sits just below command handling. */
        can_tx_drop_count++;
    }
}

/*
 * Kill all motion right now, without going through any queue.
 *
 * The FRC spec requires a device to disable *immediately* on the Disable
 * broadcast, and the normal command path is four hops deep (RX queue ->
 * canCommand -> scheduler queue -> commandSchedule -> motor task) with a
 * bounded queue at each stage that is allowed to drop under pressure. Neither
 * the latency nor the possibility of a drop is acceptable for this message, so
 * it is applied directly here and is safe to call from the RX interrupt.
 *
 * `enabled` is cleared before anything else because that is the flag the TIM2
 * step generator checks first: once it is 0 that motor emits no more pulses, so
 * nothing the rest of the loop does can produce movement even if TIM2 (which
 * runs at a higher interrupt priority and can preempt this) fires part-way
 * through.
 */
static void CanDisableAllMotors(void)
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].enabled = 0;

        HAL_GPIO_WritePin(motorCtx[i].enable_port,
                          motorCtx[i].enable_pin,
                          !motorCtx[i].enable_active_state);

        /* A motor interrupted between the two halves of a step pulse would
         * otherwise leave STEP asserted for as long as the board stays disabled,
         * because clearing `enabled` above stops TIM2 reaching the code that
         * lowers it. */
        steppers[i].step_port->BSRR = (uint32_t)steppers[i].step_pin << 16;
        steppers[i].step_high = 0;

        steppers[i].speed = 0;
        steppers[i].commanded_speed = 0;
        steppers[i].target = steppers[i].position;
    }
}

/*
 * Dispatch one received frame. Runs in the canCommand task rather than the RX
 * interrupt so that parsing and error logging (LogDeferred calls vsnprintf and
 * is not ISR-safe) stay out of interrupt context. Broadcast frames never reach
 * here - they are handled in the interrupt itself.
 *
 * The device type and manufacturer were already matched by the hardware filter,
 * so only the API identifier and the device number are examined here.
 */
static void HandleCanFrame(const CanFrame *frame)
{
    uint32_t api = FRC_API_ID(frame->id);
    uint32_t device = FRC_DEVICE_NUMBER(frame->id);

    /* Board-level identifiers, which are not addressed to a motor. */
    if (api == CAN_API_ENCODER_REQ)
    {
        /* Delegated to canTelemetry rather than transmitted here so that every
         * HAL_CAN_AddTxMessage call comes from one task: it claims a mailbox with
         * a non-atomic read-then-write, and canCommand runs above canTelemetry so
         * it could preempt a publish in progress. */
        osThreadFlagsSet(canTelemetryHandle, CAN_TLM_FLAG_PUBLISH);
        return;
    }
    if (api == CAN_API_TLM_ENCODERS)
    {
        /* Our own telemetry identifier matches our receive filter, so it comes
         * back to us in the loopback modes used for bench testing. Ignored rather
         * than reported, so loopback stays quiet. */
        return;
    }

    if (FRC_DEVICE_TYPE(frame->id) == FRC_DEVICE_TYPE_ENCODER) {

    }

    CommandType type;
    bool needs_payload;
    if (!CanCommandForApi(api, &type, &needs_payload))
    {
        CanReportBadFrame(frame, "UNKNOWN API");
        return;
    }

    int32_t target = 0;
    int32_t speed = 0;

    if (needs_payload)
    {
        /* Exactly 8 bytes, never fewer. Zero-filling a short frame would silently
         * turn a truncated command into a valid-looking one - a move to 0 at
         * default speed - which is worse than rejecting it. */
        if (frame->dlc != CAN_PAYLOAD_LEN)
        {
            CanReportBadFrame(frame, "BAD DLC");
            return;
        }

        target = CanReadInt32(&frame->data[CAN_PAYLOAD_TARGET_OFFSET]);
        speed = CanReadInt32(&frame->data[CAN_PAYLOAD_SPEED_OFFSET]);

        /* The speed field is far wider than the achievable range, so an
         * out-of-range value is much more likely to be a byte-order or units
         * mistake than a real request. Reported rather than clamped, so it shows
         * up during bring-up instead of quietly running at the ceiling. */
        if (speed > MAX_SPEED || speed < -MAX_SPEED)
        {
            CanReportBadFrame(frame, "SPEED OUT OF RANGE");
            return;
        }
    }

    /* 0x3F addresses every motor on the board at once, so an all-stop is still a
     * single frame even though a command now names one motor. */
    if (device == FRC_DEVICE_NUMBER_BROADCAST)
    {
        for (uint8_t id = 0; id < NUM_STEPPERS; id++)
        {
            CanQueueMotorCommand(type, id, target, speed);
        }
        return;
    }

    /* Unsigned subtraction on purpose: a device number below the base wraps to a
     * huge value and fails the same bounds check, so one comparison covers both
     * ends. The filter should already have excluded anything outside the block -
     * this catches a misconfigured filter, and the tail of the block when
     * NUM_STEPPERS is less than 8. */
    uint32_t motor = device - CAN_MOTOR_ID_BASE;
    if (motor >= (uint32_t)NUM_STEPPERS)
    {
        CanReportBadFrame(frame, "BAD DEVICE NUMBER");
        return;
    }

    CanQueueMotorCommand(type, (uint8_t)motor, target, speed);
}


static void MotorController_HandlePosMove(
	MotorControllerContext *context,
	StepperMotor *stepper,
	const ControllerCommand *cmd,
	bool relative)
{
	int32_t target = cmd->target;

	if (relative) {
		/* Widened to 64-bit for the addition only. A relative move is now free to
		 * ask for a full int32 offset, so adding it to a position that is itself
		 * near the limit would otherwise wrap and send the motor the wrong way. */
		int64_t absolute = (int64_t)target + (int64_t)stepper->position;
		if (absolute > INT32_MAX) {
			absolute = INT32_MAX;
		} else if (absolute < INT32_MIN) {
			absolute = INT32_MIN;
		}
		target = (int32_t)absolute;
	}

	/* Only the magnitude is used here: for a position move the direction is
	 * implied by target versus position, and for a jog it is applied below by
	 * choosing which sentinel to run toward.
	 *
	 * Negated in 64-bit because -INT32_MIN is undefined in 32-bit. The CAN path
	 * rejects out-of-range speeds before this point, but a USB line can still
	 * scan any int32, and this clamp is what makes that safe. */
	int64_t requested = (cmd->speed < 0) ? -(int64_t)cmd->speed : (int64_t)cmd->speed;
	if (requested > MAX_SPEED) {
		requested = MAX_SPEED;
	}
	int32_t magnitude = (int32_t)requested;

	stepper->mode = (cmd->type == CMD_SET_SPEED) ? MODE_SPEED : MODE_POS;
	stepper->ramp_accumulator = 0;

	if (cmd->type == CMD_SET_SPEED) {
		if (magnitude == 0) {
			/* Speed 0 is a stop request: park the target where the motor already
			 * is so the step generator has nothing left to do. */
			stepper->commanded_speed = 0;
			target = stepper->position;
		} else {
			stepper->commanded_speed = (int16_t)magnitude;
			target = (cmd->speed < 0) ? TARGET_FOREVER_NEG : TARGET_FOREVER_POS;
		}
	} else {
		stepper->commanded_speed = (magnitude == 0) ? DEFAULT_SPEED : (int16_t)magnitude;
		// Position moves snap straight to their commanded speed; only
		// speed-mode commands ramp gradually (see HAL_TIM_PeriodElapsedCallback).
		stepper->speed = stepper->commanded_speed;
	}

	stepper->target = target;
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
  MX_CAN1_Init();
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

#if USE_CAN_COMMANDS
  /* MX_CAN1_Init only calls HAL_CAN_Init, which configures the bit timing and
   * leaves the peripheral in initialisation mode. The filter, the start and the
   * interrupt all still have to be set up here.
   *
   * ABOM (automatic bus-off recovery) is off in the .ioc, which would leave the
   * controller permanently silent after the 32nd consecutive error - a control
   * interface that dies quietly on a wiring fault. It is an
   * initialisation-mode-only bit and HAL_CAN_Start is what leaves that mode, so
   * this is the last opportunity to set it. */
  SET_BIT(hcan1.Instance->MCR, CAN_MCR_ABOM);

  /* Accept this board's own commands, the same commands sent to the reserved
   * all-motors number, and the FRC broadcast class. Everything else on the bus
   * - including the roboRIO heartbeat and other manufacturers' devices - is
   * discarded in hardware at zero CPU cost. */
  static const struct {
      uint32_t id;
      uint32_t mask;
  } can_filters[] = {
      { FRC_MAKE_ID(CAN_DEVICE_TYPE, CAN_MANUFACTURER, 0, CAN_MOTOR_ID_BASE),
        CAN_FILTER_MASK_MOTORS },
	{ FRC_MAKE_ID(FRC_DEVICE_TYPE_ENCODER, CAN_MANUFACTURER, 0, CAN_MOTOR_ID_BASE),
			CAN_FILTER_MASK_MOTORS },
      { FRC_MAKE_ID(CAN_DEVICE_TYPE, CAN_MANUFACTURER, 0, FRC_DEVICE_NUMBER_BROADCAST),
        CAN_FILTER_MASK_DEVICE },
      { FRC_MAKE_ID(FRC_DEVICE_TYPE_BROADCAST, FRC_MANUFACTURER_BROADCAST, 0, 0),
        CAN_FILTER_MASK_BROADCAST },
  };

  for (uint32_t i = 0; i < (sizeof(can_filters) / sizeof(can_filters[0])); i++)
  {
      /* In a 32-bit filter the identifier is left-aligned: EXID[28:0] occupies
       * bits 31:3, then IDE (bit 2) and RTR (bit 1). CAN_ID_EXT and
       * CAN_RTR_REMOTE are defined as exactly those bit values, so they double
       * as the flags here. Putting both in the mask makes the hardware drop
       * standard-ID and remote frames outright. */
      uint32_t id = (can_filters[i].id << 3) | CAN_ID_EXT;
      uint32_t mask = (can_filters[i].mask << 3) | CAN_ID_EXT | CAN_RTR_REMOTE;

      CAN_FilterTypeDef can_filter = {
          .FilterIdHigh = (uint16_t)(id >> 16),
          .FilterIdLow = (uint16_t)(id & 0xFFFFu),
          .FilterMaskIdHigh = (uint16_t)(mask >> 16),
          .FilterMaskIdLow = (uint16_t)(mask & 0xFFFFu),
          .FilterFIFOAssignment = CAN_FILTER_FIFO0,
          .FilterBank = i,
          .FilterMode = CAN_FILTERMODE_IDMASK,
          .FilterScale = CAN_FILTERSCALE_32BIT,
          .FilterActivation = CAN_FILTER_ENABLE,
          .SlaveStartFilterBank = 14,
      };

      if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
      {
          Error_Handler();
      }
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Error/bus-off notifications are enabled purely so HAL_CAN_GetError reflects
   * live bus health in the periodic diagnostics; nothing acts on them. */
  if (HAL_CAN_ActivateNotification(
          &hcan1,
          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR |
              CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK)
  {
      Error_Handler();
  }
#endif
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
  // motorControllerHandle = osThreadNew(StartTask2, NULL, &motorController_attributes);

  /* creation of logger */
  loggerHandle = osThreadNew(StartTask03, NULL, &logger_attributes);

  /* creation of usbCommand */
  usbCommandHandle = osThreadNew(StartTask04, NULL, &usbCommand_attributes);

  /* creation of outputWriter */
  outputWriterHandle = osThreadNew(StartTask05, NULL, &outputWriter_attributes);

  /* creation of canCommand */
  canCommandHandle = osThreadNew(StartTask06, NULL, &canCommand_attributes);

  /* creation of canTelemetry */
  canTelemetryHandle = osThreadNew(StartTask07, NULL, &canTelemetry_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  for (int i = 0; i < NUM_STEPPERS; i++){
	  motorTaskHandles[i] = osThreadNew(StartTask2, &motorCtx[i], &motorController_attributes);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  schedulerCommandQueue = osMessageQueueNew(16, sizeof(ControllerCommand), NULL);
  logQueue = osMessageQueueNew(LOG_QUEUE_DEPTH, LOG_MSG_MAX, NULL);
  canRxQueue = osMessageQueueNew(CAN_RX_QUEUE_DEPTH, sizeof(CanFrame), NULL);
  if (schedulerCommandQueue == NULL || logQueue == NULL || canRxQueue == NULL)
  {
    Error_Handler();
  }

  /* Created after the motor tasks above, which is safe only because nothing runs
   * until osKernelStart below. The native FreeRTOS API is used rather than
   * osMessageQueueNew because CMSIS-RTOS2 has no overwrite primitive. */
  for (int i = 0; i < NUM_STEPPERS; i++)
  {
    motorCommandQueue[i] = xQueueCreate(1, sizeof(ControllerCommand));
    if (motorCommandQueue[i] == NULL)
    {
      Error_Handler();
    }
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

/*
 * CAN receive, at NVIC priority 6. Does the minimum and returns: pull frames out
 * of FIFO0 into canRxQueue and let canCommand do the parsing. Nothing here
 * formats or logs, because LogDeferred calls vsnprintf and is not ISR-safe, so
 * losses are recorded in counters that the logger reports instead.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    CanFrame frame;

    /* The FIFO is drained rather than handled one frame per interrupt: it holds
     * three, and anything left behind would re-enter this handler immediately. */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        {
            break;
        }

        /* FRC uses extended identifiers exclusively. The filter already rejects
         * standard-ID and remote frames in hardware, so this only catches a
         * misconfigured filter. */
        if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA)
        {
            can_rx_invalid_count++;
            continue;
        }

        frame.id = header.ExtId;
        frame.dlc = (uint8_t)header.DLC;
        can_rx_frame_count++;

        /* Broadcast messages are answered here rather than being queued. Disable
         * must take effect immediately per the spec, and that guarantee is only
         * worth anything if it cannot sit behind other traffic or be dropped by
         * a full queue. System Halt is treated identically: both mean "stop
         * actuating now". The remaining broadcasts are optional and unimplemented,
         * so they are counted and discarded. */
        if (FRC_DEVICE_TYPE(frame.id) == FRC_DEVICE_TYPE_BROADCAST &&
            FRC_MANUFACTURER(frame.id) == FRC_MANUFACTURER_BROADCAST)
        {
            can_broadcast_count++;

            uint32_t message = FRC_API_INDEX(frame.id);
            if (message == FRC_BCAST_DISABLE || message == FRC_BCAST_SYSTEM_HALT)
            {
                can_disable_count++;
                CanDisableAllMotors();
            }
            continue;
        }

        /* CAN is started in main() before the queue is created, so a frame
         * arriving in that window has nowhere to go. Counting it as a drop is
         * correct and keeps the ISR free of ordering assumptions. */
        if (canRxQueue == NULL ||
            osMessageQueuePut(canRxQueue, &frame, 0, 0) != osOK)
        {
            can_rx_queue_drop_count++;
        }
    }
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
  MX_USB_DEVICE_Init();
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

			/* Guards the queue rather than the task handle, because the queue is
			 * what gets written below and a NULL handle would fault. */
			if (motorCommandQueue[cmd.motor_id] == NULL)
			{
				LogDeferred("ERR MOTOR QUEUE NULL %d\r\n", cmd.motor_id);
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

			notify_sent[cmd.motor_id]++;

			/* xQueueOverwrite on a depth-1 queue always succeeds, so only a
			 * failure is worth reporting - logging every command just burned USB
			 * bandwidth. */
			if (xQueueOverwrite(motorCommandQueue[cmd.motor_id], &cmd) != pdPASS)
			{
				LogDeferred("ERR MOTOR QUEUE FAILED motor=%d\r\n", cmd.motor_id);
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
	ControllerCommand cmd;
	MotorControllerContext *context = (MotorControllerContext*) argument;
	StepperMotor* stepper = context->stepper;

	/* Infinite loop */
	for(;;)
	{
        if (xQueueReceive(motorCommandQueue[context->id], &cmd, portMAX_DELAY) == pdTRUE){
        	notify_recv[context->id]++;
        	switch (cmd.type) {
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
        				context, stepper, &cmd, true);
        			break;
        		}
        		case CMD_SET_SPEED:
        		case CMD_MOVE_ABS: {
        			MotorController_HandlePosMove(
        				context, stepper, &cmd, false);
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


	#if LOG_LEVEL == 3

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

	  /* CAN transport health. err is HAL_CAN_GetError, which latches the last
	   * protocol error and the bus-off/passive state - a nonzero value with a
	   * climbing tx_drop usually means the board is talking to nobody (no node
	   * to ACK), not that the commands were bad. */
	  LogDeferred("CAN1 rx=%lu irq=%lu read=%lu rerr=%lu qput=%lu qget=%lu qerr=%lu\r\n",
	         (unsigned long)can_rx_frame_count,
	         (unsigned long)can_rx_isolation_irq_count,
	         (unsigned long)can_rx_isolation_read_count,
	         (unsigned long)can_rx_isolation_error_count,
	         (unsigned long)can_rx_isolation_queue_put_count,
	         (unsigned long)can_rx_isolation_queue_get_count,
	         (unsigned long)can_rx_isolation_queue_error_count);
	  LogDeferred("CAN2 parse=%lu cmd=%lu before=%lu entry=%lu after=%lu sink=%lu\r\n",
	         (unsigned long)can_rx_isolation_parse_count,
	         (unsigned long)can_rx_isolation_command_count,
	         (unsigned long)can_rx_isolation_call_before_count,
	         (unsigned long)can_rx_isolation_call_entry_count,
	         (unsigned long)can_rx_isolation_call_after_count,
	         (unsigned long)can_rx_isolation_queue_command_count
	  );
	  LogDeferred("CAN3 qdrop=%lu invalid=%lu queued=%lu tx_drop=%lu bcast=%lu dis=%lu err=0x%08lX\r\n",
	         (unsigned long)can_rx_queue_drop_count,
	         (unsigned long)can_rx_invalid_count,
	         (unsigned long)can_cmd_queued_count,
	         (unsigned long)can_tx_drop_count,
	         (unsigned long)can_broadcast_count,
	         (unsigned long)can_disable_count,
	         (unsigned long)HAL_CAN_GetError(&hcan1));

	  uint32_t esr = CAN1->ESR;

	  LogDeferred(
	      "CAN ESR=%08lX TEC=%lu REC=%lu LEC=%lu "
	      "BOFF=%lu EPVF=%lu EWGF=%lu\r\n",
	      (unsigned long)esr,
	      (unsigned long)((esr >> 16) & 0xFF),
	      (unsigned long)((esr >> 24) & 0xFF),
	      (unsigned long)((esr >> 4) & 0x07),
	      (unsigned long)((esr >> 2) & 0x01),
	      (unsigned long)((esr >> 1) & 0x01),
	      (unsigned long)(esr & 0x01)
	  );
	  /* Minimum free stack ever seen, in words (x4 = bytes). A value near 0
	  	   * means that task is about to overflow - bump its stack_size in the .ioc.
	  	   * heap is the FreeRTOS pool left out of configTOTAL_HEAP_SIZE; every task
	  	   * stack and queue is allocated from it, and logQueue alone takes
	  	   * LOG_QUEUE_DEPTH * LOG_MSG_MAX = 5KB of the 32KB. */
	  	  LogDeferred("Stack free words: motor0=%lu logger=%lu usb=%lu out=%lu can=%lu tlm=%lu heap=%lu\r\n",
	  	         (unsigned long)uxTaskGetStackHighWaterMark(motorTaskHandles[0]),
	  	         (unsigned long)uxTaskGetStackHighWaterMark(NULL),
	  	         (unsigned long)uxTaskGetStackHighWaterMark(usbCommandHandle),
	  	         (unsigned long)uxTaskGetStackHighWaterMark(outputWriterHandle),
	  	         (unsigned long)uxTaskGetStackHighWaterMark(canCommandHandle),
	  	         (unsigned long)uxTaskGetStackHighWaterMark(canTelemetryHandle),
	  	         (unsigned long)xPortGetFreeHeapSize());
	#endif

	#if LOG_LEVEL > 2
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
	#endif

//	  DiagnoseSPIControllers();

#if Log_Level == 3
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
#endif
	  osDelay(DIAG_INTERVAL_MS);

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

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the canCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Alternative control interface to the USB console, feeding the same
   * scheduler queue. It runs above usbCommand and outputWriter so a burst of
   * console text - or a host that has stopped reading, which makes a USB write
   * busy-wait for up to 50ms - can never delay a CAN command; and below
   * commandSchedule so that consumer still preempts this producer and drains the
   * queue inside the put that filled it. */
  CanFrame frame;

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(canRxQueue, &frame, NULL, osWaitForever) == osOK)
    {
#if CAN_RX_IRQ_ISOLATION_TEST
      /* Stage 4 isolation: run the real API/device/DLC parser. Its command sink
       * above prevents scheduler forwarding and motor actions. */
      can_rx_isolation_queue_get_count++;
      can_rx_isolation_parse_count++;
      HandleCanFrame(&frame);
#else
      HandleCanFrame(&frame);
#endif
    }
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the canTelemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
#if USE_CAN_COMMANDS
  /* Sole transmitter on the bus, for the same reason outputWriter is the sole
   * writer to stdout: HAL_CAN_AddTxMessage claims a mailbox with a
   * read-then-write that is not atomic against a higher-priority task doing the
   * same thing.
   *
   * The wait is relative rather than an osDelayUntil deadline so that the same
   * blocking call serves both the periodic cadence and an on-request
   * CAN_API_ENCODER_REQ. A request therefore re-phases the periodic stream, which
   * is harmless for independent samples, and the rate is "at least
   * CAN_TLM_INTERVAL_MS apart" - already the case anyway, since a 10ms timeout
   * on a 1kHz tick quantises to +/-1ms. */
//  const uint32_t wait =
//      (CAN_TLM_INTERVAL_MS > 0) ? CAN_TLM_INTERVAL_MS : osWaitForever;

  /* Infinite loop */
  for(;;)
  {
	if (encoder_poll_rate == 0) {
		osDelay(50);
		continue;
	}
    CanPublishEncoders();
    osDelay(1000 / encoder_poll_rate);
  }
#else
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
#endif
  /* USER CODE END StartTask07 */
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
