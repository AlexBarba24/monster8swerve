# monster8swerve

Stepper-motor controller firmware for an **STM32F407VET6** board. It runs on
FreeRTOS and exposes a simple line-based command interface over a **USB virtual
COM port (CDC)**. You type text commands into a serial terminal and the board
moves the motor, reports status, and streams telemetry back.

The longer-term goal is multi-motor swerve-drive control; the current build is
configured for a single stepper (`NUM_STEPPERS = 1`, motor id `0`).

## Features

- USB CDC serial command interface (no extra UART/cable needed beyond USB).
- Step-pulse generation from a 10 kHz `TIM2` interrupt, so timing is independent
  of the command/logging tasks.
- Absolute / relative position moves, continuous speed (jog) mode, stop, and
  disable.
- 4-channel ADC sampling (intended for analog encoders) streamed via DMA.
- FreeRTOS tasks for command parsing, scheduling, motor control, and logging.
- Crash reporting: a stack overflow / malloc failure / hard fault is recorded in
  no-init RAM and printed on the next boot.

## Hardware

| Item        | Detail                                  |
|-------------|-----------------------------------------|
| MCU         | STM32F407VET6 (custom board)            |
| Clock       | 168 MHz (HSE + PLL)                     |
| Motor 0 STEP| `PD13`                                  |
| Motor 0 DIR | `PD12`                                  |
| Motor 0 EN  | `PB6` (active low)                      |
| Encoders    | ADC1 channels 4–7 (`PA4`–`PA7`)         |
| Console     | USB Full-Speed CDC (virtual COM port)   |

Wire STEP/DIR/EN to your stepper driver (e.g. a DRV8825/A4988-style module).
The enable pin is driven active-low.

## Building and flashing

This is an **STM32CubeIDE** project (`.cproject` / `.project` / `monster8swerve.ioc`).

### Option A — STM32CubeIDE (easiest)

1. Open STM32CubeIDE and import this folder (*File → Open Projects from File System*).
2. Build the `Debug` configuration (hammer icon).
3. Connect an ST-Link to the board's SWD header and click *Run* to flash.

### Option B — Command line

A generated makefile lives in `Debug/`:

```bash
cd Debug
make -j
```

This produces `monster8swerve.elf` / `.bin`. Flash with your tool of choice, e.g.:

```bash
st-flash write monster8swerve.bin 0x08000000
```

> Tip: if you edit peripheral configuration in `monster8swerve.ioc`, regenerate
> code from CubeMX/CubeIDE before rebuilding.

## Connecting to the command console

After flashing, plug the board's USB port into your computer. It enumerates as a
CDC virtual serial port:

- **Linux:** `/dev/ttyACM0`
- **macOS:** `/dev/tty.usbmodem*`
- **Windows:** a `COMx` port

Open it with any serial terminal (baud rate is ignored for USB CDC):

```bash
# Linux/macOS example
screen /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0
```

Each command is a single line terminated by Enter (`\r` or `\n`). The board
echoes characters as you type and replies with an `OK ...` or `ERR ...` line.

## Command reference

All commands target a motor by id. With the current build only id `0` is valid.

| Command | Arguments | Description |
|---------|-----------|-------------|
| `MOVEABS <id> <target> <speed>` | target in steps (absolute), speed | Move to an absolute step position. |
| `MOVEREL <id> <target> <speed>` | target in steps (relative), speed | Move by an offset from the current position. |
| `MOVESPEED <id> <speed>`        | signed speed | Run continuously (jog). Sign of speed sets direction; `0` stops. |
| `STOP <id>`                     | —            | Stop the motor (sets speed to 0). |
| `DISABLE <id>`                  | —            | De-energize the driver (releases the enable pin). |
| `STATUS <id>`                   | —            | Print target, position, speed, enabled flag, and mode. |

### Speed and position units

- **Position/`target`** is in motor steps; `position` is tracked by the firmware
  starting from `0` at boot.
- **Speed** is a unitless rate fed into the step accumulator. The step ISR runs
  at 10 kHz and emits a step whenever the accumulator exceeds `MAX_SPEED` (250),
  so the resulting pulse rate is roughly `10000 * speed / 250` steps/s (e.g.
  `speed = 80` ≈ 3.2 kHz). Speed is clamped to an 8-bit value, and `0` selects
  the default speed (80) for moves.

### Examples

```text
MOVEABS 0 2000 80      # move motor 0 to step 2000 at speed 80
MOVEREL 0 -400 120     # back off 400 steps from current position
MOVESPEED 0 60         # jog forward continuously
MOVESPEED 0 -60        # jog in reverse
MOVESPEED 0 0          # stop jogging
STOP 0                 # stop
STATUS 0               # query state
DISABLE 0              # release the driver
```

Typical responses:

```text
OK QUEUED MOVEABS 0 2000 80
Received Command for motor: 0!
Setting Stepper target: 2000, speed: 80
Motor (0) Status: target=2000, position=2000, speed=80, enabled=0, mode=position
```

An unrecognized line returns `Invalid Command Received.`

## Telemetry / logging

A low-priority logger task prints periodic lines (~every 10 s) to the same USB
console:

```text
ADC: IN4=2048 IN5=2050 IN6=2047 IN7=2049
Stack free words: motor0=180 logger=120 usb=540
```

- `ADC:` shows the raw 12-bit readings of the four encoder channels.
- `Stack free words:` is the minimum free stack ever seen per task (multiply by
  4 for bytes). A value near `0` means that task is close to overflowing — bump
  its `stack_size` in `main.c`.

On boot, if the previous run crashed you'll also see a line such as:

```text
*** PREVIOUS RESET CAUSE: STACK OVERFLOW (task: 'motorController') ***
```

## Firmware layout

| Path | Purpose |
|------|---------|
| `Core/Src/main.c` | Application: motor structs, command parsing, FreeRTOS tasks, step ISR. |
| `Core/Src/retarget_usb.c` | Routes `printf`/`_write` to USB CDC. |
| `USB_DEVICE/` | USB CDC device stack (ST middleware). |
| `Middlewares/Third_Party/FreeRTOS/` | FreeRTOS kernel. |
| `Drivers/` | STM32 HAL + CMSIS. |
| `Debug/` | Build output and generated makefile. |
| `monster8swerve.ioc` | STM32CubeMX project configuration. |

### Tasks

- **commandSchedule** — reads parsed commands from a queue and notifies the right
  motor task.
- **motorController** (one per motor) — applies move/speed/stop/disable commands
  to the stepper state.
- **usbCommand** — receives USB bytes, assembles lines, and parses them.
- **logger** — periodic ADC + stack telemetry.

The actual STEP pulses are generated in the `TIM2` period-elapsed interrupt, not
in a task.

## Notes and limitations

- Only motor id `0` is wired up (`NUM_STEPPERS = 1`). Sending another id returns
  an error. Additional motors can be added in `MotorContexts_Init()`.
- Speed is limited to 8 bits and clamped by `MAX_SPEED`.
- CAN command transport is stubbed out (`USE_CAN_COMMANDS = 0`); only USB is
  active.
