# monster8swerve

Firmware for an **STM32F407VET6** board that drives eight steppers (four drive,
four steer) for a swerve chassis. It runs on FreeRTOS. Commands arrive over USB
CDC (text, for bench work) or CAN1 at 1 Mbit/s (FRC addressing, for the
roboRIO). Both feeds share one motion scheduler.

## Contents

- [Usage](#usage)
- [Wiring](#wiring)
- [Flashing with STM32CubeProgrammer (USB)](#flashing-with-stm32cubeprogrammer-usb)
- [Serial console](#serial-console)
- [CAN protocol](#can-protocol)
- [Task hierarchy](#task-hierarchy)

## Usage

This is an STM32CubeIDE project (`.cproject` / `.project` / `monster8swerve.ioc`).
Build either from the IDE or from the generated makefile. The artifacts you
flash are `Debug/monster8swerve.elf` and `Debug/monster8swerve.bin`.

If you change peripherals in `monster8swerve.ioc`, regenerate code from CubeMX /
CubeIDE before rebuilding.

### STM32CubeIDE

1. Open STM32CubeIDE.
2. Import this folder: **File → Open Projects from File System…** (or
  **File → Import → Existing Projects into Workspace**) and select the
   `monster8swerve` root.
3. Select the `Debug` configuration.
4. Build with the hammer icon, or **Project → Build Project**.

That writes `monster8swerve.elf` / `.bin` / `.hex` under `Debug/`.

### Make

A generated makefile lives in `Debug/`:

```bash
cd Debug
make -j
```

On Windows, use the CubeIDE-bundled toolchain (the `Debug` makefile already
points at it), or run **make** from the CubeIDE Makefile project build. The
same three artifacts appear in `Debug/` when the build succeeds.

```bash
ls monster8swerve.elf monster8swerve.bin
```



## Firmware pin map

Enable pins are active-low. Some enables are shared between two motors.


| Id  | Role              | STEP   | DIR    | EN     |
| --- | ----------------- | ------ | ------ | ------ |
| 0   | Front left drive  | `PC14` | `PC13` | `PC15` |
| 1   | Front left steer  | `PE5`  | `PE4`  | `PC15` |
| 2   | Front right drive | `PE1`  | `PE0`  | `PE2`  |
| 3   | Front right steer | `PB5`  | `PB4`  | `PB6`  |
| 4   | Back left drive   | `PD6`  | `PD5`  | `PD7`  |
| 5   | Back left steer   | `PD2`  | `PD1`  | `PD3`  |
| 6   | Back right drive  | `PC7`  | `PC6`  | `PC8`  |
| 7   | Back right steer  | `PD13` | `PD12` | `PB6`  |



| Function        | MCU pins                                                                     |
| --------------- | ---------------------------------------------------------------------------- |
| USB CDC         | USB Full-Speed (virtual COM port)                                            |
| CAN1            | `PB8` (RX), `PB9` (TX), 1 Mbit/s — needs a transceiver and 120 Ω termination |
| Analog encoders | ADC1 IN4–IN7 on `PA4`–`PA7`                                                  |

The encoders connect to pins `PA4`-`PA7` and recieve 3.3V power from the 3v3 and GND pins
from the `EXP2` module. The `EXP2` layout can be found in the [pinout](https://github.com/makerbase-mks/MKS-Monster8/blob/0116434039f06b17c72ed5d1c43724a9d4a5d81b/hardware/MKS%20Monster8%20V2.0_003/MKS%20Monster8%20V2.0_003%20PIN.pdf).


## Flashing with STM32CubeProgrammer (USB)

You do not need an ST-Link. STM32CubeProgrammer can load the image over the
board's USB port using the STM32 ROM bootloader (USB DFU). That is the USB
connection type in CubeProgrammer; the chip has to be in **system-memory boot**,
not running the application from flash.

### 1. Put the board in USB DFU boot mode

Press the `BOOT0` and `RESET` Buttons simultaneously, a red light will indicate that the board is in DFU boot mode.

The chip should enumerate as an STM32 DFU device (`STM32 BOOTLOADER`), not as
the CDC serial port the firmware uses.

Windows: install the USB DFU driver that ships with STM32CubeProgrammer
(`DFU driver/STM32 Bootloader.bat`) if the device does not appear.

### 2. Connect CubeProgrammer over USB

1. Open **STM32CubeProgrammer**.
2. In the connection panel on the right, change the interface from *ST-LINK* to
  **USB**.
3. Click the refresh icon next to **Port**. You should see something like
  `USB1` with an STM32 DFU serial number.
4. Click **Connect**. The indicator turns green and the device information
  panel fills in (an F4 part, flash size, and so on).

If Port stays empty, the board is still booting from flash. Recheck BOOT0,
reset again, and confirm you are on the MCU USB port.

### 3. Download the image

1. **Open file** and choose `Debug/monster8swerve.elf` (or `.bin` / `.hex`).
2. Click **Download**.
3. Wait for the "File download complete" log line.
4. **Disconnect**.



### 4. Run the application

Press the reset button to run the flashed application. The board will automatically reload the flashed
firmware on startup.

It should now enumerate as a USB CDC serial port (see [Serial console](#serial-console)),
not as a DFU device.

## Serial console

After a normal (flash-boot) reset, plug the board's USB port into the host. It
enumerates as a CDC virtual COM port:

- Linux: `/dev/ttyACM0`
- macOS: `/dev/tty.usbmodem*`
- Windows: `COMx` (Device Manager → Ports)

For windows, connect to the board using `serial_terminal.py`, which will automatically
find and open the COM port for the monster8.

On macOS/Linux, Attach with `screen`:

```bash
# Linux / macOS
screen /dev/ttyACM0 115200
```

Detach with `Ctrl+A`, then `K`, then `Y`. `minicom` or PuTTY work as well.

Each command is one line ended by Enter (`\r` or `\n`). Characters are not
echoed (`USB_ECHO_ENABLED` is 0). Errors, batch summaries, and the periodic
diagnostics always print; per-command traces follow `CMD_LOG_VERBOSE`.

### Command table

Motor ids are `0`–`7`. Every command except `STATUS` can repeat its argument
group on one line to address several motors, e.g. `STOP 0 1 2 3`. Batches are
**not** atomic: under queue pressure some motors in the line can be accepted
while others are dropped.


| Command                         | Arguments                         | Description                                            |
| ------------------------------- | --------------------------------- | ------------------------------------------------------ |
| `MOVEABS {id} {target} {speed}` | target in steps (absolute), speed | Move to an absolute step position.                     |
| `MOVEREL {id} {target} {speed}` | target in steps (relative), speed | Move by an offset from the current position.           |
| `MOVESPEED {id} {speed}`        | signed speed                      | Jog continuously. Sign is direction; `0` stops.        |
| `STOP {id}`                     | —                                 | Stop (speed 0); driver stays energized.                |
| `DISABLE {id}`                  | —                                 | De-energize the driver (release enable).               |
| `STATUS {id}`                   | —                                 | Print target, position, speed, enabled flag, and mode. |


Position/`target` is a signed 32-bit step count, zeroed at boot. Speed is a
unitless rate clamped to `MAX_SPEED` (250). The step ISR runs at 10 kHz;
pulse rate is roughly `10000 * speed / (250 + speed)` steps/s (about 2.4 kHz at
speed 80, ceiling 5 kHz at 250). For position moves, speed `0` means the
default (80).

```text
MOVEABS 0 2000 80      # motor 0 to step 2000 at speed 80
MOVEREL 0 -400 120     # back off 400 steps
MOVESPEED 0 60         # jog forward
MOVESPEED 0 -60        # jog reverse
MOVESPEED 0 0          # stop jogging
STOP 0
STATUS 0
DISABLE 0
```

Typical replies:

```text
MOVEABS BATCH DONE queued=1 failed=0
Motor (0) Status: target=2000, position=2000, speed=80, commanded_speed=80, enabled=0, mode=position
```

An unrecognized line returns `Invalid Command Received.`

## CAN protocol

CAN1 runs at **1 Mbit/s** with **29-bit extended identifiers** in the
[FRC addressing format](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html).
All multi-byte fields are little-endian.

[CAN_PROTOCOL.md](CAN_PROTOCOL.md) is the full wire-format reference. This is
the working summary.

### Addressing


| Field             | Bits  | Value                                                       |
| ----------------- | ----- | ----------------------------------------------------------- |
| Device type       | 28:24 | **2** motor controller, or **7** encoder (telemetry / poll) |
| Manufacturer      | 23:16 | **8** (Team Use)                                            |
| API class / index | 15:6  | per command, below                                          |
| Device number     | 5:0   | motor id `0`–`7`                                            |


Motor identifier base is `0x0208_0000`:

```text
full_id = base | (api_id << 6) | motor_id
```

Each motor is its own FRC device. Device number `0x3F` means every motor on
this board. `CAN_MOTOR_ID_BASE` in `main.c` must stay a multiple of 8 so one
hardware filter covers the whole block (valid bases: 0, 8, … 48).

### Payload (commands)


| Bytes | Field           | Type       |
| ----- | --------------- | ---------- |
| 0–3   | Target position | `int32` LE |
| 4–7   | Speed           | `int32` LE |


`MOVESPEED` ignores target. `STOP` / `DISABLE` / `STATUS` ignore the payload
and accept any DLC. Motion commands require DLC **exactly 8**. Speed outside
±250 is rejected, not clamped.

### Commands (client → board)

Add the motor id, or `0x3F` for all eight.


| Motor 0 ID   | API     | Command      | Payload                                    |
| ------------ | ------- | ------------ | ------------------------------------------ |
| `0x02080000` | `0x000` | STOP         | none                                       |
| `0x02080040` | `0x001` | DISABLE      | none                                       |
| `0x02080400` | `0x010` | MOVESPEED    | speed (sign = direction)                   |
| `0x02080C00` | `0x030` | MOVEABS      | target + speed                             |
| `0x02080C40` | `0x031` | MOVEREL      | target + speed                             |
| `0x02081400` | `0x050` | STATUS       | none; reply is printed on USB, not CAN     |
| `0x02081440` | `0x051` | ENCODER_REQ  | none; one telemetry frame now              |
| `0x07081840` | `0x061` | POLL_ENCODER | `int32` rate in Hz (`0` pauses the stream) |


API classes follow the FRC motor-controller example: 1 speed, 3 position, 5
status, 6 periodic status. Class 0 is stop/disable, so a `STOP` still wins
arbitration against a `MOVEABS`.

### Broadcasts


| Full ID      | Message                    | Behaviour                                                       |
| ------------ | -------------------------- | --------------------------------------------------------------- |
| `0x00000000` | Disable                    | All motors stopped and de-energized **inside the RX interrupt** |
| `0x00000040` | System Halt                | Same as Disable                                                 |
| others       | Reset, Query, Heartbeat, … | Counted and ignored                                             |


Disable does **not** latch: the next motion command re-enables the board. There
is no roboRIO heartbeat timeout. If the client vanishes, motors hold their last
speed until you send `STOP` or the Disable broadcast.

### Telemetry (board → client)


| Full ID      | API     | Contents                           | DLC |
| ------------ | ------- | ---------------------------------- | --- |
| `0x07081800` | `0x060` | 4 × `u16` raw ADC counts (IN4–IN7) | 8   |


Device type **7** (encoder), manufacturer 8, base device number. Raw 12-bit
counts; the client owns scaling (`degrees = count * 360 / 4096`). Published at
`encoder_poll_rate` Hz (default 1) and also whenever `ENCODER_REQ` arrives.

### Examples (`cansend`)

Eight hex digits in the identifier selects an extended frame.

```bash
# Motor 0 to step 51200 at speed 80
cansend can0 02080C00#00C8000050000000

# Jog motor 0 forward at 60; motor 2 reverse at 60
cansend can0 02080400#000000003C000000
cansend can0 02080402#00000000C4FFFFFF

# Stop motor 0; stop all eight
cansend can0 02080000#
cansend can0 0208003F#

# One encoder frame now
cansend can0 02081440#

# FRC Disable
cansend can0 00000000#
```

A malformed frame is counted and logged to USB, not acknowledged on the bus.
With no other node to ACK, `AutoRetransmission` is off, so TX fails immediately
(`tx_drop` climbs). For a single-board bench test, temporarily set
`hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK` in `MX_CAN1_Init`.

Set `USE_CAN_COMMANDS` to `0` in `main.c` to leave CAN unstarted.

## Task hierarchy

Use this when adding work: pick a priority that preserves the producer /
consumer order, register the task in CubeMX, and keep STEP generation and
Disable handling in interrupts.

### Interrupts (always above every task)


| NVIC priority | IRQ        | Role                                                              |
| ------------- | ---------- | ----------------------------------------------------------------- |
| 5             | `TIM2`     | Step-pulse generator. Emits STEP independently of the scheduler.  |
| 6             | `CAN1_RX0` | Copies frames into `canRxQueue`. Applies FRC Disable immediately. |


`CAN1_RX0` is enabled in `HAL_CAN_MspInit`, not the CubeMX NVIC tab, and the
handler is hand-written in `stm32f4xx_it.c`. Do not enable it in CubeMX or you
will get duplicate symbols.

### FreeRTOS tasks (highest first)

CMSIS-RTOS2 numeric priorities, configured in the CubeMX task list (and the
matching `osThreadAttr_t` in `main.c`).


| Prio | CMSIS                    | Task                   | Role                                                              |
| ---- | ------------------------ | ---------------------- | ----------------------------------------------------------------- |
| 32   | `osPriorityAboveNormal`  | **motorController** ×8 | Applies one motor's queued command to stepper state.              |
| 24   | `osPriorityNormal`       | **commandSchedule**    | Reads the shared command queue; notifies the matching motor task. |
| 18   | `osPriorityBelowNormal2` | **canCommand**         | Parses CAN frames into commands.                                  |
| 17   | `osPriorityBelowNormal1` | **canTelemetry**       | Only task that transmits on CAN (encoder frames).                 |
| 16   | `osPriorityBelowNormal`  | **usbCommand**         | Assembles USB lines and parses them.                              |
| 15   | `osPriorityLow7`         | **outputWriter**       | Only task that writes USB; drains `logQueue`.                     |
| 8    | `osPriorityLow`          | **logger**             | Periodic diagnostics.                                             |


```text
USB CDC --> usbCommand --+
                         +--> scheduler queue --> commandSchedule --> motor task --> stepper state
CAN RX  --> canCommand --+                                              ^
                                                                        |
TIM2 (prio 5) ---------------------------------------------- STEP pulses +
CAN1_RX0 (prio 6) -- Disable in ISR; other frames --> canRxQueue

canTelemetry --> CAN TX (encoders)
logger --> logQueue --> outputWriter --> USB CDC IN
```

Rules that the current ranking encodes — keep them if you add a task:

1. **Consumer above producer** on every bounded queue, so the queue drains
  inside the `put` that filled it and does not build a backlog.
2. **Both CAN tasks above both USB tasks**, so a stuck USB host (a CDC write
  can busy-wait 50 ms) cannot delay machine control.
3. `canCommand` **above** `canTelemetry`, so an inbound command preempts an
  outbound publish. Only `canTelemetry` calls `HAL_CAN_AddTxMessage`.
4. **STEP stays in** `TIM2`, never in a task. Nothing at NVIC 5 or more urgent
  may run long enough to stretch a pulse.
5. **FRC Disable stays in** `CAN1_RX0`. It must not sit behind a full queue.



### CubeMX regeneration

All seven named tasks are in the CubeMX task list; application code lives in
`/* USER CODE */` blocks. If you add a task, register it in CubeMX as well —
a task created only in generated code disappears on the next regeneration.

The eight motor tasks are spawned from a `USER CODE` loop using the
`motorController` attributes. The CubeMX `motorController` create call is
intentionally commented out so you do not get a ninth empty instance.