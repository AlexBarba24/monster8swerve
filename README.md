# monster8swerve

Stepper-motor controller firmware for an **STM32F407VET6** board. It runs on
FreeRTOS and drives eight steppers (four drive + four steer) for swerve-drive
control. Two command transports feed the same motion scheduler:

- a line-based text interface over a **USB virtual COM port (CDC)**, for humans
  and bench testing;
- a binary frame interface over **CAN1 at 1 Mbit/s** using **FRC/FIRST addressing**
  (manufacturer 8, one device number per motor), intended as the machine control
  path, which also publishes encoder telemetry back onto the bus.

## Features

- Two independent command transports (USB CDC text, CAN frames) sharing one
  command queue, so either can drive the machine and both can be used at once.
- Step-pulse generation from a 10 kHz `TIM2` interrupt, so timing is independent
  of the command/logging tasks.
- Absolute / relative position moves with full `int32` targets, continuous speed
  (jog) mode, stop, and disable. A USB line can address several motors at once; a
  CAN frame addresses one motor, or all eight via the reserved `0x3F` address.
- Honours the **FRC Disable broadcast**, applied inside the receive interrupt so it
  cannot queue behind other traffic or be dropped.
- 4-channel ADC sampling (analog encoders) streamed via DMA, published over CAN
  either periodically or on request.
- Crash reporting: a stack overflow / malloc failure / hard fault is recorded in
  no-init RAM and printed on the next boot.
- Per-stage drop counters for both transports, so a lost command can be
  attributed to a stage instead of guessed at.

## Hardware

| Item        | Detail                                  |
|-------------|-----------------------------------------|
| MCU         | STM32F407VET6 (custom board)            |
| Clock       | 168 MHz (HSE + PLL)                     |
| Encoders    | ADC1 channels 4–7 (`PA4`–`PA7`)         |
| Console     | USB Full-Speed CDC (virtual COM port)   |
| CAN         | CAN1 on `PB8` (RX) / `PB9` (TX), 1 Mbit/s |

`PB8`/`PB9` need a CAN transceiver (e.g. an SN65HVD230 or MCP2551 module) and a
120 Ω terminated bus; they are not a bus by themselves.

### Motor map

Even ids are drive motors, odd ids are steer motors. Enable pins are active-low
and some are shared between two motors.

| id | Position          | STEP   | DIR    | EN     |
|----|-------------------|--------|--------|--------|
| 0  | Front left drive  | `PC14` | `PC13` | `PC15` |
| 1  | Front left steer  | `PE5`  | `PE4`  | `PC15` |
| 2  | Front right drive | `PE1`  | `PE0`  | `PE2`  |
| 3  | Front right steer | `PB5`  | `PB4`  | `PB6`  |
| 4  | Back left drive   | `PD6`  | `PD5`  | `PD7`  |
| 5  | Back left steer   | `PD2`  | `PD1`  | `PD3`  |
| 6  | Back right drive  | `PC7`  | `PC6`  | `PC8`  |
| 7  | Back right steer  | `PD13` | `PD12` | `PB6`  |

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

Each command is a single line terminated by Enter (`\r` or `\n`). Characters are
not echoed back by default (`USB_ECHO_ENABLED` is `0`, since echoing costs a USB
transaction per line), and per-command acknowledgements are off by default too
(`CMD_LOG_VERBOSE` is `0`). Errors, batch summaries, and the periodic
diagnostics are always reported.

## Command reference

All commands target a motor by id `0`–`7` (see the motor map above).

| Command | Arguments | Description |
|---------|-----------|-------------|
| `MOVEABS <id> <target> <speed>` | target in steps (absolute), speed | Move to an absolute step position. |
| `MOVEREL <id> <target> <speed>` | target in steps (relative), speed | Move by an offset from the current position. |
| `MOVESPEED <id> <speed>`        | signed speed | Run continuously (jog). Sign of speed sets direction; `0` stops. |
| `STOP <id>`                     | —            | Stop the motor (sets speed to 0). |
| `DISABLE <id>`                  | —            | De-energize the driver (releases the enable pin). |
| `STATUS <id>`                   | —            | Print target, position, speed, enabled flag, and mode. |

Every command except `STATUS` accepts its argument group repeatedly to address
several motors on one line, e.g. `MOVEABS 0 2000 80 2 2000 80` or `STOP 0 1 2 3`.
The batch is deliberately **not** atomic: each motor is queued independently, so
under queue pressure some can be accepted while others in the same line are
dropped. The summary line reports how many of each.

### Speed and position units

- **Position/`target`** is a signed 32-bit count of motor steps; `position` is
  tracked by the firmware starting from `0` at boot.
- **Speed** is a unitless rate fed into the step accumulator. The step ISR runs at
  10 kHz and emits a pulse whenever the accumulator exceeds `MAX_SPEED` (250),
  with a pulse occupying two ticks, so the pulse rate is roughly
  `10000 * speed / (250 + speed)` steps/s — about 2.4 kHz at `speed = 80`, with a
  hard ceiling of 5 kHz at `speed = 250`. Speed is clamped to `MAX_SPEED`, and `0`
  selects the default speed (80) for position moves.

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
MOVEABS BATCH DONE queued=1 failed=0
Motor (0) Status: target=2000, position=2000, speed=80, commanded_speed=80, enabled=0, mode=position
```

An unrecognized line returns `Invalid Command Received.`

## CAN control interface

CAN1 runs at **1 Mbit/s** (42 MHz PCLK1 ÷ prescaler 3 ÷ 14 TQ) using
**29-bit extended identifiers in the [FRC addressing
format](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html)**.
All multi-byte fields are **little-endian**.

[CAN_PROTOCOL.md](CAN_PROTOCOL.md) is the full wire-format reference; this is the
summary.

### Addressing

| Field | Bits | Value |
|-------|------|-------|
| Device type | 28:24 | **2** — Motor Controller |
| Manufacturer | 23:16 | **8** — Team Use |
| API class / index | 15:6 | per command, below |
| Device number | 5:0 | **motor id**, 0–7 |

So the identifier base is `0x0208_0000`, and
`full_id = base | (api_id << 6) | motor_id`.

**Each motor is its own FRC device.** One frame commands one motor, the same shape
a Talon or SparkMax presents, which means any FRC tool walking the bus sees eight
motor controllers rather than one opaque board — and it leaves the whole 8-byte
payload free for a 32-bit position and a 32-bit speed.

`CAN_MOTOR_ID_BASE` moves the block of eight. It must be a multiple of 8 (enforced
by a `_Static_assert`) because one filter mask accepts the whole aligned block, so
valid bases are 0, 8, 16 … 48 and seven boards can share a bus. Device number
`0x3F` is the spec's device-specific broadcast address and is accepted as **"every
motor on this board"**, which keeps an all-stop down to one frame.

### Payload

Every command uses the same 8 bytes:

| Bytes | Field | Type |
|-------|-------|------|
| 0–3 | Target position | `int32` little-endian |
| 4–7 | Speed | `int32` little-endian |

`MOVESPEED` ignores the target; `STOP`, `DISABLE` and `STATUS` ignore the payload
entirely and accept any DLC. Motion commands require DLC exactly 8 — a short frame
is rejected rather than zero-filled, since zero-filling would turn a truncated
command into a plausible move-to-zero.

### Commands (client → board)

Add the motor id to each identifier, or `0x3F` to address all eight.

| Motor 0 ID | API | Command | Payload used |
|------------|-----|---------|--------------|
| `0x02080000` | `0x000` | STOP | none |
| `0x02080040` | `0x001` | DISABLE | none |
| `0x02080400` | `0x010` | MOVESPEED | speed (sign = direction) |
| `0x02080C00` | `0x030` | MOVEABS | target + speed |
| `0x02080C40` | `0x031` | MOVEREL | target + speed |
| `0x02081400` | `0x050` | STATUS | none; reply goes to the USB log, not the bus |
| `0x02081440` | `0x051` | ENCODER_REQ | none; board-level, always the base device number |

API classes follow the convention in the spec's motor-controller example: 1 is
speed control, 3 is position control, 5 is status, 6 is periodic status, and class
0 holds the stop/disable controls. Because the API identifier sits above the
device number, a `STOP` for any motor still wins arbitration against a `MOVEABS`
for any motor.

API `0x061`+ is reserved for per-motor status frames, which would mirror the
command payload. The receive filter accepts every API identifier already, so
adding one needs no filter change.

### Broadcast messages

| Full ID | Message | Behaviour |
|---------|---------|-----------|
| `0x00000000` | Disable | All motors stopped and de-energised **inside the RX interrupt** |
| `0x00000040` | System Halt | Treated identically to Disable |
| others | Reset, Query, Heartbeat, … | Counted and ignored |

Disable is arbitration ID 0, the highest-priority frame that can exist on a CAN
bus, and the spec requires devices to act on it immediately. It is therefore
handled in the interrupt rather than queued: the normal command path is four hops
deep through bounded queues that may drop frames under load, and neither the
latency nor the possibility of a drop is acceptable here. Each motor's `enabled`
flag is cleared first, because that is the flag the step ISR checks before
emitting a pulse.

**Disable does not latch** — the next motion command re-enables the board. Proper
FRC enable gating uses the roboRIO universal heartbeat (`0x01011840`, every 20 ms),
which this firmware does not implement; see
[CAN_PROTOCOL.md](CAN_PROTOCOL.md#disable-is-not-latching) for why a latch without
heartbeat support would be worse.

Positions are full `int32` all the way to the motor. That matters rather than
being theoretical: a 200-step motor at 256 microsteps is 51,200 steps per
revolution, so the `int16` target of the previous protocol revision could not
express even one full turn. Speeds are still limited to ±250 (`MAX_SPEED`), and a
value outside that range is rejected and logged rather than clamped.

A frame sent to `0x3F` expands to eight independent commands, so it is **not
atomic**: under scheduler-queue pressure some motors can be accepted while others
are dropped, the same as a batched USB line.

### Telemetry (board → client)

| Full ID | API | Contents | DLC |
|---------|-----|----------|-----|
| `0x02081800` | `0x060` | 4 × `u16` raw encoder ADC counts (channels IN4–IN7) | 8 |

Raw 12-bit counts, not degrees, so the client owns the scaling: `degrees =
count * 360 / 4096`.

This is board-level rather than per-motor, and goes out on the base device number.
Four channels in one frame costs a quarter of the bus time of four per-motor
frames, and with only three hardware transmit mailboxes a four-frame burst at
100 Hz would drop frames.

Published every `CAN_TLM_INTERVAL_MS` (default 10 ms = 100 Hz) and additionally
whenever an `ENCODER_REQ` arrives. Set `CAN_TLM_INTERVAL_MS` to `0` for
request-only polling. At 100 Hz this costs roughly 1.6% of the bus and a
negligible slice of CPU — publishing over CAN avoids the `vsnprintf`, queue copy,
and potentially 50 ms blocking write that the same data would cost as a USB log
line.

### Receive filter

Three mask-mode filter banks feed FIFO0:

| Match | Mask | Accepts |
|-------|------|---------|
| `0x02080000` | `0x1FFF0038` | Any API identifier for any of this board's eight motors |
| `0x0208003F` | `0x1FFF003F` | Any API identifier sent to all motors |
| `0x00000000` | `0x1FFFFC00` | The whole broadcast class |

The first bank is the interesting one: masking bits 5:3 of the device number and
leaving bits 2:0 free accepts an aligned block of exactly eight device numbers, so
all eight motors cost one filter bank rather than eight. That is why
`CAN_MOTOR_ID_BASE` has to be 8-aligned.

The device masks also cover the `IDE` and `RTR` bits, so standard-identifier and
remote frames are rejected in hardware. Everything else on the bus — other
manufacturers, other boards' blocks, the roboRIO heartbeat — is dropped at zero
CPU cost.

Set `USE_CAN_COMMANDS` to `0` in `main.c` to leave the peripheral unstarted and
the CAN tasks idle.

### Examples

Using `cansend` from can-utils on a Linux host:

Using `cansend` from can-utils on a Linux host. Eight hex digits in the
identifier is what selects an extended frame; three digits would send a standard
frame that the board rejects.

```bash
# Move motor 0 to step 51200 at speed 80
#   51200 = 0x0000C800 -> bytes 00 C8 00 00 ;  80 -> 50 00 00 00
cansend can0 02080C00#00C8000050000000

# Move motor 3 to step -1000 at the default speed (speed field 0)
#   -1000 = 0xFFFFFC18 -> bytes 18 FC FF FF
cansend can0 02080C03#18FCFFFF00000000

# Jog motor 0 forward at 60, motor 2 in reverse at 60 (-60 = 0xFFFFFFC4)
cansend can0 02080400#000000003C000000
cansend can0 02080402#00000000C4FFFFFF

# Stop motor 0
cansend can0 02080000#

# Stop all eight motors in one frame
cansend can0 0208003F#

# Request one encoder frame
cansend can0 02081440#

# FRC Disable broadcast: every device on the bus stops immediately
cansend can0 00000000#
```

A malformed frame (unknown API identifier, DLC other than 8 on a motion command,
speed outside ±250, device number outside the block) is counted and logged to the
USB console rather than acknowledged on the bus.

### Bench testing without a second node

`AutoRetransmission` is disabled, so with no other node on the bus to ACK, every
transmitted frame fails immediately and shows up as a climbing `tx_drop` and a
nonzero `err` in the diagnostics. To exercise the code with only the board
connected, temporarily set `hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK` in
`MX_CAN1_Init`.

Automatic bus-off recovery (`ABOM`) is enabled in software before
`HAL_CAN_Start`, so a wiring fault that takes the controller bus-off recovers on
its own rather than silently killing the interface.

## Telemetry / logging

A low-priority logger task prints periodic lines (every `DIAG_INTERVAL_MS`,
default 1 s) to the USB console:

```text
RX cb=412 bytes=1980 rx_drop=0 lines=96 invalid=0
CMD qdrop=0 qpeak=1/16 notify_sent=96 recv=96 coalesced=0
CAN rx=1204 qdrop=0 invalid=0 queued=1204 tx_drop=0 bcast=0 dis=0 err=0x00000000
ADC: IN4=180.1 IN5=180.2 IN6=179.9 IN7=180.0
Stack free words: motor0=180 logger=120 usb=540 out=210 can=88 tlm=70 heap=5120
```

- `RX` / `CMD` account for the USB command path: bytes the ISR could not buffer,
  lines that matched no command, commands rejected by a full scheduler queue,
  the deepest queue backlog seen, and notifications overwritten before a motor
  task consumed them.
- `CAN` is the same accounting for the CAN path. `bcast` counts FRC broadcast
  messages seen and `dis` counts the Disable/System Halt broadcasts acted on, so a
  climbing `dis` while you expect motion means something on the bus is holding the
  board disabled. `err` is `HAL_CAN_GetError`; a nonzero value alongside a
  climbing `tx_drop` usually means nothing is on the bus to ACK, not that commands
  were bad.
- `ADC:` shows the four encoder channels scaled to degrees.
- `Stack free words:` is the minimum free stack ever seen per task (multiply by
  4 for bytes). A value near `0` means that task is close to overflowing — bump
  its stack size in the CubeMX task list. `heap` is the FreeRTOS pool left free
  out of `configTOTAL_HEAP_SIZE`; `logQueue` alone accounts for 5 KB of the 32 KB.

On boot, if the previous run crashed you'll also see a line such as:

```text
*** PREVIOUS RESET CAUSE: STACK OVERFLOW (task: 'motorController') ***
```

## Firmware layout

| Path | Purpose |
|------|---------|
| `Core/Src/main.c` | Application: motor structs, USB + CAN command parsing, FreeRTOS tasks, step ISR. |
| `Core/Src/retarget_usb.c` | Routes `printf`/`_write` to USB CDC. |
| `Core/Src/tmcSPI.c` | Bit-banged SPI configuration and diagnostics for the TMC5160 drivers. |
| `USB_DEVICE/` | USB CDC device stack (ST middleware). |
| `Middlewares/Third_Party/FreeRTOS/` | FreeRTOS kernel. |
| `Drivers/` | STM32 HAL + CMSIS. |
| `Debug/` | Build output and generated makefile. |
| `monster8swerve.ioc` | STM32CubeMX project configuration. |

### Tasks

Listed highest priority first. The numbers are CMSIS-RTOS2 priorities and are
configured in the CubeMX task list, not by hand in `main.c`.

| Priority | Task | Role |
|----|------------------|------|
| 32 | **motorController** ×8 | Applies move/speed/stop/disable commands to one stepper's state. |
| 24 | **commandSchedule** | Reads commands from the shared queue and notifies the right motor task. |
| 18 | **canCommand** | Parses CAN frames into commands. |
| 17 | **canTelemetry** | Publishes encoder frames; the only task that transmits on CAN. |
| 16 | **usbCommand** | Receives USB bytes, assembles lines, parses them. |
| 15 | **outputWriter** | Drains `logQueue` to stdout; the only task that writes to USB. |
| 8  | **logger** | Periodic diagnostics. |

The ordering is deliberate. Each stage of a command path runs **above** its
producer, so a queue is drained inside the `put` that filled it and never builds
a backlog. Both CAN tasks sit above both USB tasks, so a slow or absent USB host
cannot delay the machine control path — relevant because a USB write busy-waits
up to 50 ms when the endpoint is busy. `canCommand` sits above `canTelemetry` so
an inbound command always preempts an outbound publish.

The actual STEP pulses are generated in the `TIM2` period-elapsed interrupt, not
in a task. `CAN1_RX0` is at NVIC priority 6, one step below `TIM2`'s 5, so a CAN
interrupt can never delay a step pulse.

### A note on CubeMX regeneration

All seven tasks are registered in the CubeMX task list, and everything else lives
inside `/* USER CODE */` blocks. This matters: `outputWriter` was once created
only in the generated section without a matching entry in the task list, and a
regeneration silently deleted it — taking the only `logQueue` consumer with it and
leaving a dangling reference that broke the build. If you add a task, register it
in CubeMX.

`CAN1_RX0` is *not* enabled in the CubeMX NVIC tab; it is enabled by hand in
`HAL_CAN_MspInit`, and `CAN1_RX0_IRQHandler` is hand-written in
`stm32f4xx_it.c`. If you ever enable it in CubeMX, delete both — CubeMX will
generate them and the duplicates will not link.

## Notes and limitations

- Positions are full `int32` on both transports; each motor task has its own
  depth-1 overwrite queue carrying the whole command, replacing the packed 32-bit
  task notification that used to cap targets at `int16`.
- Speed is limited to `MAX_SPEED` (250); `0` selects `DEFAULT_SPEED` (80). Over
  CAN an out-of-range speed is rejected and logged; over USB it is clamped.
- Command batches are not atomic on either transport.
- There is no link-loss failsafe: if the CAN client stops sending, motors hold
  their last commanded speed. Send an explicit `STOP`, or the FRC Disable
  broadcast. The roboRIO heartbeat, which is how the FRC spec expects an actuator
  device to notice the robot has gone away, is not implemented.
- `STATUS` replies are printed to the USB console on both transports; there is no
  status, ack, or firmware-version frame on the bus, so the board is invisible to
  FRC device-enumeration tools.
