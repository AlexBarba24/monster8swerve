# CAN Protocol Reference

Complete wire format for the CAN control interface of the `monster8swerve`
board: every command frame, how to use it, and the encoder telemetry stream.

Addressing follows the
[FIRST/FRC CAN specification](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html)
using manufacturer code **8 (Team Use)**. Each of the eight motors is a separate
FRC device, so one frame commands one motor — the same shape a Talon or SparkMax
presents.

For the USB text console, build instructions, and firmware architecture, see
[README.md](README.md).

## Bus parameters

| Parameter | Value |
|-----------|-------|
| Peripheral | CAN1 |
| Pins | `PB8` = RX, `PB9` = TX |
| Bit rate | **1 Mbit/s** (42 MHz PCLK1 ÷ 3 prescaler ÷ 14 TQ) |
| Identifier format | **Extended, 29-bit** (FRC addressing) |
| Byte order | **Little-endian** for all multi-byte fields |
| Frame type | Data frames only (remote frames are rejected in hardware) |

`PB8`/`PB9` are logic-level pins, not a bus. You need a transceiver (SN65HVD230,
MCP2551, TJA1050, or similar) and a bus terminated with 120 Ω at both ends.

## Addressing

Every FRC identifier is a 29-bit value split into five fields:

| Bits | Width | Field | This board |
|------|-------|-------|------------|
| 28:24 | 5 | Device type | **2** — Motor Controller |
| 23:16 | 8 | Manufacturer | **8** — Team Use |
| 15:10 | 6 | API class | per command, below |
| 9:6 | 4 | API index | per command, below |
| 5:0 | 6 | Device number | **motor id**, 0–7 |

The API class and index together form the 10-bit API identifier, so:

```
full_id = (device_type << 24) | (manufacturer << 16) | (api_id << 6) | device_number
api_id  = (api_class << 4) | api_index
```

Which makes this board's identifier base `0x0208_0000`, and **motor n's address is
that base plus the API identifier plus n**.

### One motor per device number

The board claims a block of eight consecutive device numbers, one per motor. With
the default base of 0, motor 0 is device 0 and motor 7 is device 7.

Because the motor is named by the identifier, the entire 8-byte payload is free
for the command's own data, and any FRC tool that walks the bus sees eight motor
controllers rather than one opaque board.

`CAN_MOTOR_ID_BASE` in `Core/Src/main.c` moves the block. It **must be a multiple
of 8**, because a single hardware filter mask accepts the whole aligned block; the
firmware enforces this with a `_Static_assert`. Valid bases are 0, 8, 16, 24, 32,
40 and 48, which lets seven of these boards share one bus:

| Base | Device numbers | Motor 0 `STOP` |
|------|----------------|----------------|
| 0 (default) | 0–7 | `0x02080000` |
| 8 | 8–15 | `0x02080008` |
| 16 | 16–23 | `0x02080010` |

Device number **`0x3F` (63)** is reserved by the spec for device-specific
broadcasts, and this board accepts it as **"every motor on this board"**. That is
what keeps an all-stop down to a single frame now that a command names one motor.
It is also why the base tops out at 48: a block starting at 56 would run into it.

## Payload

Every command uses the same 8-byte layout:

| Bytes | Field | Type |
|-------|-------|------|
| 0–3 | Target position | `int32` little-endian |
| 4–7 | Speed | `int32` little-endian |

A command that needs only one field ignores the other, and a command that needs
neither ignores the payload entirely:

| Command | Target | Speed | Required DLC |
|---------|--------|-------|--------------|
| MOVEABS, MOVEREL | used | used | **8** |
| MOVESPEED | ignored | used | **8** |
| STOP, DISABLE, STATUS | ignored | ignored | any, including 0 |

Motion commands are rejected unless the DLC is exactly 8. A short frame is not
zero-filled, because doing so would silently turn a truncated command into a
plausible one — a move to position 0 at default speed — which is worse than
refusing it.

## Conventions

**Motor ids** are `0`–`7`. Even ids are drive motors, odd ids are steer motors:

| id | Position | | id | Position |
|----|----------|-|----|----------|
| 0 | Front left drive | | 4 | Back left drive |
| 1 | Front left steer | | 5 | Back left steer |
| 2 | Front right drive | | 6 | Back right drive |
| 3 | Front right steer | | 7 | Back right steer |

**Target** is a signed 32-bit step position. Absolute positions are counted from
wherever the motor was at boot, which is defined as 0.

The full `int32` range genuinely reaches the motor. That matters: a 200-step motor
at 256 microsteps is 51,200 steps per revolution, so a 16-bit position could not
express even one full turn. Relative moves are added to the current position
through a 64-bit intermediate and clamped, so a large offset saturates rather than
wrapping and reversing direction.

**Speed** is a unitless rate fed into the step accumulator, not a physical unit,
and is limited to **±250** (`MAX_SPEED`). Anything outside that range is
**rejected and logged**, not clamped — the field is far wider than the achievable
range, so an out-of-range value is much more likely to be a byte-order or units
mistake than a real request, and you want to hear about it during bring-up.

The step ISR runs at 10 kHz and emits a pulse when the accumulator exceeds 250,
with a pulse occupying two ticks, so:

```
steps/s  ~=  10000 * speed / (250 + speed)
```

That gives roughly 2400 steps/s at `speed = 80` and a hard ceiling of 5000
steps/s at `speed = 250`. A speed of `0` means "use the default" (80) for
position moves, and "stop" for jog commands.

For a jog, the **sign of the speed sets the direction**. For a position move only
the magnitude is used, since direction is implied by target versus current
position.

## Identifier map

Lower identifiers win CAN arbitration. Because the API identifier sits above the
device number, the ordering below is a safety property rather than cosmetic: a
`STOP` for any motor wins against a `MOVEABS` for any motor, so an all-stop is
never stuck behind a stream of motion frames.

### Commands (client to board)

Add the motor id to each identifier. `0x3F` in place of the motor id addresses all
eight.

| Motor 0 ID | API | Class/Index | Command | Payload |
|------------|-----|-------------|---------|---------|
| `0x02080000` | `0x000` | 0 / 0 | [STOP](#stop) | ignored |
| `0x02080040` | `0x001` | 0 / 1 | [DISABLE](#disable) | ignored |
| `0x02080400` | `0x010` | 1 / 0 | [MOVESPEED](#movespeed) | speed |
| `0x02080C00` | `0x030` | 3 / 0 | [MOVEABS](#moveabs) | target + speed |
| `0x02080C40` | `0x031` | 3 / 1 | [MOVEREL](#moverel) | target + speed |
| `0x02081400` | `0x050` | 5 / 0 | [STATUS](#status) | ignored |

Class numbering follows the convention in the spec's motor-controller example, so
the device reads sensibly to anyone familiar with FRC CAN: class 1 is speed
control, class 3 is position control, class 5 is status, class 6 is periodic
status. Class 0 holds the stop and disable controls.

### Board-level frames

Two identifiers are not per-motor. Both use the **base** device number, since a
board-level frame still needs one and that is the number identifying this board's
block.

| ID | API | Direction | Purpose |
|----|-----|-----------|---------|
| `0x02081440` | `0x051` | client to board | [ENCODER_REQ](#encoder_req) — publish one telemetry frame now |
| `0x02081800` | `0x060` | board to client | [Encoder telemetry](#encoder-telemetry) — all four channels |

### Broadcast messages (any node to all nodes)

| Full ID | Message | Handled |
|---------|---------|---------|
| `0x00000000` | Disable | **Yes** — immediate stop, see [below](#broadcast-messages-and-disable) |
| `0x00000040` | System Halt | **Yes** — treated identically to Disable |
| `0x00000080`–`0x00000280` | Reset, Assign, Query, Heartbeat, Sync, Update, Firmware Version, Enumerate, Resume | Counted, otherwise ignored |

### Reserved

| API | Intended use |
|-----|--------------|
| `0x061`+ | Per-motor status frames, mirroring the command payload: `int32` position then `int32` speed |

The receive filter accepts every API identifier already, so adding one needs no
filter change.

---

## Command details

All `cansend` examples use eight hex digits for the identifier, which is how
SocketCAN selects an extended frame. Three digits would send a standard frame that
this board rejects.

### STOP

`0x02080000 + motor` — ramps the motor to a stop (speed 0) but leaves the driver
energised, so it keeps holding torque. Payload ignored.

```bash
cansend can0 02080000#          # stop motor 0
cansend can0 02080003#          # stop motor 3
cansend can0 0208003F#          # stop all eight motors
```

An empty `STOP` to `0x0208003F` is the cheapest possible all-stop: the shortest
frame with the lowest identifier this board uses, so it both wins arbitration and
occupies the least bus time.

### DISABLE

`0x02080040 + motor` — de-energises the driver by releasing its enable pin. The
motor freewheels and loses holding torque. Payload ignored.

```bash
cansend can0 02080040#          # release motor 0
cansend can0 02080045#          # release motor 5
cansend can0 0208007F#          # release all eight  (0x040 + 0x3F)
```

Note that several motors share an enable pin (motors 0 and 1 share `PC15`;
motors 3 and 7 share `PB6`), so disabling one of a pair also de-energises the
other.

### MOVEABS

`0x02080C00 + motor` — move to an absolute step position. Position moves jump
straight to their commanded speed rather than ramping. DLC must be 8.

```bash
# Motor 0 to step 51200 (one revolution at 256 microsteps) at speed 80.
#   51200 = 0x0000C800 -> bytes 00 C8 00 00
#      80 = 0x00000050 -> bytes 50 00 00 00
cansend can0 02080C00#00C8000050000000

# Motor 3 to step -1000 at the default speed (speed field 0).
#   -1000 = 0xFFFFFC18 -> bytes 18 FC FF FF
cansend can0 02080C03#18FCFFFF00000000

# Every motor to step 0 at speed 100 (0x64) - a synchronised return to origin
cansend can0 02080C3F#0000000064000000
```

### MOVEREL

`0x02080C40 + motor` — identical payload to `MOVEABS`, but the target is an offset
from the motor's current position. DLC must be 8.

```bash
# Back motor 0 off by 400 steps at speed 120.
#   -400 = 0xFFFFFE70 -> bytes 70 FE FF FF
#    120 = 0x00000078 -> bytes 78 00 00 00
cansend can0 02080C40#70FEFFFF78000000

# Nudge motor 1 forward 100 steps at speed 40.
#   100 = 0x00000064 -> bytes 64 00 00 00
#    40 = 0x00000028 -> bytes 28 00 00 00
cansend can0 02080C41#6400000028000000
```

### MOVESPEED

`0x02080400 + motor` — run continuously (jog) at a signed speed. The target field
is ignored; the sign of the speed sets the direction. DLC must still be 8.

Unlike position moves, jog commands **ramp**: the speed changes by 1 unit every
5 ms, so reaching speed 80 from standstill takes about 400 ms.

```bash
# Motor 0 forward at 60.   60 = 0x0000003C -> bytes 3C 00 00 00
cansend can0 02080400#000000003C000000

# Motor 2 reverse at 60.  -60 = 0xFFFFFFC4 -> bytes C4 FF FF FF
cansend can0 02080402#00000000C4FFFFFF

# Stop jogging motor 0 (speed 0 - equivalent to STOP for that motor)
cansend can0 02080400#0000000000000000
```

Because a jog runs until countermanded, a motor keeps its last commanded speed if
the client stops transmitting or the cable is pulled. The Disable broadcast will
stop it; there is no timeout that does so on its own. See
[Broadcast messages](#broadcast-messages-and-disable).

### STATUS

`0x02081400 + motor` — requests a status report. The reply is printed to the **USB
console**, not returned on the bus — there is no status frame yet. Payload ignored.

```bash
cansend can0 02081400#          # report motor 0
cansend can0 0208143F#          # report all eight motors
```

Each selected motor produces a line like:

```text
Motor (0) Status: target=51200, position=51200, speed=80, commanded_speed=80, enabled=0, mode=position
```

### ENCODER_REQ

`0x02081440` — asks for one immediate encoder telemetry frame. Board-level, so it
is not per-motor and takes no payload.

```bash
cansend can0 02081440#
```

The board responds with a single [encoder telemetry](#encoder-telemetry) frame.
This is in addition to the periodic stream, not instead of it — unless you have
disabled the periodic stream, in which case this is the only way to sample the
encoders.

---

## Broadcast messages and Disable

Broadcast messages set both device type and manufacturer to 0, with API class 0,
so the message number lands in the API index field. **Disable is arbitration ID
`0x00000000`** — the numerically lowest identifier possible, and therefore the
highest priority frame that can exist on a CAN bus.

```bash
cansend can0 00000000#          # disable every device on the bus
```

The spec requires devices to disable *immediately* on this message, and this
board takes that literally. The Disable frame is **acted on inside the receive
interrupt** rather than being handed to a task:

- Every motor has its `enabled` flag cleared, its STEP line dropped, its speed
  zeroed, and its enable pin released, in that order. Clearing `enabled` first is
  what matters — it is the flag the step generator checks before emitting a pulse,
  so from that instant no motor can move even if the higher-priority step
  interrupt fires part-way through the loop.
- Nothing is queued. The normal command path is four hops deep with a bounded
  queue at each stage that is allowed to drop frames under load; neither the
  latency nor the possibility of a drop is acceptable for this message.

**System Halt** (`0x00000040`) is handled identically, on the reading that both
mean "stop actuating now". The remaining broadcast messages are optional in the
spec and unimplemented here; they increment the `bcast` counter and are
discarded.

### Disable is not latching

Once disabled, the board will act on the next motion command it receives. It does
not wait for a System Resume broadcast, and it does not stay disabled.

That is a deliberate choice, and it is the part of this implementation furthest
from full FRC compliance. The spec ties ongoing enable state to the **roboRIO
universal heartbeat** (ID `0x01011840`, every 20 ms, carrying an `enabled` bit and
a system watchdog flag), and it expects an actuator device to stop when that
heartbeat goes stale for 100 ms. This board does not listen to the heartbeat at
all — the receive filter discards it.

The consequence is that Disable behaves correctly under a roboRIO (the robot
sends no motion commands while disabled, so nothing re-enables the motors) but
provides **no protection against a client that dies mid-jog**. A latching disable
without heartbeat support would be worse, not better: nothing in a normal FRC
system sends System Resume, so the board would latch off and appear dead.

Adding heartbeat-gated enable is the natural next step if you need that
protection. It means a fourth receive filter for `0x01011840`, a timestamp
refreshed on each heartbeat, and a check in the step generator against both the
`enabled` bit and a 100 ms staleness deadline.

---

## Encoder telemetry

`0x02081800` — the four analog encoder channels, as raw ADC counts. Board-level:
one frame carries all four channels.

| Byte | Field | Source |
|------|-------|--------|
| 0–1 | Channel 0 (`ADC1_IN4`, pin `PA4`) | `u16` little-endian |
| 2–3 | Channel 1 (`ADC1_IN5`, pin `PA5`) | |
| 4–5 | Channel 2 (`ADC1_IN6`, pin `PA6`) | |
| 6–7 | Channel 3 (`ADC1_IN7`, pin `PA7`) | |

Values are **12-bit, so `0`–`4095`**, sent as 16-bit words. Counts are deliberately
raw rather than pre-scaled to degrees, so the client owns the calibration and the
board's publish path stays a handful of register writes:

```
degrees = count * 360 / 4096
```

The ADC free-runs over all four channels via DMA and completes a full scan every
~2.9 µs, so the published values are always fresh regardless of publish rate.

This is the one place the per-motor addressing does not apply. Four channels in
one frame costs a quarter of the bus time of four per-motor frames, and with only
three hardware transmit mailboxes a four-frame burst at 100 Hz would drop frames.
Per-motor status frames are reserved at API `0x061`+ if you later want them.

### Publishing frequency

**Default: every 10 ms (100 Hz).** Controlled by `CAN_TLM_INTERVAL_MS` in
`Core/Src/main.c`:

```c
#define CAN_TLM_INTERVAL_MS 10
```

| Value | Behaviour |
|-------|-----------|
| `10` (default) | 100 Hz periodic stream |
| `20` | 50 Hz |
| `1000` | 1 Hz |
| `0` | Periodic publishing **disabled** — only `ENCODER_REQ` produces frames |

Two properties worth knowing:

- **The interval is a minimum gap, not a locked phase.** The publisher waits
  `CAN_TLM_INTERVAL_MS` relative to finishing the previous publish, and an
  `ENCODER_REQ` re-phases the stream. In practice this is dominated by the 1 kHz
  FreeRTOS tick, which already quantises any interval to ±1 ms.
- **Frames are dropped, never queued, when the bus is congested.** If all three
  hardware transmit mailboxes are busy, the sample is discarded and
  `can_tx_drop_count` increments. Telemetry never blocks or delays command
  handling. If you see `tx_drop` climbing, either nothing is on the bus to
  acknowledge the frames, or the bus is saturated.

### Cost of periodic publishing

| | Per sample |
|---|---|
| Bus time | ~155 µs worst case (128 bits + stuffing at 1 Mbit/s) |
| Bus load at 100 Hz | ~1.6% |
| CPU | A few register writes; no formatting, no allocation, no blocking |

The equivalent USB log line costs a `vsnprintf`, a 160-byte queue copy, and a
write that busy-waits up to 50 ms when the endpoint is busy. That is the reason
CAN telemetry can run at 100 Hz while the USB diagnostics run at 1 Hz.

If you need faster than 100 Hz, the limit is bus bandwidth rather than the board:
at 1 kHz a single telemetry stream is ~16% of a 1 Mbit/s bus.

---

## Usage patterns

### Fixed-rate swerve control loop

One frame per motor means **eight frames per cycle**: four `MOVEABS` for the steer
motors and four `MOVESPEED` for the drive motors.

```text
0x02080C01  TT TT TT TT  SS SS SS SS      steer motor 1 to a target
0x02080C03  TT TT TT TT  SS SS SS SS      steer motor 3
0x02080C05  TT TT TT TT  SS SS SS SS      steer motor 5
0x02080C07  TT TT TT TT  SS SS SS SS      steer motor 7
0x02080400  -- -- -- --  VV VV VV VV      drive motor 0 at a signed speed
0x02080402  -- -- -- --  VV VV VV VV      drive motor 2
0x02080404  -- -- -- --  VV VV VV VV      drive motor 4
0x02080406  -- -- -- --  VV VV VV VV      drive motor 6
```

A concrete cycle — steer all wheels to +512 steps at speed 100 (`0x64`), drive all
wheels forward at 60 (`0x3C`):

```bash
cansend can0 02080C01#0002000064000000
cansend can0 02080C03#0002000064000000
cansend can0 02080C05#0002000064000000
cansend can0 02080C07#0002000064000000
cansend can0 02080400#000000003C000000
cansend can0 02080402#000000003C000000
cansend can0 02080404#000000003C000000
cansend can0 02080406#000000003C000000
```

Eight frames is roughly 1.2 ms of bus time, so a 50 Hz control rate costs about
**6% of a 1 Mbit/s bus**, plus ~1.6% for 100 Hz telemetry. That is comfortable,
but it is worth being explicit that this is the price of per-motor addressing: the
earlier batched format did the same work in three frames and about 2.3%. You are
buying FRC-idiomatic addressing and full 32-bit fields with bus bandwidth.

If the four steer wheels share a target, `0x02080C3F` sends it to all eight motors
in one frame, which collapses a synchronised move to a single frame.

### Request-driven polling

If you would rather pull encoder data than have it pushed, set
`CAN_TLM_INTERVAL_MS` to `0` and send `ENCODER_REQ` whenever you want a sample.
The reply arrives as a normal telemetry frame. This costs you a round trip
instead of saving bus bandwidth, so it is generally only worth it if your
client's timing is irregular.

### Python client sketch

```python
import can
import struct

DEVICE_TYPE = 2   # Motor Controller
MANUFACTURER = 8  # Team Use
MOTOR_ID_BASE = 0
ALL_MOTORS = 0x3F

API_STOP = 0x000
API_DISABLE = 0x001
API_MOVESPEED = 0x010
API_MOVEABS = 0x030
API_MOVEREL = 0x031
API_STATUS = 0x050
API_ENCODER_REQ = 0x051
API_TLM_ENCODERS = 0x060

BROADCAST_DISABLE = 0x00000000

bus = can.interface.Bus(channel="can0", bustype="socketcan")


def frc_id(api_id, device_number):
    return (DEVICE_TYPE << 24) | (MANUFACTURER << 16) | (api_id << 6) | device_number


def send(api_id, motor_id, target=0, speed=0, payload=True):
    """motor_id may be ALL_MOTORS to address every motor at once."""
    device = motor_id if motor_id == ALL_MOTORS else MOTOR_ID_BASE + motor_id
    data = struct.pack("<ii", target, speed) if payload else b""
    bus.send(can.Message(arbitration_id=frc_id(api_id, device), data=data,
                         is_extended_id=True))


def move_abs(motor_id, target, speed=0):
    """Absolute position move; speed 0 selects the firmware default."""
    send(API_MOVEABS, motor_id, target=target, speed=speed)


def move_rel(motor_id, offset, speed=0):
    send(API_MOVEREL, motor_id, target=offset, speed=speed)


def jog(motor_id, speed):
    """Signed speed, -250..250. Sign selects direction; 0 stops."""
    send(API_MOVESPEED, motor_id, speed=speed)


def stop(motor_id=ALL_MOTORS):
    send(API_STOP, motor_id, payload=False)


def disable_bus():
    """FRC Disable broadcast - stops every device on the bus immediately."""
    bus.send(can.Message(arbitration_id=BROADCAST_DISABLE, data=b"",
                         is_extended_id=True))


def read_encoders(msg):
    """Decode a telemetry frame into four angles in degrees."""
    counts = struct.unpack("<4H", msg.data)
    return [c * 360.0 / 4096.0 for c in counts]


TELEMETRY_ID = frc_id(API_TLM_ENCODERS, MOTOR_ID_BASE)

for msg in bus:
    if msg.is_extended_id and msg.arbitration_id == TELEMETRY_ID:
        print(["%.1f" % d for d in read_encoders(msg)])
```

`struct.pack("<ii", target, speed)` is exactly the 8-byte payload — the format
string is the wire format.

---

## Receive filtering

Three mask-mode filter banks feed FIFO 0. A `1` in the mask means the bit must
match:

| Bank | Match | Mask | Accepts |
|------|-------|------|---------|
| 0 | `0x02080000` | `0x1FFF0038` | Any API identifier for any of this board's eight motors |
| 1 | `0x0208003F` | `0x1FFF003F` | Any API identifier sent to all motors |
| 2 | `0x00000000` | `0x1FFFFC00` | The whole broadcast class, any message |

Bank 0 is the interesting one: masking bits 5:3 of the device number and leaving
bits 2:0 free accepts an aligned block of exactly eight device numbers. That is
why `CAN_MOTOR_ID_BASE` must be 8-aligned, and why the whole board costs one
filter bank rather than eight.

Both device masks also cover the `IDE` and `RTR` bits, so standard-identifier
frames and remote-transmission requests are rejected in hardware rather than
reaching the interrupt.

Everything else is dropped at zero CPU cost — including other manufacturers'
devices, other boards' device blocks, and the roboRIO heartbeat.

One consequence of bank 0 accepting every API identifier is that the board's own
telemetry identifier matches its own filter. That is invisible on a real bus,
since a CAN controller does not receive its own transmissions, but in the
loopback modes used for bench testing the telemetry comes back. It is silently
ignored rather than reported, so loopback testing stays quiet.

## Error handling and diagnostics

Nothing is acknowledged on the bus. A rejected frame is counted and reported on
the USB console:

| Condition | Console message | Counter |
|-----------|-----------------|---------|
| API identifier not in the map | `ERR CAN UNKNOWN API id=0x... api=0x... dlc=...` | `invalid` |
| Motion command with DLC ≠ 8 | `ERR CAN BAD DLC ...` | `invalid` |
| Speed outside ±250 | `ERR CAN SPEED OUT OF RANGE ...` | `invalid` |
| Device number outside the block | `ERR CAN BAD DEVICE NUMBER ...` | `invalid` |
| Standard-ID or remote frame | *(silent, ISR context)* | `invalid` |
| Scheduler queue full | `ERR CAN QUEUE PUT FAILED ...` | — |

Errors report both the raw 29-bit identifier, which is what a bus trace shows,
and the decoded API identifier, which is what the tables above list.

The logger prints a CAN summary every second:

```text
CAN rx=1204 qdrop=0 invalid=0 queued=1204 tx_drop=0 bcast=0 dis=0 err=0x00000000
```

| Field | Meaning |
|-------|---------|
| `rx` | Frames accepted by the filter and read out of the hardware FIFO |
| `qdrop` | Frames the interrupt could not buffer. A definite, unambiguous loss. |
| `invalid` | Unknown API identifier, bad DLC, out-of-range speed, or wrong frame type |
| `queued` | Commands successfully handed to the motion scheduler |
| `tx_drop` | Telemetry frames dropped because all mailboxes were busy |
| `bcast` | Broadcast messages seen, handled or not |
| `dis` | Disable / System Halt broadcasts acted on |
| `err` | `HAL_CAN_GetError`; latches the last protocol error and bus state |

With one motor per frame, a healthy stream shows `queued` tracking `rx` exactly.
Frames sent to `0x3F` are the exception: each produces eight queued commands.

**`dis` climbing while you expect motion** means something on the bus is holding
the board disabled — check for a driver station in a disabled state.

**`err` nonzero with `tx_drop` climbing almost always means nothing else is on
the bus**, not that your commands were malformed — see below.

## Bench testing without a second node

Automatic retransmission is disabled, so with no other node to acknowledge a
frame, every transmission fails immediately. You will see `tx_drop` climbing and
`err` nonzero even though receive works fine.

To exercise the code with only the board connected, temporarily switch the
peripheral into loopback in `MX_CAN1_Init`:

```c
hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
```

Automatic bus-off recovery (`ABOM`) is enabled in software before
`HAL_CAN_Start`, so a wiring fault that drives the controller bus-off recovers on
its own instead of silently killing the interface.

To disable the CAN interface entirely, set `USE_CAN_COMMANDS` to `0` in
`Core/Src/main.c`. The peripheral is then never started and the CAN tasks stay
idle.

## Limits

- **Speed is limited to ±250** (`MAX_SPEED`) and out-of-range values are rejected
  rather than clamped. The 32-bit field is for layout uniformity, not range.
- **Positions are full `int32`** and reach the motor intact. This is a change from
  the earlier protocol revision, which was capped at `int16` by the internal
  task-notification word; that hop is now a per-motor queue carrying the whole
  command.
- **A frame to `0x3F` is not atomic.** It expands to eight independent commands,
  so under scheduler-queue pressure some motors can be accepted while others are
  dropped. The counters tell you when it happens.
- **The roboRIO heartbeat is not implemented**, so there is no link-loss failsafe
  and no enable gating. Motors hold their last commanded speed if the client goes
  away. The Disable broadcast is honoured but does not latch — see
  [Disable is not latching](#disable-is-not-latching).
- **`STATUS` replies go to the USB console**, not the bus. There is no status,
  ack, or firmware-version response frame, so the board does not answer FRC
  device-enumeration queries.
- **Absolute positions are relative to boot.** There is no homing routine; the
  position counter starts at 0 wherever the motor happens to be at power-on.
