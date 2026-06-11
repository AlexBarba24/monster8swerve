import time
import serial
from serial.tools import list_ports

# CubeMX/ST USB CDC often uses VID:PID 0483:5740.
# STM32 DFU bootloader is usually 0483:DF11, which is NOT the CDC COM port.
PREFERRED_VID = 0x0483
PREFERRED_PID = 0x5740

BAUD = 115200

def find_stm32_cdc_port():
    ports = list(list_ports.comports())

    # Prefer the normal ST CDC VID/PID.
    for p in ports:
        if p.vid == PREFERRED_VID and p.pid == PREFERRED_PID:
            return p.device

    # Fallback: look for likely descriptions.
    for p in ports:
        text = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if "stm" in text or "usb serial" in text or "cdc" in text:
            return p.device

    return None

print("Waiting for STM32 CDC serial port...")

while True:
    port = find_stm32_cdc_port()

    if port is None:
        time.sleep(0.5)
        continue

    try:
        print(f"\nOpening {port} at {BAUD}...")
        with serial.Serial(port, BAUD, timeout=0.2) as ser:
            # Toggle DTR so firmware can detect that a terminal opened, if you implement that later.
            ser.dtr = True
            ser.rts = True

            while True:
                data = ser.read(1024)
                if data:
                    print(data.decode("utf-8", errors="replace"), end="", flush=True)

    except serial.SerialException as e:
        print(f"\nSerial disconnected: {e}")
        print("Waiting for reconnect...")
        time.sleep(0.5)