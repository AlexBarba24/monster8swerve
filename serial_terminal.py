import argparse
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports


PREFERRED_VID = 0x0483
PREFERRED_PID = 0x5740
BAUD_DEFAULT = 115200


def timestamp():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def get_candidate_ports():
    ports = list(list_ports.comports())
    candidates = []

    # Prefer normal ST CDC VID/PID.
    for p in ports:
        if p.vid == PREFERRED_VID and p.pid == PREFERRED_PID:
            candidates.append(p)

    # Fallback: likely serial descriptions.
    for p in ports:
        if p in candidates:
            continue

        text = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if "stm" in text or "usb serial" in text or "cdc" in text or "virtual com" in text:
            candidates.append(p)

    return candidates


def open_stm32_serial(preferred_port, baud):
    if preferred_port:
        try:
            ser = serial.Serial(preferred_port, baud, timeout=0.05, write_timeout=0.5)
            return ser, preferred_port
        except (serial.SerialException, FileNotFoundError, OSError):
            return None, preferred_port

    candidates = get_candidate_ports()

    for p in candidates:
        try:
            print(f"Trying {p.device}...", flush=True)
            ser = serial.Serial(p.device, baud, timeout=0.05, write_timeout=0.5)
            return ser, p.device
        except (serial.SerialException, FileNotFoundError, OSError) as e:
            print(f"  Could not open {p.device}: {e}")

    return None, None


def input_worker(cmd_queue, stop_event, log_file):
    while not stop_event.is_set():
        try:
            cmd = input()
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break

        if cmd.lower() in ("exit", "quit"):
            stop_event.set()
            break

        cmd_queue.put(cmd)

        if log_file:
            log_file.write(f"[{timestamp()}] > {cmd}\n")
            log_file.flush()

def list_available_ports():
    ports = list(list_ports.comports())

    if not ports:
        print("No serial ports found.")
        return

    for p in ports:
        print("-----")
        print(f"device:       {p.device}")
        print(f"name:         {p.name}")
        print(f"description:  {p.description}")
        print(f"manufacturer: {p.manufacturer}")
        print(f"hwid:         {p.hwid}")
        print(f"vid/pid:      {p.vid}:{p.pid}")
        print(f"location:     {p.location}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", default=None, help="Optional COM port, e.g. COM6")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    parser.add_argument("--log", default=None)
    parser.add_argument("--timestamp", action="store_true")
    parser.add_argument("--retry-delay", type=float, default=0.5)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    stop_event = threading.Event()
    cmd_queue = queue.Queue()

    if args.list:
        list_available_ports()
        return
    log_file = None
    if args.log:
        log_path = Path(args.log)
        log_file = open(log_path, "a", encoding="utf-8")
        log_file.write(f"\n\n===== Session started {timestamp()} =====\n")
        log_file.flush()
        print(f"Logging to {log_path}")

    print("Type commands and press Enter.")
    print("Example: MOVE 0 3200 50")
    print("Type 'exit' or 'quit' to close.\n")

    input_thread = threading.Thread(
        target=input_worker,
        args=(cmd_queue, stop_event, log_file),
        daemon=True,
    )
    input_thread.start()

    print("Waiting for STM32 CDC serial port...")

    last_no_port_print = 0.0

    try:
        while not stop_event.is_set():
            ser, port = open_stm32_serial(args.port, args.baud)

            if ser is None:
                now = time.time()
                if now - last_no_port_print > 2.0:
                    print("No openable STM32 CDC port yet. Retrying...")
                    last_no_port_print = now

                time.sleep(args.retry_delay)
                continue

            try:
                with ser:
                    print(f"\nConnected to {port} at {args.baud}.")
                    ser.dtr = True
                    ser.rts = True
                    time.sleep(0.2)

                    if log_file:
                        log_file.write(f"===== Connected to {port} at {timestamp()} =====\n")
                        log_file.flush()

                    while not stop_event.is_set():
                        # Read device logs/output.
                        data = ser.read(1024)

                        if data:
                            text = data.decode("utf-8", errors="replace")

                            if args.timestamp:
                                output = f"[{timestamp()}] {text}"
                            else:
                                output = text

                            print(output, end="", flush=True)

                            if log_file:
                                log_file.write(output)
                                log_file.flush()

                        # Send queued host commands.
                        while not cmd_queue.empty():
                            cmd = cmd_queue.get_nowait()
                            ser.write((cmd + "\n").encode("utf-8"))
                            ser.flush()

            except (serial.SerialException, FileNotFoundError, OSError) as e:
                print(f"\nSerial disconnected: {e}")
                print("Waiting for reconnect...")
                time.sleep(args.retry_delay)

    except KeyboardInterrupt:
        print("\nExiting.")

    finally:
        stop_event.set()

        if log_file:
            log_file.write(f"===== Session ended {timestamp()} =====\n")
            log_file.close()


if __name__ == "__main__":
    main()