import argparse
import serial
import threading
import sys
import time

BAUD = 115200

def read_from_device(ser: serial.Serial) -> None:
    while True:
        try:
            data = ser.readline()
            if data:
                print(data.decode("utf-8", errors="replace"), end="", flush=True)
        except serial.SerialException:
            print("\nDevice disconnected.")
            break

def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive STM32 USB CDC terminal")
    parser.add_argument("port", help="COM port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=BAUD, help="Baud rate, default 115200")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
            ser.dtr = True
            ser.rts = True
            time.sleep(0.5)

            print(f"Connected to {args.port} at {args.baud} baud.")
            print("Type commands like: MOVE 0 3200 50")
            print("Press Ctrl+C to exit.\n")

            reader = threading.Thread(target=read_from_device, args=(ser,), daemon=True)
            reader.start()

            while True:
                cmd = input("> ")
                ser.write((cmd + "\n").encode("utf-8"))
                ser.flush()

    except KeyboardInterrupt:
        print("\nExiting.")
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()