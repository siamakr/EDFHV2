#!/usr/bin/env python3
"""Capture Teensy serial to Data/*.csv and echo it. Keys you type go to the board.

    python3 scripts/serial_log.py
    python3 scripts/serial_log.py --port /dev/cu.usbmodem141859801

Ctrl+C stops. Do not run PlatformIO monitor at the same time — one process per port.
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
import termios
import tty
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("need pyserial:  python3 -m pip install pyserial")

REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "Data"
BAUD = 115200


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    for p in ports:
        blob = f"{p.device} {p.description} {p.hwid}".lower()
        if "usbmodem" in blob or "teensy" in blob or "usb serial" in blob:
            return p.device
    if ports:
        return ports[0].device
    sys.exit("no serial port found")


def stdin_to_serial(ser: serial.Serial, stop: threading.Event) -> None:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop.is_set():
            ch = sys.stdin.read(1)
            if not ch:
                break
            ser.write(ch.encode("utf-8", errors="replace"))
    except serial.SerialException:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args()

    DATA.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("serial_%Y-%m-%d_%H-%M-%S.csv")
    out = DATA / stamp
    port = find_port(args.port)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(f"port {port}  ->  {out}   Ctrl+C to stop", flush=True)

    stop = threading.Event()
    # Keys typed here are forwarded (n/f/a/w/m/...). Needs a real TTY.
    if sys.stdin.isatty():
        threading.Thread(target=stdin_to_serial, args=(ser, stop), daemon=True).start()

    t0 = time.time()
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_pc", "elapsed_s", "line"])
        f.flush()
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                now = time.time()
                w.writerow([datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                            f"{now - t0:.3f}", line])
                f.flush()
                print(line, flush=True)
        except KeyboardInterrupt:
            print("\nstopped", flush=True)
        finally:
            stop.set()
            ser.close()


if __name__ == "__main__":
    main()
