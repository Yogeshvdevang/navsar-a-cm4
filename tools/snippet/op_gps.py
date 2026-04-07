#!/usr/bin/env python3

import serial
import threading


def pipe(source, destination, stop_event, label):
    while not stop_event.is_set():
        try:
            data = source.read(256)
            if data:
                destination.write(data)
                destination.flush()
                print(f"{label}: {data!r}")
        except Exception:
            if not stop_event.is_set():
                raise
            break


def duplex_tunnel():
    stop_event = threading.Event()
    ama0 = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.1)
    ama3 = serial.Serial("/dev/ttyAMA3", 115200, timeout=0.1)
    try:
        t1 = threading.Thread(
            target=pipe,
            args=(ama0, ama3, stop_event, "ama0->ama3"),
            daemon=True,
        )
        t2 = threading.Thread(
            target=pipe,
            args=(ama3, ama0, stop_event, "ama3->ama0"),
            daemon=True,
        )
        t1.start()
        t2.start()
        while t1.is_alive() and t2.is_alive():
            t1.join(0.2)
            t2.join(0.2)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        for port in (ama0, ama3):
            try:
                port.cancel_read()
            except Exception:
                pass
        t1.join(1.0)
        t2.join(1.0)
        ama0.close()
        ama3.close()


if __name__ == "__main__":
    duplex_tunnel()
