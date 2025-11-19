#!/usr/bin/python3
'''
Trigger script to bind a joystick key to

- Make a screenshot
- Store current liftoff telemetry
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
import datetime
import json
import math
from pathlib import Path
import socket
import struct
import sys
import threading
import time

import evdev
from PIL import ImageGrab

from geo_util import gps_from_coord, quat2heading
from liftoff_telemetry import TelemetryParser, telemetry_socket

KEY_TRIGGER = 292
SCREENSHOT_PATH = "liftoff_screenshots"
MAX_TELEMETRY_AGE = 1.0

last_telemetry = None
telemetry_config = None
writer_thread = None

def new_screenshot_name():
    ts = datetime.datetime.now()
    return Path(SCREENSHOT_PATH) / (ts.strftime('%Y%m%d-%H%M%S') + '.png')

def run_trigger():
    data = last_telemetry
    print()
    if not data or data[0] < (time.monotonic() - MAX_TELEMETRY_AGE):
        print("No telemetry, or too old - exiting")
        return

    filepath = new_screenshot_name()
    # Save screenshot
    image = ImageGrab.grab()
    image.save(filepath)

    telemetry_filename = filepath.parent / (filepath.stem + ".telemetry")
    with open(telemetry_filename, "wb") as f:
        # write a copy of the telemetry configuration
        f.write(struct.pack("<I", len(telemetry_config)) + telemetry_config)
        # write the telemetry data itself
        f.write(data[1])
    print(f'New screenshot: {filepath.stem}, wrote telemetry to: {telemetry_filename}')

def key_received(key):
    if key.code == KEY_TRIGGER and key.type == 0x01 and key.value == 1:
        run_trigger()

def telemetry_display(rec):
    #GLYPH_DIRECTIONS='↓↘→↗↑↖←↗'
    #GLYPH_STATIONARY='·'
    GLYPH_DIRECTIONS='↑↗→↘↓↙←↖'
    GLYPH_STATIONARY='·'
    angle = math.atan2(rec.Velocity[0], rec.Velocity[2])
    magnitude = math.sqrt(rec.Velocity[0]*rec.Velocity[0] + rec.Velocity[2]*rec.Velocity[2])
    if magnitude < 0.1:
        dir_glyph = GLYPH_STATIONARY
    else:
        glyph_idx = round(angle / (2.0 * math.pi) * 8.0) % 8
        dir_glyph = GLYPH_DIRECTIONS[glyph_idx]

    heading = quat2heading(rec.Attitude[0], rec.Attitude[1], rec.Attitude[2], rec.Attitude[3])
    glyph_idx = round(heading / (2.0 * math.pi) * 8.0) % 8
    facing_glyph = GLYPH_DIRECTIONS[glyph_idx]

    sys.stdout.write(f' XZ {rec.Position[0]:6.1f} {rec.Position[2]:6.1f} ({dir_glyph} {rec.Velocity[0]:5.1f} {rec.Velocity[2]:5.1f}) alt: {rec.Position[1]:5.1f} ({rec.Velocity[1]:5.1f}) facing: {facing_glyph} {math.degrees(heading):5.1f}\33[K\r')
    #sys.stdout.write(f' XZ {rec.Position[0]:6.1f} {rec.Position[2]:6.1f} ({dir_glyph} {rec.Velocity[0]:5.1f} {rec.Velocity[2]:5.1f}) alt: {rec.Position[1]:5.1f} ({rec.Velocity[1]:5.1f}) facing: {rec.Attitude[0]:4.1f} {rec.Attitude[1]:4.1f} {rec.Attitude[2]:4.1f} {rec.Attitude[3]:4.1f} \33[K\r')
    sys.stdout.flush()

def telemetry_main(sock, desc):
    print('Starting telemetry thread')
    config = json.loads(desc)
    parser = TelemetryParser(config['StreamFormat'])
    while True:
        data = sock.recv(4096)
        if not data:
            break
        global last_telemetry
        last_telemetry = (time.monotonic(), data)

        rec = parser.parse(data)
        telemetry_display(rec)

def main():
    try:
        # Find device
        # XXX cannot use evdev.list_devices because it will miss some accessible devices
        devices_by_name = {}
        for path in Path('/dev/input').iterdir():
            if path.name.startswith('event'):
                try:
                    device = evdev.InputDevice(path)
                except PermissionError as e:
                    pass
                else:
                    devices_by_name[device.name] = device

        print(devices_by_name)

        device = devices_by_name['OpenTX Radiomaster Pocket Joystick (remote)']

        # Bind telemetry UDP socket
        sock, desc = telemetry_socket()
        global telemetry_config
        telemetry_config = desc.encode()

        thread = threading.Thread(target=telemetry_main, args=(sock,desc))
        thread.start()

        for ev in device.read_loop():
            key_received(ev)
    finally:
        try: # This will raise "transport endpoint not connected" but still interrupt the thread.
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        thread.join()

if __name__ == '__main__':
    main()
