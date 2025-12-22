#!/usr/bin/env python3
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
from enum import IntEnum
import evdev
from evdev.ecodes import EV_ABS, EV_KEY
import evdev.ecodes as ec
import json
import os
import socket
import struct
import time
import threading

from crsf import PacketsTypes
from liftoff_telemetry import TelemetryParser, telemetry_socket, telemetry_keepalive
from liftoff_telemetry_crsf import telemetry_liftoff_to_crsf

BIND = ("0.0.0.0", 9005)

# us values from https://www.expresslrs.org/software/switch-config/#hybrid-and-wide-switch-configuration-modes
# CRSF axis maximum value
AXIS_MAX = 1984 - 1
# CRSF axis mid-point
AXIS_MID = 992            # us_to_ticks(1500)
AXIS_3POS_LEFT = 592      # (us_to_ticks(1000) + us_to_ticks(1500))/2
AXIS_3POS_RIGHT = 1392    # (us_to_ticks(1500) + us_to_ticks(2000))/2

# linux udev descriptors
DEVICES_JSON = [
{
    "name": "CRSF Joystick",
    "capabilities": {
            # see /usr/include/linux/input-event-codes.h
            # EV_KEY (13 keys)
            "1": [
                ec.BTN_JOYSTICK,
                ec.BTN_THUMB,
                ec.BTN_THUMB2,
                ec.BTN_TOP,
                ec.BTN_TOP2,
                ec.BTN_PINKIE,
                ec.BTN_BASE,
                ec.BTN_BASE2,
                ec.BTN_BASE3,
                ec.BTN_BASE4,
                ec.BTN_BASE5,
                ec.BTN_BASE6,
                ec.BTN_BASE6 + 1, # no constant for 300
            ],
            # EV_ABS (7 axes)
            "3": [
                [ec.ABS_X, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_Y, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_Z, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_RX, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_THROTTLE, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_RUDDER, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
                [ec.ABS_WHEEL, {"value": 0, "min": 0, "max": AXIS_MAX, "fuzz": 7, "flat": 127, "resolution": 0}],
            ],
            # EV_MSC
            "4": [
                ec.MSC_SCAN,
            ]
        },
        # Radiomaster Pocket
        "vendor": 0x1209,
        "product": 0x4f54,
    },
]
BUTTONS = DEVICES_JSON[0]["capabilities"]["1"]
AXES = [a[0] for a in DEVICES_JSON[0]["capabilities"]["3"]]

def unpack_crsf_from_bytes(data) -> bytes:
    channels = []
    src_shift = 0
    ptr = 0
    for i in range(16):
        value = data[ptr] >> src_shift
        ptr += 1

        value_bits_left = 11 - 8 + src_shift
        value = (value | (data[ptr] << (11 - value_bits_left))) & 0x7ff
        if value_bits_left >= 8:
            ptr += 1
            value_bits_left -= 8
            if value_bits_left:
                value = (value | (data[ptr] << (11 - value_bits_left))) & 0x7ff

        src_shift = value_bits_left

        channels.append(value)

    return channels


def do_controller_mapping(devices, old_channels, channels):
    """
    Do custom mapping from channels to evdev ABS/KEY.
    """
    # 0  AIL
    if channels[0] != old_channels[0]:
        devices[0].write(EV_ABS, AXES[0], channels[0])
    # 1  ELE
    if channels[1] != old_channels[1]:
        devices[0].write(EV_ABS, AXES[1], channels[1])
    # 2  THR
    if channels[2] != old_channels[2]:
        devices[0].write(EV_ABS, AXES[2], channels[2])
    # 3  RUD
    if channels[3] != old_channels[3]:
        devices[0].write(EV_ABS, AXES[3], channels[3])
    # 4  SD disarm/arm button(s)
    if channels[4] != old_channels[4]:
        devices[0].write(EV_KEY, BUTTONS[0], bool(channels[4] < AXIS_MID))
        devices[0].write(EV_KEY, BUTTONS[1], bool(channels[4] >= AXIS_MID))
        devices[0].write(EV_ABS, AXES[4], channels[4])
    # 5  button SE (2POS, momentary)
    if channels[5] != old_channels[5]:
        devices[0].write(EV_KEY, BUTTONS[2], bool(channels[5] >= AXIS_MID))
    # 6  S1-pot (axis)
    if channels[6] != old_channels[6]:
        devices[0].write(EV_ABS, AXES[5], channels[6])
    # 7  button SA (2POS, fixed)
    if channels[7] != old_channels[7]:
        devices[0].write(EV_KEY, BUTTONS[11], bool(channels[7] < AXIS_MID))
        devices[0].write(EV_KEY, BUTTONS[12], bool(channels[7] >= AXIS_MID))
        devices[0].write(EV_ABS, AXES[6], channels[7])
    # 9  trim RUD (3POS, momentary)
    if channels[8] != old_channels[8]:
        devices[0].write(EV_KEY, BUTTONS[3], bool(channels[8] <= AXIS_3POS_LEFT))
        devices[0].write(EV_KEY, BUTTONS[4], bool(channels[8] >= AXIS_3POS_RIGHT))
    # 10 trim ELE (3POS, momentary)
    if channels[9] != old_channels[9]:
        devices[0].write(EV_KEY, BUTTONS[5], bool(channels[9] <= AXIS_3POS_LEFT))
        devices[0].write(EV_KEY, BUTTONS[6], bool(channels[9] >= AXIS_3POS_RIGHT))
    # 11 trim THR (3POS, momentary)
    if channels[10] != old_channels[10]:
        devices[0].write(EV_KEY, BUTTONS[7], bool(channels[10] <= AXIS_3POS_LEFT))
        devices[0].write(EV_KEY, BUTTONS[8], bool(channels[10] >= AXIS_3POS_RIGHT))
    # 12 trim AIL (3POS, momentary)
    if channels[11] != old_channels[11]:
        devices[0].write(EV_KEY, BUTTONS[9], bool(channels[11] <= AXIS_3POS_LEFT))
        devices[0].write(EV_KEY, BUTTONS[10], bool(channels[11] >= AXIS_3POS_RIGHT))
    # ------ channel 12-15 aren't sent in ELRS wide mode
    # unassigned buttons
    #   button SA (2POS, fixed, push, left)
    #   button SB (3POS, fixed, switch, left)
    #   button SC (3POS, fixed, switch, right)


old_channels = [-1] * 16
def handle_crsf_packet(devices, ptype, data):
    if ptype == PacketsTypes.RC_CHANNELS_PACKED and len(data) == 22:
        global old_channels
        channels = unpack_crsf_from_bytes(data)
        #print(f"Channels: {' '.join(('%3d' % val) for val in channels)}")
        if any(value > AXIS_MAX for value in channels):
            print(f"Error: channel out of range (values {channels})")
            return

        do_controller_mapping(devices, old_channels, channels)
        devices[0].syn()

        old_channels = channels

if __name__ == '__main__':
    sock_crsf = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock_crsf.bind(BIND)
    print(f'Listening on {BIND}')

    devices = []
    for device_json in DEVICES_JSON:
        capabilities = {}
        for k, v in device_json['capabilities'].items():
            capabilities[int(k)] = [x if not isinstance(x, list) else (x[0], evdev.AbsInfo(**x[1])) for x in v]
        devices.append(evdev.UInput(capabilities, name=device_json['name'], vendor=device_json['vendor'], product=device_json['product']))

    time.sleep(1) # wait for udev to settle
    for device in devices:
        os.chmod(device.device.path, 0o664)

    print('Devices created:')
    for i,device in enumerate(devices):
        print(f'  {i:3d} {device.device.path} {device.device.name}')

    # initialize mapper thread for return telemetry
    sock_tel, desc = telemetry_socket()
    config = json.loads(desc)
    parser = TelemetryParser(config['StreamFormat'])

    evt_quit = threading.Event()
    thread_ka = threading.Thread(target=telemetry_keepalive, args=(sock_tel, evt_quit))
    thread_ka.start()
    thread_rt = threading.Thread(target=telemetry_liftoff_to_crsf, args=(parser, sock_tel, sock_crsf))
    thread_rt.start()

    # main loop
    try:
        cur_target_addr = None
        while True:
            try:
                data, addr = sock_crsf.recvfrom(254)
            except ConnectionRefusedError:
                continue
            if cur_target_addr != addr:
                print('Connecting return socket ', addr)
                sock_crsf.connect(addr)
                cur_target_addr = addr
            if len(data) > 1:
                handle_crsf_packet(devices, data[0], data[1:])
    finally:
        evt_quit.set()

        sock_tel.close()
        sock_crsf.close()

        thread_ka.join()
        thread_rt.join()
