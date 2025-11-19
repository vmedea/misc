#!/usr/bin/env python3
'''
Forward CRSF data from Radiomaster RP2 over UDP.
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
from collections import deque
import serial
import socket
import time
import argparse
import sys

# 420000 is the default baudrate for CSRF
# When using linux's built-in CH341 USB uart driver, using 422000
# may give less CRC errors.
# Alternatively, one can use https://github.com/WCHSoftGroup/ch341ser_linux
DEFAULT_BAUDRATE = 420000
DEFAULT_DEST = '127.0.0.1:9005'
DEFAULT_SRC = '127.0.0.1:9006'
STATS_INTERVAL = 1.0
STATS_DISPLAY_INTERVAL = 0.2

CRSF_SYNC = bytes([0xC8])

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

parser = argparse.ArgumentParser()
parser.add_argument('-P', '--port', default='/dev/ttyUSB0', required=False)
parser.add_argument('-b', '--baud', default=DEFAULT_BAUDRATE, required=False)
parser.add_argument('-d', '--dest', default=DEFAULT_DEST, required=False)
parser.add_argument('-s', '--src', default=DEFAULT_SRC, required=False)
args = parser.parse_args()

stats = deque()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest_ip, dest_port = args.dest.rsplit(':', 2)
dest_port = int(dest_port)
sock.connect((dest_ip, dest_port))
sock.setblocking(False)

# extra telemetry socket
tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
src_ip, src_port = args.src.rsplit(':', 2)
src_port = int(src_port)
tel_sock.bind((src_ip, src_port))
tel_sock.setblocking(False)

with serial.Serial(args.port, args.baud, timeout=2,
                  bytesize=8, parity='N', stopbits=1,
                  xonxoff=0, rtscts=0 ) as ser:
    buffer = bytearray()
    next_stats_display = 0
    while True:
        # handle telemetry return channel (low prio)
        # XXX it would be better to handle this in a seperate threaad
        # to prevent interrupting serial reads?
        while True:
            packet = None
            for s in [sock, tel_sock]:
                try:
                    # receive packet without sync byte, length and crc
                    # we will add these ourselves
                    packet = s.recv(254)
                except BlockingIOError:
                    pass
            if packet is None: # break receive loop if no packet received
                break

            packet = CRSF_SYNC + bytes([len(packet) + 1]) + packet + bytes([crc8_data(packet)])
            assert crsf_validate_frame(packet)
            #print('tel', packet.hex())
            ser.write(packet)
            stats.append((time.monotonic(), -2))

        # read one byte, then read rest of available bytes
        buffer.extend(ser.read(1))
        buffer.extend(ser.read(ser.in_waiting))

        ofs = 0
        while True:
            ofs = buffer.find(CRSF_SYNC, ofs)
            if ofs == -1 or (ofs + 2) >= len(buffer):
                break
            expected_len = buffer[ofs + 1] + 2
            if (ofs + expected_len) > len(buffer):
                break
            single = buffer[ofs:ofs + expected_len]
            if not crsf_validate_frame(single):
                #packet = ' '.join(map(hex, single))
                #print(f"crc error: {packet}")
                stats.append((time.monotonic(), -1))
            else:
                stats.append((time.monotonic(), len(single)))
                # don't send sync, length, checksum bytes
                try:
                    sock.send(single[2:-1])
                except ConnectionRefusedError:
                    pass
                except BlockingIOError:
                    pass
            ofs += expected_len

        buffer = buffer[ofs:]

        # compute and print stats
        now = time.monotonic()
        cutoff = now - STATS_INTERVAL
        while stats and stats[0][0] < cutoff:
            stats.popleft()

        if now >= next_stats_display:
            num_packets = sum(1 for j in stats if j[1] >= 0)
            num_bytes = sum(j[1] for j in stats if j[1] >= 0)
            num_errors = sum(1 for j in stats if j[1] == -1)
            num_telemetry = sum(1 for j in stats if j[1] == -2)
            sys.stdout.write(f'[↓] pkt {num_packets:6d} byt {num_bytes:6d} bit {num_bytes*8:6d} err {num_errors:6d} [↑] tel {num_telemetry:6d}\33[K\r')
            sys.stdout.flush()

            next_stats_display = now + STATS_DISPLAY_INTERVAL
