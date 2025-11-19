#!/usr/bin/python3
'''
Receive telemetry from liftoff, send it to CRSF listener.
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
import json
from collections import namedtuple
import math
import socket
import struct
import threading
import time

from crsf import PacketsTypes
from geo_util import gps_from_coord, quat2heading, quat2eulers
from liftoff_telemetry import TelemetryParser, telemetry_socket

CRSF_TELEMETRY_INTERVAL = 0.1

def telemetry_keepalive(sock, quit_event):
    keepalive_interval = 30.0
    while True:
        if quit_event.wait(keepalive_interval):
            break
        sock.send(b'\x00')

if __name__ == '__main__':
    sock_tel, desc = telemetry_socket()
    config = json.loads(desc)
    parser = TelemetryParser(config['StreamFormat'])

    sock_crsf = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_crsf.connect(('127.0.0.1', 9006))

    evt_quit = threading.Event()
    thread_ka = threading.Thread(target=telemetry_keepalive, args=(sock_tel, evt_quit))
    thread_ka.start()

    try:
        next_send = 0.0
        while True:
            data = sock_tel.recv(4096)

            now = time.monotonic()
            if now >= next_send:
                rec = parser.parse(data)

                (longitude, latitude, altitude) = gps_from_coord(rec.Position)
                mag_heading = math.degrees(quat2heading(rec.Attitude[0], rec.Attitude[1], rec.Attitude[2], rec.Attitude[3]))
                if mag_heading < 0.0: # Prevent negative numbers
                    mag_heading += 360.0

                # supported by edgetx (radio/src/telemetry/crossfire.cpp):
                # VARIO_ID          0x07 VARIO                    VSpd
                # GPS_ID            0x02 GPS                      GPS(lat,long) GSpd Hdg Sats
                # BARO_ALT_ID       0x09 BARO_ALT                 Alt
                # AIRSPEED_ID       0x0A AIRSPEED                 ASpd
                # CF_RPM_ID         0x0C RPM                      RPM
                # TEMP_ID           0x0D TEMP                     Temp
                # CELLS_ID          0x0E VOLTAGES                 Cels
                # LINK_ID           0x14 LINK_STATISTICS          1RSS 2RSS RQly RSNR ANT RFMD TPWR TRSS TQly TSNR (from radio, not from drone)
                # LINK_RX_ID        0x1C LINK_STATISTICS_RX       RRSP RPWR 
                # LINK_TX_ID        0x1D LINK_STATISTICS_TX       TRSP TPWR TFPS
                # BATTERY_ID        0x08 BATTERY_SENSOR           RxBt Curr Capa Bat%
                # ATTITUDE_ID       0x1E ATTITUDE                 Ptch Roll Yaw
                # FLIGHT_MODE_ID    0x21 FLIGHT_MODE              FM
                #
                # betaflight writes the following (src/main/telemetry/crsf.c)
                # CRSF_FRAMETYPE_GPS
                # CRSF_FRAMETYPE_VARIO_SENSOR
                # CRSF_FRAMETYPE_BATTERY_SENSOR
                # CRSF_FRAMETYPE_BARO_ALTITUDE
                # CRSF_FRAMETYPE_HEARTBEAT
                # CRSF_FRAMETYPE_PING
                # CRSF_FRAMETYPE_ATTITUDE
                # CRSF_FRAMETYPE_FLIGHT_MODE
                # CRSF_FRAMETYPE_DEVICE_INFO

                # ground speed in m/s
                vel2d_magnitude = math.sqrt(rec.Velocity[0]*rec.Velocity[0] + rec.Velocity[2]*rec.Velocity[2])
                # air speed in m/s
                vel3d_magnitude = math.sqrt(rec.Velocity[0]*rec.Velocity[0] + rec.Velocity[1]*rec.Velocity[1] + rec.Velocity[2]*rec.Velocity[2])

                # Build GPS backet
                packet = bytes([PacketsTypes.GPS])
                packet += int(latitude * 10_000_000).to_bytes(4, byteorder='big', signed=True)
                packet += int(longitude * 10_000_000).to_bytes(4, byteorder='big', signed=True)
                # https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x02-gps specifies km/h / 100
                # however, what both edgetx and betaflight actually use is km/h / 10
                packet += int(vel2d_magnitude * 3.6 * 10).to_bytes(2, byteorder='big', signed=False) # km/h / 10
                packet += int(mag_heading * 100).to_bytes(2, byteorder='big', signed=False)
                packet += int(altitude + 1000).to_bytes(2, byteorder='big', signed=False)
                packet += bytes([1]) # 1 sat
                sock_crsf.send(packet)

                # Build battery packet
                voltage = rec.Battery[1]
                current = 0.0
                capacity_used = 0.0
                remaining = rec.Battery[0]
                packet = bytes([PacketsTypes.BATTERY_SENSOR])
                packet += int(voltage * 10.0).to_bytes(2, byteorder='big', signed=True)
                packet += int(current * 10.0).to_bytes(2, byteorder='big', signed=True)
                packet += int(capacity_used).to_bytes(3, byteorder='big', signed=False)
                packet += int(remaining * 100.0).to_bytes(1, byteorder='big', signed=False)
                sock_crsf.send(packet)

                # Build vario (vertical speed in cm/s)
                packet = bytes([PacketsTypes.VARIO])
                packet += int(rec.Velocity[1] * 100.0).to_bytes(2, byteorder='big', signed=True)
                sock_crsf.send(packet)

                # Build airspeed (airspeed in 0.1 * km/h)
                packet = bytes([PacketsTypes.AIRSPEED])
                packet += int(vel3d_magnitude * 3.6 * 10).to_bytes(2, byteorder='big', signed=False)
                sock_crsf.send(packet)

                # Build attitude (pitch, roll, yaw in 100 µrad, -pi to pi)
                packet = bytes([PacketsTypes.ATTITUDE])
                pitch, roll, yaw = quat2eulers(rec.Attitude[0], rec.Attitude[1], rec.Attitude[2], rec.Attitude[3])
                packet += int(pitch * 10000).to_bytes(2, byteorder='big', signed=True)
                packet += int(roll * 10000).to_bytes(2, byteorder='big', signed=True)
                packet += int(yaw * 10000).to_bytes(2, byteorder='big', signed=True)
                sock_crsf.send(packet)

                # build rpm packet
                packet = bytes([PacketsTypes.RPM])
                packet += int(0).to_bytes(1, byteorder='big', signed=False) # source id - fixed
                assert len(rec.MotorRPM) < 20 # Max 19 values, but liftoff will send only 4 anyway
                for rpm in rec.MotorRPM:
                    packet += int(rpm).to_bytes(3, byteorder='big', signed=False)
                sock_crsf.send(packet)

                # unused: Timestamp, Gyro, Input

                next_send = now + CRSF_TELEMETRY_INTERVAL
    finally:
        evt_quit.set()

        sock_tel.close()

        thread_ka.join()
