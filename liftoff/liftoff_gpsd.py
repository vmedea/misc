#!/usr/bin/python3
'''
Emulate gpsd according to liftoff telemetry, for QGIS.
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
# gpsd protocol is documented in https://gpsd.gitlab.io/gpsd/client-howto.html
from datetime import datetime, timedelta, timezone
import json
import math
import socket
import struct
import sys
import threading
import time

from nmeasim.models import GpsReceiver
from nmeasim.constants import FixType

from geo_util import gps_from_coord, quat2heading
from liftoff_telemetry import TelemetryParser, telemetry_socket

# Update frequency (Hz)
UPDATE_FREQUENCY = 10
MAX_TELEMETRY_AGE = 1.0

last_telemetry = None

def telemetry_main(sock, parser):
    print('Starting telemetry thread')
    while True:
        data = sock.recv(4096)
        if not data: # Socket shut down
            break
        rec = parser.parse(data)

        #print(rec) # debug
        global last_telemetry
        last_telemetry = (time.monotonic(), rec)

def telemetry_keepalive(sock, quit_event):
    keepalive_interval = 30.0
    while True:
        if quit_event.wait(keepalive_interval):
            break
        print('Send keepalive')
        sock.send(b'\x00')


def read_json_token(client):
    '''
    Read one JSON token until terminating ";".
    This is complicated by the fact that this character can appear inside a valid JSON expression.
    '''
    arg = b""
    while True:
        ch = client.recv(1)
        if ch == b";":
            try:
                json.loads(arg)
            except json.JSONDecodeError as e:
                if e.pos == len(arg):
                    pass # error at end of string, keep reading
                else:
                    raise
            else:
                break # succesful - finished
        else:
            arg += ch

    return json.loads(arg)

class GPSDException(Exception):
    pass

def connection_loop(client, addr):
    try:
        print(f"New connection from {addr}")

        client.send(json.dumps(BANNER).encode() + b"\n")

        # wait for ?WATCH command
        # ?WATCH={"enable":true,"nmea":true,"raw":true};
        line = client.recv(7)
        if line != b"?WATCH=":
            raise GPSDException(f'Invalid command {line}')
        arg = read_json_token(client)
        if not arg.get('enable', False) or not arg.get('nmea', False) or not arg.get('raw', False):
            raise GPSDException(f'Unhandled watch spec {arg}, only raw nmea mode is supported for now')

        # XXX a real server would respond something like
        # {"class":"DEVICES","devices":[{"class":"DEVICE","path":"/dev/ttyUSB0",
        #          "activated":1269959537.20,"native":0,"bps":4800,"parity":"N",
        #          "stopbits":1,"cycle":1.00}]}
        # {"class":"WATCH","enable":true,"json":true,"nmea":false,"raw":0, "scaled":false,"timing":false,"pps":false}
        # but *not* in raw nmea mode.

        gps = GpsReceiver(
            date_time=datetime.now(timezone.utc),
            output=('GGA', 'GLL', 'GSA', 'GSV', 'RMC', 'VTG', 'ZDA',)
        )

        print('Starting broadcast')
        while True:
            time_rec = last_telemetry
            if not time_rec or time_rec[0] < (time.monotonic() - MAX_TELEMETRY_AGE): # too old or no telemetry, no GPS fix
                gps.fix = FixType.INVALID_FIX
            else:
                rec = time_rec[1]
                (longitude, latitude, altitude) = gps_from_coord(rec.Position)
                mag_heading = quat2heading(rec.Attitude[0], rec.Attitude[1], rec.Attitude[2], rec.Attitude[3])

                vel_heading = math.atan2(rec.Velocity[0], rec.Velocity[2])
                vel_magnitude = math.sqrt(rec.Velocity[0]*rec.Velocity[0] + rec.Velocity[2]*rec.Velocity[2])

                # Set emulated GPS from liftoff telemetry data.
                gps.fix = FixType.SPS_FIX
                gps.date_time = datetime.now(timezone.utc)
                gps.lat = latitude
                gps.lon = longitude
                gps.altitude = altitude
                gps.heading = math.degrees(vel_heading)
                gps.kph = vel_magnitude * 3600.0 / 1000.0
                gps.mag_heading = math.degrees(mag_heading)

            # Send output to client.
            for out in gps.get_output():
                client.send(out.encode() + b'\r\n')

            # Repeat sending packets
            time.sleep(1.0 / UPDATE_FREQUENCY)

    except BrokenPipeError:
        print('Connection lost')
    except GPSDException as e:
        print(f'Closing connection: {e.args[0]}')
    except Exception as e:
        print(f'Exception, closing connection: {e.args[0]}')
    finally:
        client.close()

if __name__ == '__main__':
    tel_sock, desc = telemetry_socket()
    config = json.loads(desc)
    parser = TelemetryParser(config['StreamFormat'])
    gpsd_bind = ("0.0.0.0", 2947)

    gpsd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
    gpsd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gpsd_sock.bind(gpsd_bind)
    gpsd_sock.listen(1)

    print(f'Listening for gpsd connections on {gpsd_bind}')

    evt_quit = threading.Event()
    thread_tel = threading.Thread(target=telemetry_main, args=(tel_sock, parser))
    thread_tel.start()
    thread_ka = threading.Thread(target=telemetry_keepalive, args=(tel_sock, evt_quit))
    thread_ka.start()

    BANNER = {"class":"VERSION","release":"2.93","rev":"2010-03-30T12:18:17", "proto_major":3,"proto_minor":2}
    try:
        while True:
            client, addr = gpsd_sock.accept()
            # XXX only handles one connection at a time at the moment
            connection_loop(client, addr)
    finally:
        print('Terminating')
        evt_quit.set()
        # interrupt UDP receive
        try:
            tel_sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

        gpsd_sock.close()
        tel_sock.close()

        thread_tel.join()
        thread_ka.join()
