#!/usr/bin/python3
'''
Module for parsing UDP telemetry packets from the game Liftoff.
Documented in: https://steamcommunity.com/sharedfiles/filedetails/?id=3160488434
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
import json
from math import atan2, sqrt, pi
from collections import namedtuple
import socket
import struct
from sys import stdout

def _unpack_floats(data: bytes):
	'''Unpack bytes to little-endian floats'''
	# XXX this could definitely be sped up using numpy.
	return struct.unpack('<' + ('f' * (len(data)//4)), data)

class TelemetryParser:
	ATTRIBUTES = {
		"Timestamp": float,
		"Position":  (3, float), # PositionX PositionY PositionZ
		"PositionX": float,
		"PositionY": float,
		"PositionZ": float,
		"Attitude":  (4, float), # AttitudeX AttitudeY AttitudeZ AttitudeW
		"AttitudeX":  float,
		"AttitudeY":  float,
		"AttitudeZ":  float,
		"AttitudeW":  float,
		"Velocity":  (3, float), # SpeedX SpeedY SpeedZ
		"SpeedX":  float,
		"SpeedY":  float,
		"SpeedZ":  float,
		"Gyro": (3, float),      # GyroPitch GyroRoll GyroYaw
		"GyroPitch": float,
		"GyroRoll": float,
		"GyroYaw": float,
		"Input": (4, float),     # InputThrottle InputYaw InputPitch InputRoll
		"InputThrottle": float,
		"InputYaw": float,
		"InputPitch": float,
		"InputRoll": float,
		"Battery": (2, float),   # BatteryPercentage BatteryVoltage
		"BatteryPercentage": float,
		"BatteryVoltage": float,
		"MotorRPM": (0, float),  # dynamic: 1 byte + 4 floats (left front, right front, left back, right back)
	}

	def __init__(self, stream_format):
		for attr_name in stream_format:
			if attr_name not in self.ATTRIBUTES:
				raise ValueError(f"(attr_name) is not a supported telemetry attribute")
		self.stream_format = stream_format
		self.tuple = namedtuple('TelemetryPacket', stream_format)

	def parse(self, packet):
		rv = [None] * len(self.stream_format)
		ptr = 0
		for i, attr_name in enumerate(self.stream_format):
			attr_desc = self.ATTRIBUTES[attr_name]
			if attr_desc is float:
				size = 4
				data = _unpack_floats(packet[ptr:ptr+size])[0]
			elif isinstance(attr_desc, tuple):
				assert attr_desc[1] is float
				if attr_desc[0] == 0: # dynamic-sized tuple
					num = packet[ptr]
					ptr += 1
				else: # fixed-size tuple
					num = attr_desc[0]
				size = 4 * num
				data = _unpack_floats(packet[ptr:ptr+size])
			else:
				raise NotImplemented
			rv[i] = data
			ptr += size
		return self.tuple(*rv)

def telemetry_socket():
    from telemetry_config import DESC, TELEMETRY_BIND, TELEMETRY_ROUTER
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    # XXX keepalive thread
    if TELEMETRY_ROUTER:
        sock.connect(TELEMETRY_ROUTER)
        sock.send(b'\x00') # Request data
    else:
        sock.bind(TELEMETRY_BIND)
    return sock, DESC


if __name__ == '__main__':
    sock, desc = telemetry_socket()
    config = json.loads(desc)
    parser = TelemetryParser(config['StreamFormat'])

    GLYPH_DIRECTIONS='↑↗→↘↓↙←↖'
    GLYPH_STATIONARY='·'
    # +Z  N
    # -Z  S
    # +X  E
    # -X  W

    while True:
        data = sock.recv(4096)

        rec = parser.parse(data)
        angle = atan2(rec.Velocity[0], rec.Velocity[2])
        magnitude = sqrt(rec.Velocity[0]*rec.Velocity[0] + rec.Velocity[2]*rec.Velocity[2])
        if magnitude < 0.1:
            dir_glyph = GLYPH_STATIONARY
        else:
            glyph_idx = round(angle / (2.0 * pi) * 8.0) % 8
            dir_glyph = GLYPH_DIRECTIONS[glyph_idx]
        stdout.write(f' XZ {rec.Position[0]:6.1f} {rec.Position[2]:6.1f} ({dir_glyph} {rec.Velocity[0]:5.1f} {rec.Velocity[2]:5.1f}) alt: {rec.Position[1]:5.1f} ({rec.Velocity[1]:5.1f})\33[K\r')
        stdout.flush()
