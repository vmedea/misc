#!/usr/bin/python3
'''
Generate a ODM geo.txt file from liftoff telemetry.
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
import json
from liftoff_telemetry import TelemetryParser
import math
from pathlib import Path
import struct
import sys

from geo_util import gps_from_coord, quat2heading

dirname = sys.argv[1]
projdir = Path(dirname)
geo_data = []

# <projection>
#image_name geo_x geo_y [geo_z] [yaw (degrees)] [pitch (degrees)] [roll (degrees)] [horz accuracy (meters)] [vert accuracy (meters)] [extras...]
for filename in sorted((projdir/"images").iterdir()):
    if filename.suffix == '.jpg' and not filename.name.startswith('.'):
        telemetry_file = filename.with_suffix('.telemetry')
        with open(telemetry_file, 'rb') as f:
            l,= struct.unpack('<i', f.read(4))
            telemetry_json = f.read(l)
            config = json.loads(telemetry_json)
            data = f.read()

        parser = TelemetryParser(config['StreamFormat'])
        rec = parser.parse(data)
        (longitude, latitude, altitude) = gps_from_coord(rec.Position)

        # XXX could compute this from pos.Attitude quaternion
        # +Z  N / -Z  S / +X  E / -X  W
        # Yaw: 0 --> top of image points north
        # Yaw: 90 --> top of image points east
        # Yaw: 270 --> top of image points west
        # Pitch: 0 --> nadir camera
        # Pitch: 90 --> camera is looking forward
        # Roll: 0 (assuming gimbal)
        heading = quat2heading(rec.Attitude[0], rec.Attitude[1], rec.Attitude[2], rec.Attitude[3])

        yaw = math.degrees(heading) # we only care about heading, for now
        pitch = 0.0
        roll = 0.0

        geo_data.append((filename.name, longitude, latitude, altitude, yaw, pitch, roll))
        print(f'{filename.name} {rec.Position[0]:5.0f} {rec.Position[2]:5.0f} {yaw:6.1f}')

geo_filename = projdir / "geo.txt"
with open(geo_filename, 'w') as f:
    f.write('EPSG:4326\n')
    for (filename, longitude, latitude, altitude, yaw, pitch, roll) in geo_data:
        # longitude is X and latitude is Y
        f.write(f'{filename} {longitude} {latitude} {yaw} {pitch} {roll}\n')
