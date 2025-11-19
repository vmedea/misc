'''
Geo-from-Unity utilitites.
'''
import math
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
def gps_from_coord(coord, base=(0.0, 0.0)):
    '''
    3D position to longitide (degree), lattitude (degree), altitude (meter).
    '''
    x = coord[0]
    y = coord[2]
    # If your displacements aren't too great (less than a few kilometers) and you're
    # not right at the poles, use the quick and dirty estimate that 111,111 meters
    # (111.111 km) in the y direction is 1 degree (of latitude) and 111,111 *
    # cos(latitude) meters in the x direction is 1 degree (of longitude).
    latitude = base[1] + y / 111111.0
    longitude = base[0] + x / (111111.0 * math.cos(math.radians(latitude)))
    altitude = coord[1]

    return longitude, latitude, altitude

def quat2heading(q0:float, q1:float, q2:float, q3:float) -> float:
    '''
    From a quaternion, return heading in XZ plane.
    '''
    return math.atan2(
        2 * ((q2 * q0) + (q3 * q1)),
        q3**2 + q2**2 - q0**2 - q1**2
    )

def quat2eulers(qx:float, qy:float, qz:float, qw:float):
    # swap (liftoff y-up to imu coordinate system z-up)
    (qx, qy, qz, qw) = (qx, qz, qy, -qw)

    #    betaflight: imuComputeRotationMatrix, imuUpdateEulerAngles in src/main/flight/imu.c
    m00 = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz
    m10 = 2.0 * (qx * qy + qw * qz)
    m20 = 2.0 * (qx * qz - qw * qy)
    m21 = 2.0 * (qy * qz + qw * qx)
    m22 = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy

    roll = math.atan2(m21, m22)
    pitch = (0.5 * math.pi) - math.acos(-m20)
    yaw = -math.atan2(m10, m00)

    return (roll, pitch, yaw)

if __name__ == '__main__':
    # N  0.000, 0.000, 0.000, 1.000     0
    # W  0.000,-0.707, 0.000, 0.707   -90
    # S  0.000,-1.000, 0.000, 0.000   180
    # E  0.000,-0.707, 0.000,-0.707    90
    assert round(math.degrees(quat2heading(0.000, 0.000, 0.000, 1.000))) == 0
    assert round(math.degrees(quat2heading(0.000,-0.707, 0.000, 0.707))) == -90
    assert round(math.degrees(quat2heading(0.000,-1.000, 0.000, 0.000))) == 180
    assert round(math.degrees(quat2heading(0.000,-0.707, 0.000,-0.707))) == 90

    #print(quat2eulers(0.000, 0.000, 0.000, 1.000))
    #print(quat2eulers(0.000,-0.707, 0.000, 0.707))
    #print(quat2eulers(0.000,-1.000, 0.000, 0.000))
    #print(quat2eulers(0.000,-0.707, 0.000,-0.707))
