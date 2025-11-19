# CRSF spec: https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x16-rc-channels-packed-payload
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
from enum import IntEnum

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    AIRSPEED = 0x0A
    HEARTBEAT = 0x0B
    RPM = 0x0C
    TEMP = 0x0D
    VOLTAGES = 0x0E
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    LINK_STATISTICS_RX = 0x1C
    LINK_STATISTICS_TX = 0x1D
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def us_to_ticks(x):
    return ((x - 1500) * 8 / 5 + 992)

def ticks_to_us(x):
    return ((x - 992) * 5 / 8 + 1500)
