# liftoff

This directory has various scripts for CRSF joystick and telemetry scripts for the quadcopter sim "liftoff":

- `crsf.py`: Python CRSF utility library
- `fake_geo.py`: Produce GPS-annotated coordinates for using OpenDroneMap on liftoff screenshots
- `forward_crsf.py`: Forward CRSF control and telemetry between a ELRS receiver and UDP socket
- `geo_util.py`: Utility module to fake GPS coordinates from liftoff spatial coordinates
- `liftoff_gpsd.py`: gpsd emulator for viewing liftoff telemetry in QGIS
- `liftoff_screenshot_lin.py`: Make screenshots, storing last telemetry packet
- `liftoff_telemetry.py`: Utility module for handling liftoff's telemetry stream
- `liftoff_telemetry_crsf.py`: Utility module for converting between liftoff's telemetry and CRSF telemetry
- `liftoff_telemetry_router.py`: Telemetry hub (to receive liftoff's telemetry in multiple scripts)
- `remote_input_server_crsf.py`: CRSF joystick. Receives CRSF packets over UDP and simulates a linux udev joystick. Sends back telemetry from liftoff to UDP
- `telemetry_config.py.example`: Example configuration. Copy this to `telemetry_config.py`

