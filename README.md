# SatPointer

Takes commands from rotctl or rotctld and points an antenna.
Uses a BNO085 IMU mounted on the antenna to detect the absolute rotation compared to gravity and magnetic north.
PID control closes the loop between the motor and IMU.

Example roctld command:
.\rotctld.exe -m 204 -r COM1 -s 115200 -T 127.0.0.1 -t 4533

Satdump should be able to connect and command the device with the above ip and port.