# OpenGS - The Open Source Ground Station Project
![logo](https://github.com/nik012003/OpenGS/raw/master/resources/OpenGS-title.png)
# How does it work?
OpenGS Client runs on you Linux/MacOs computer or your Android device.

The Client gets all the Telemetry data and sends its requests to the Server, running on a Linux-based computer on the aircraft(e.g. Raspberry Pi).

The Server is connected via a serial link to the Flight Controller running iNav and it can sand and receive data thanks to the Protocol Handler. In this case we are using MSP v2.  (documentation for the protocol can be found here: https://github.com/iNavFlight/inav/wiki/MSP-V2)

# Status
Client ❌ Development has not started yet

Server ❌ Development has not started yet

Protocol handler ✅ Development has started

# How to set-up Client
# How to set-up Server
# Documentation for the Protocol handler
Required modules : pyserial, MSP
```
pip3 install pyserial
git clone https://github.com/nik012003/OpenGS
cd OpenGS/Protocol_handler
python3 setup.py install
```
Example code:
```python
from MSP import MSP
from serial import Serial
ser = Serial('/dev/tty.SETPORTHERE', baudrate=115200 , timeout=0.1) #initialize serial connection
copter = MSP(ser)
copter.get_attitude() #get copter attitude(roll, pitch, yaw)
```
