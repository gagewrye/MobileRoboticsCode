# Mobile Robotics Code

Code for CS 181R---Mobile Robotics---at Pomona College.

## Directory Structure

```text
.
├── Examples
│  ├── MotorControl
│  ├── WSHeartbeat
│  └── WSHeartbeatRefactored
├── include
│  ├── motorcontrol.h
│  └── wscommunicator.h
└── README.md
```

The Arduino-based motor code is split into the `Examples` and `include` directories. The `include` directory contains headers files for use in all complete programs.

## Dependencies

You will need to install

- [Arduino](https://www.arduino.cc/en/software)
- The [Espressif Systems ESP32 board manager](https://github.com/espressif/arduino-esp32)
- [WebSocket Server and Client for Arduino](https://github.com/Links2004/arduinoWebSockets)

Setup:
1. Install Toit and Jaguar
2. Initialize Jaguar: jag setup
3. Plug your XIAO ESP32-S3 board into your computer
4. Flash the board: 
    - jag flash --chip esp32s3 (SSID: Claremont-ETC; password on Slack)
5. Clone the course code: 
    - git clone https://github.com/anthonyjclark/MobileRoboticsCode
6. In two separate terminals:
    Monitor the serial output with: 
    - jag monitor
    Run a program with: 
    - jag run FILE --device IP

Use the following command to save the Wi-Fi credentials so that you only need to type them once:
- jag config wifi set --wifi-ssid SSID --wifi-password PASSWORD

Get IP:
- jag device ls

To Run:
# In terminal 1
- jag monitor
# In terminal 2
- cd ~/Classes/MobileRobotics/MobileRoboticsCode/led
- jag run demo.toit --device IP

Remote Connect:
- wscat --connect ws://10.128.5.87:8181


Arduino setup:
1. Install [Arduino](https://www.arduino.cc/en/software) IDE