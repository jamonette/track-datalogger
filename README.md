## *Track datalogger*

### Purpose

Log GPS, IMU, and CANBUS data to an SD card.

### Hardware requirements

- MCU: `Arduino Mega 2560`
    - Used for its multiple hardware UART's - could get by with a smaller
      device using software serial
- CAN interface: `MCP2515`
- GPS: `MTK3329`
- IMU: `MPU6050`

### Todo

- Implement CAN logging

- Evaluate difference in accuracy between logging GPS VTG data directly
  vs. deriving PIT velocity from position vs. time

- Maybe add on-device lap timer display functionality?
