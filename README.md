# RPLIDAR servo
This code was made for RC car avoids obstacles. This algorithm use only 0 to 90 degrees from lidar data.
## Prerequisite
### Hardware
0. Arduino Mega 2560(I used 2 serial bus)
1. RPLIDAR A1 or A2(tested both of them. A2 works better.)
2. Servo motor
3. PC(I used Windows 10)

### Arduino pins
Devices|pin number
--------|---------
RPLIDAR|3 
Servo motor pwm|7
Throttle motor pwm|8

### Software
0. Arduino IDE
1. RPLIDAR Library
[RPDLIDAR github repository](https://github.com/robopeak/rplidar_arduino)

## Library install
- `git clone https://github.com/robopeak/rplidar_arduino.git`
- `cd rplidar_arduino`
- `mv RPLidarDriver ~/Arduino`

## Upload to arduino
0. Launch Arduino IDE
1. Open rplidar_servo.ino
2. Set the arduino board to Arudino Mega 2560
3. Set the COM port(serial port) 
4. Press Ctrl+u
