/*
   RoboPeak RPLIDAR Arduino Example
   This example shows how to control an RGB led based on the scan data of the RPLIDAR

   The RGB led will change its hue based on the direction of the closet object RPLIDAR has been detected.
   Also, the light intensity changes according to the object distance.

   USAGE:
   ---------------------------------
   1. Download this sketch code to your Arduino board
   2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
   3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
   4. Connect an RGB LED to your Arduino board, with the Red led to pin 9, Blue led to pin 10, Green led to pin 11
   5. Connect the required power supplies.
   6. RPLIDAR will start rotating when the skecth code has successfully detected it.
   7. Remove objects within the 0.5 meters' radius circule range of the RPLIDAR
   8. Place some object inside the 0.5 meters' range, check what will happen to the RGB led :)
*/

/*
   Copyright (c) 2014, RoboPeak
   All rights reserved.
   RoboPeak.com

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
   SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
   OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <Servo.h>
#include <RPLidar.h>

#define DEBUG

// You need to create an driver instance
Servo steer_servo;
RPLidar lidar;

// Change the pin mapping based on your needs.
/////////////////////////////////////////////////////////////////////////////
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal
#define STEER_SERVO   9 // The PWM pin for contorl the angle of steering servo motor.
// This pin should connected with the servo motor's signal pin(white).
//////////////////////////////////////////////////////////////////////////////
void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial.begin(115200);
  lidar.begin(Serial3);
  steer_servo.attach(STEER_SERVO);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

float minDistance = 100000;
float maxDistance = 0;
float angleAtMinDist = 0;
int angleAtMaxDist = 0;
int decideDistance = 0;
float lidarDistance[360] = {0,};
float rightDistance[90] = {300,};
float leftDistance[89] = {300,};

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here...
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    if (lidar.getCurrentPoint().startBit) {
      for (int i = 0; i < 90; i++) {
        if (rightDistance[i] >= 1000
        && !decideDistance) {
          maxDistance = rightDistance[i];
          angleAtMaxDist = i;
          decideDistance = 1;
        }
#ifdef DEBUG
        Serial.print(rightDistance[i]);
        Serial.print(' ');
#endif
      }
#ifdef DEBUG
      Serial.println(' ');
      Serial.println(angleAtMaxDist);
#endif
      steer_servo.write(90 + angleAtMaxDist); steer_servo.write(90 + angleAtMaxDist);
      minDistance = 100000;
      maxDistance = 0;
      angleAtMinDist = 0;
      angleAtMaxDist = 0;
      decideDistance = 0;
    }
    else {
      int index = (int)angle;
      lidarDistance[index] = distance;
      if (index < 90) {
        rightDistance[index] = distance;
      }
      else if (index >= 270) {
        leftDistance[-(index - 359)] = distance;
      }
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      //detected...
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}
