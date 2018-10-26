#include <Servo.h>
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

//#define DEBUG
#define KP 1.5

// You need to create an driver instance
Servo steerServo;
Servo throttleMotor;
RPLidar lidar;

// Change the pin mapping based on your needs.
/////////////////////////////////////////////////////////////////////////////
#define RPLIDAR_MOTOR  3 // The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal
#define STEER_SERVO    7 // The PWM pin for contorl the angle of steering servo motor.
// This pin should connected with the servo motor's signal pin(white).
#define THROTTLE_MOTOR 8 // The PWM pin for contorl the speed of throttle dc motor.
// This pin should connected with the servo motor's signal pin(white).
//////////////////////////////////////////////////////////////////////////////
void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial.begin(115200);
  lidar.begin(Serial3);
  steerServo.attach(STEER_SERVO);
  throttleMotor.attach(THROTTLE_MOTOR);

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
int steerAngle = 0;
int throttleSpeed = 1600;

void loop() {
  if(throttleSpeed > 1500){
    throttleSpeed -= 1;
  }
  throttleMotor.write(throttleSpeed);
  
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
      steerAngle = 90 + angleAtMaxDist * KP;
      steerServo.write(steerAngle);
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
  }
  else {
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
