#include <Servo.h>

Servo myservo;  // create servo object to control a servo

// const int servoPin = 9;
const int servoMin = 0;
const int servoMax = 180; // make smaller if it doesn't work

unsigned long long timeToPublishAngle = 0;
unsigned long lastUpdateTime = 0;
const int updateInterval = 15;
unsigned long timeToMoveServo = 0;

int pos;
int targetPos;

String message;

void setup() {
  Serial.begin(115200);
  // while(!Serial){
  //   ;
  // }
  Serial.setTimeout(1);
  myservo.attach(9);  
  // Serial.println("SERVO_ANGLE:" + String(myservo.read()));
}

void loop() {
  if(Serial.available()){
    // Serial.println("HELLOOOOO");
    message = Serial.readString();

    if(message.equals("GRIPPER_LOAD")){
      targetPos = servoMax;
    } else if (message.equals("GRIPPER_RELEASE")){
      targetPos = servoMin;
    }

    timeToPublishAngle = millis() + 15.0l;
  }

  int currentPos = myservo.read();

  unsigned long currentTime = millis();

  if(timeToMoveServo < currentTime) {
    timeToMoveServo = currentTime + 15;

    if(currentPos < targetPos) {
      myservo.write(currentPos + 1);
    } else if (currentPos > targetPos) {
      myservo.write(currentPos - 1);
    }
  }

  currentTime = millis();

  // if(myservo.read() > servoMin && myservo.read() < servoMax && timeToPublishAngle < currentTime) {
  //   Serial.println("SERVO_ANGLE:" + String(myservo.read()));

  //   timeToPublishAngle = currentTime + 50.0l;
  // } 

}