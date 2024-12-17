#include <Servo.h>

Servo gripperServoHorisontal;
Servo gripperServoVertical;

const int gripperServoHorisontalPin = 9;
const int gripperServoVerticalPin = 10;

const int gripperServoHorisontalMin = 23; // this is fully open
const int gripperServoHorisontalMax = 2; // this is fully closed

const int gripperServoVerticalMin = 0;
const int gripperServoVerticalMax = 180;

const int gripperClosingDelay = 3000; // when loading the sensor box, wait this long before closing the gripper
unsigned long timeToCloseGripper;
const int gripperOpeningDelay = 1500; // when installing the sensor box, lower the gripper and wait this long before opening the fingers
unsigned long timeToOpenGripper;
const int gripperLiftingDelay = 2000; // when installing the sensor box, open the fingers and wait this long to raise the gripper
unsigned long timeToLiftGripper;

bool canCloseGripper = false;
bool canOpenGripper = false;
bool canLiftGripper = false;

String command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);

  gripperServoHorisontal.attach(gripperServoHorisontalPin);  
  gripperServoVertical.attach(gripperServoVerticalPin);

  gripperServoHorisontal.write(gripperServoHorisontalMin);
  gripperServoVertical.write(gripperServoVerticalMin);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    if (command.equals("GRIPPER_LOAD")){
      gripperServoHorisontal.write(gripperServoHorisontalMin);

      canCloseGripper = true;
      timeToCloseGripper = millis() + gripperClosingDelay;
    } else if (command.equals("GRIPPER_RELEASE")){
      gripperServoHorisontal.write(gripperServoHorisontalMin);
    } else if (command.equals("GRIPPER_PLACE")) {
      gripperServoVertical.write(gripperServoVerticalMax);

      canOpenGripper = true;
      timeToOpenGripper = millis() + gripperOpeningDelay;
    }
  }

  if (canCloseGripper && (timeToCloseGripper < millis())){
    gripperServoHorisontal.write(gripperServoHorisontalMax);
    canCloseGripper = false;
  }

  if (canOpenGripper && (timeToOpenGripper < millis())){
    gripperServoHorisontal.write(gripperServoHorisontalMin);
    canOpenGripper = false;

    canLiftGripper = true;
    timeToLiftGripper = millis() + gripperLiftingDelay;
  }

  if (canLiftGripper && (timeToLiftGripper < millis())){
    gripperServoVertical.write(gripperServoVerticalMin);
    canLiftGripper = false;
  }
}