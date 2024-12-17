#include <Servo.h>

// CLAMP DISPENSER

Servo clampDispenserServo;

const int clampDispenserServoPin = 11;

const int clampDispenserServoMin = 0;
const int clampDispenserServoMax = 180;

const int clampPressingDuration = 5000;
unsigned long timeToReloadClamp;

bool canReloadClamp = false;

// GRIPPER

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

// WHEELS (DC MOTORS)

const int motorA1Pin = 12;
const int motorA2Pin = 13;
const int motorAPWM = 3;

String command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);

  // ATTACH SERVOS

  gripperServoHorisontal.attach(gripperServoHorisontalPin);  
  gripperServoVertical.attach(gripperServoVerticalPin);

  clampDispenserServo.attach(clampDispenserServoPin);

  // SERVO START POSITIONS

  gripperServoHorisontal.write(gripperServoHorisontalMin);
  gripperServoVertical.write(gripperServoVerticalMin);

  clampDispenserServo.write(clampDispenserServoMin);

  // INITIALISE DC MOTOR PINS
  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    command = Serial.readStringUntil(';');

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
    } else if (command.equals("DISPENSE_CLAMP")) {
      clampDispenserServo.write(clampDispenserServoMax);

      timeToReloadClamp = millis() + clampPressingDuration;
      canReloadClamp = true;
    } else if (command.equals("GRIPPER_H")) {
      command = Serial.readStringUntil(';');
      gripperServoHorisontal.write(command.toInt());
    } else if (command.equals("GRIPPER_V")) {
      command = Serial.readStringUntil(';');
      gripperServoVertical.write(command.toInt());
    } else if (command.equals("DISPENSER")) {
      command = Serial.readStringUntil(';');
      clampDispenserServo.write(command.toInt());
    } else if (command.equals("FORWARD")) {
      command = Serial.readStringUntil(';')
      driveDCMotor(true, command.toInt());
    } else if (command.equals("BACKWARD")) {
      command = Serial.readStringUntil(';')
      driveDCMotor(false, command.toInt());
    }

    Serial.println(command);
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

  if (canReloadClamp && timeToReloadClamp < millis()){
    clampDispenserServo.write(clampDispenserServoMin);
    canReloadClamp = false;
  }
}
