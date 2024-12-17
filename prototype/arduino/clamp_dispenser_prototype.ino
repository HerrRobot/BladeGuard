#include <Servo.h>

Servo clampDispenserServo;

const int clampDispenserServoPin = 11;

const int clampDispenserServoMin = 0;
const int clampDispenserServoMax = 180;

const int clampPressingDuration = 5000;
unsigned long timeToReloadClamp;

bool canReloadClamp = false;

String command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);

  clampDispenserServo.attach(clampDispenserServoPin);  

  clampDispenserServo.write(clampDispenserServoMin);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    command = Serial.readStringUntil('\n');

    if (command.equals("DISPENSE_CLAMP")) {
      clampDispenserServo.write(clampDispenserServoMax);

      timeToReloadClamp = millis() + clampPressingDuration;
      canReloadClamp = true;
    } else if (command.equals("MIN")) {
      
    }

    Serial.println(command);
  }

  if (canReloadClamp && timeToReloadClamp < millis()){
    clampDispenserServo.write(clampDispenserServoMin);
    canReloadClamp = false;
  }
}
