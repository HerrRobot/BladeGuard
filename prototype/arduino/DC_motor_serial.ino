const int motorA1Pin = 12;
const int motorA2Pin = 13;
const int motorAPWM = 3;

bool isForward;
int speed;

String command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(2);

  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    command = Serial.readStringUntil('-');

    if (command.equals("FORWARD") || command.equals("BACKWARD")){
      if (command.equals("FORWARD")){
        isForward = true;
      } else {
        isForward = false;
      }

      speed = Serial.readStringUntil('\n').toInt();

      driveDCMotor(isForward, speed);
    } else {
      driveDCMotor(true, 0);
    }
  }
}

void driveDCMotor(bool driveForward, int speed) {
  digitalWrite(motorA1Pin, driveForward);
  digitalWrite(motorA2Pin, !driveForward);
  analogWrite(motorAPWM, speed);
}
