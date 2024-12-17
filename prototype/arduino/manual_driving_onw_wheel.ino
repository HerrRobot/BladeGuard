const int wheelFrontLeftPin1 = 12;
const int wheelFrontLeftPin2 = 13;
const int wheelFrontLeftPinPWM = 3;

// const int wheelBackLeftPin1 = 12;
// const int wheelBackLeftPin2 = 13;
// const int wheelBackLeftPinPWM = 3;

const int wheelFrontRightPin1 = 7;
const int wheelFrontRightPin2 = 8;
const int wheelFrontRightPinPWM = 9;

// const int wheelBackRightPin1 = 12;
// const int wheelBackRightPin2 = 13;
// const int wheelBackRightPinPWM = 3;

int leftWheelsSpeed;
int rightWheelsSpeed;

bool leftWheelsState1;
bool leftWheelsState2;
bool rightWheelsState1;
bool rightWheelsState2;

String command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(3);

  initialisePinModes();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    command = Serial.readStringUntil(';');

    if (command.equals("W_L")) {
      command = Serial.readStringUntil(';');

      decodeState(command, true);

      command = Serial.readStringUntil(';');

      driveDCMotor(wheelFrontLeftPin1, wheelFrontLeftPin2, wheelFrontLeftPinPWM, leftWheelsState1, leftWheelsState2, command.toInt());
      // driveDCMotor(wheelBackLeftPin1, wheelBackLeftPin2, wheelBackLeftPinPWM, leftWheelsState1, leftWheelsState2, command.toInt());
    } 
    else if (command.equals("W_R")) {
      command = Serial.readStringUntil(';');

      decodeState(command, false);

      command = Serial.readStringUntil(';');

      driveDCMotor(wheelFrontRightPin1, wheelFrontRightPin2, wheelFrontRightPinPWM, rightWheelsState1, rightWheelsState2, command.toInt());
      // driveDCMotor(wheelBackRightPin1, wheelBackRightPin2, wheelBackRightPinPWM, rightWheelsState1, rightWheelsState2, command.toInt());
    }
  }
}

void driveDCMotor(int wheelPin1, int wheelPin2, int wheelPWMPin, bool state1, bool state2, int speed) {
  digitalWrite(wheelPin1, state1);
  digitalWrite(wheelPin2, state2);
  analogWrite(wheelPWMPin, speed);
}

void decodeState(String s, bool isLeftWheels) {
  bool state1;
  bool state2;

  if (s.equals("F")) {
    state1 = true;
    state2 = false;
  } else if (s.equals("B")) {
    state1 = false;
    state2 = true;
  } else if (s.equals("N")) {
    state1 = false;
    state2 = false;
  } else if (s.equals("BR")) {
    state1 = true;
    state2 = true;
  }

  if (isLeftWheels) {
    leftWheelsState1 = state1;
    leftWheelsState2 = state2;
  } else {
    rightWheelsState1 = state1;
    rightWheelsState2 = state2;
  }
}

void initialisePinModes() {
  pinMode(wheelFrontLeftPin1, OUTPUT);
  pinMode(wheelFrontLeftPin2, OUTPUT);
  pinMode(wheelFrontLeftPinPWM, OUTPUT);

  // pinMode(wheelBackLeftPin1, OUTPUT);
  // pinMode(wheelBackLeftPin2, OUTPUT);
  // pinMode(wheelBackLeftPinPWM, OUTPUT);

  pinMode(wheelFrontRightPin1, OUTPUT);
  pinMode(wheelFrontRightPin2, OUTPUT);
  pinMode(wheelFrontRightPinPWM, OUTPUT);

  // pinMode(wheelBackRightPin1, OUTPUT);
  // pinMode(wheelBackRightPin2, OUTPUT);
  // pinMode(wheelBackRightPinPWM, OUTPUT);
}