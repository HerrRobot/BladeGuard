const int motorA1Pin = 12;
const int motorA2Pin = 13;
const int motorAPWM = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(motorA1Pin, OUTPUT);
  pinMode(motorA2Pin, OUTPUT);
  pinMode(motorAPWM, OUTPUT);

  delay(2000);

  digitalWrite(motorA1Pin, HIGH);
  digitalWrite(motorA2Pin, LOW);

  delay(2000);

  analogWrite(motorAPWM, 32);

  delay(2000);

  analogWrite(motorAPWM, 64);

  delay(2000);

  analogWrite(motorAPWM, 127);

  delay(2000);

  analogWrite(motorAPWM, 0);

  delay(2000);

  digitalWrite(motorA1Pin, LOW);
  digitalWrite(motorA2Pin, HIGH);

  delay(2000);

  analogWrite(motorAPWM, 32);

  delay(2000);

  analogWrite(motorAPWM, 64);

  delay(2000);

  analogWrite(motorAPWM, 127);

  delay(2000);

  analogWrite(motorAPWM, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
