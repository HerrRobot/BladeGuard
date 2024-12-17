void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // while(!Serial){
  //   ;
  // }
  Serial.setTimeout(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String message = Serial.readString();

    if(message.equals("YES")){
      Serial.println("WE GOOD");
    } else if (message.equals("NO")){
      Serial.println("WE BAD");
    } else {
      Serial.println("WE NEUTRAL");
    }
  }
}
