float incoming_data = 0; 
char mode = 'd';
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorVal = analogRead(A5);
  int angle = map(sensorVal, 0, 672, 0, 360);
  Serial.print("raw value: "); 
  Serial.println(sensorVal); 
  Serial.print("angle: "); 
  Serial.println(angle); 
  delay(500);  
//  if (Serial.available() > 0){
//    incoming_data = Serial.read(); 
//    Serial.print("angle: "); 
//    Serial.println(incoming_data, DEC); 
//  }
//  delay(1000); 
}
