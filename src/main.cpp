#include <Arduino.h>

#define pwm 3
#define dir 2

#define SENSE_PIN A0
#define ADC_RESOLUTION 10
#define FWD 1
#define REV 0
#define STOP -1

// ADC_MAX = 2^ADC_RESOLUTION - 1
#define ADC_MAX 1023
int adc_supply_voltage = 4.6;

int adc_voltage = 0;
float voltage_change, mapped_voltage, voltage_ratio, current = 0;
float sensitivity = 0.068;
float read_current = 0.0; 
int pwm_value = 0; 
float offset_voltage = 0.0;
float adc_total = 0.0;
float average_adc =0.0;

//Function headers
void driveMotor(int direction, int speed);
float readCurrent();

void setup() {
  // put your setup code here, to run once:
  pinMode(SENSE_PIN, INPUT);
  analogReference(DEFAULT);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  Serial.begin(9600);
  analogWrite(pwm, 0); //turn motor rotation off on initial startup.
  set_offset();
}

void loop() {
  // put your main code here, to run repeatedly:
  driveMotor(REV, 10);
  read_current = readCurrent(); 
  Serial.print("Current: ");
  Serial.println(read_current);
  delay (750); 
}

void set_offset(){
  for (int i = 0; i < 10; i++){
    adc_voltage = analogRead(SENSE_PIN); //read analog read value
    adc_total += adc_voltage;
    delay(50);
  }
  average_adc = adc_total/10; 
  voltage_ratio = average_adc/1024;
  offset_voltage = voltage_ratio * 5;
}

/*
Function to drive DC Motor.
Direction = 1 --> forward; Direction = 0 --> Reverse; Direction = -1 -->Stopped
Speed --> Motor speed between 0 - 100 (0 = min speed; 100 = max speed)
*/
void driveMotor(int direction, int speed){
  if (direction == -1){
    Serial.print("Motor Stopped");
    pwm_value = 0;
  }
  else if(direction == 1){
    Serial.println("Driving Motor Forward");
    pwm_value = map(speed, 0, 100, 0, 255);
  }
  else{
    Serial.println("Driving Motor Backward");
     pwm_value = map(speed, 0, 100, 0, 255);
  }
  digitalWrite(dir, direction);
  analogWrite(pwm, pwm_value);
}

/*
Function to read current sensor.
*/
float readCurrent(){
  for (int i = 0; i < 10; i++){
    adc_voltage = analogRead(SENSE_PIN); //read analog read value
    adc_total += adc_voltage;
    delay(100); 
  }
  Serial.println(adc_total);
  average_adc = adc_total/10; 
  voltage_ratio = average_adc/1024; //turn the analog value into a fraction based on the ADC resolution.
  Serial.println(voltage_ratio);
  mapped_voltage = voltage_ratio * 5; //voltage between 0 - 5;
  Serial.println(mapped_voltage);
  /*
  This is based on the current equation that was given on the pololu website. I hope this shit works. 
  current = 73.3*(mapped_voltage/ADC_SUPPLY_VOLTAGE) - 36.7; 
  */
  //voltage_change = mapped_voltage - (float)adc_supply_voltage/2; //get change in voltage from current monitor offset value
  voltage_change = mapped_voltage - offset_voltage;
  Serial.println(voltage_change);
  current = voltage_change/sensitivity; //calculate the current. adc out changes by 68mV/A. 
  return current; 
}
