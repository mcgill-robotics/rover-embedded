#include <Arduino.h>
#include <RoverArmMotor.h>

#define HIGHEST_ANGLE_VOLTAGE 3850
#define ZERO_POINT 200

#define WRIST_PWM PB_6
#define WRIST_DIR PA_4
#define WRIST_ENCODER PB_5

#define FWD 1
#define REV -1

// PID variables and constants
double Setpoint, Input, Output;
double aggKp=0.05, aggKi=0.021,  aggKd=0;
double regKp=0.05, regKi=0.016, regKd=0;
RoverArmMotor Wrist(WRIST_PWM, WRIST_ENCODER, CYTRON, 0, 360, WRIST_DIR);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Wrist.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);

  Wrist.setMultiplierBool(true, 2);

  // TODO: After the competition, figure out how to
  // get the hardware averager to work!
  // ADCHardwareOversampleConfigure(ADC0_BASE, 64);

}

void loop() {

  // Get new setpoint from serial - will be gone when the serial bridge is up
  if(Serial.available()){
    Serial.println("Press A to change setpoint");
    Serial.println("Press S to see encoder position");

    if(Serial.available()){

      bool valid_input = false;

      while(!valid_input){
        if(Serial.available()){
        int input_character = Serial.read();

        switch(input_character){
          case 'A':
            Serial.print("Enter new setpoint: ");
            while(true){
              if(Serial.available()){
                double set = (double) Serial.parseFloat();
                Wrist.newSetpoint(set); 
                delay(5000);
                break;
              }


            }
            Serial.println();
            Serial.print("New setpoint: ");Serial.println(Wrist.getSetpoint());
            valid_input = true;
            break;
          
          case 'S':
            Serial.print("Current encoder position: ");
            Serial.println(Wrist.getCurrentAngle());
            delay(200);
            valid_input = true;
            break;

        }
     
       }
      } 
   }
  }

  Serial.print("Encoder position: ");
  Serial.println(Wrist.getCurrentAngle());
  
  Wrist.tick();

}
