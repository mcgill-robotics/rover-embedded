#include <Arduino.h>
#include <RoverArmMotor.h>
#include <tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"

#define HIGHEST_ANGLE_VOLTAGE 3850
#define ZERO_POINT 200

#define WRIST_PWM PB_6
#define WRIST_DIR PA_4
#define WRIST_ENCODER PB_5

#define WP_PWM PC_4
#define WP_DIR PB_3
#define WP_ENCODER PB_4

#define FWD 1
#define REV -1

// PID variables and constants
double Setpoint, Input, Output;
double aggKp=0.025, aggKi=0.019,  aggKd=0, elbaggKp=0.025, elbaggKi=0.019,  elbaggKd=0;
double regKp=0.025, regKi=0.014, regKd=0, elbregKp=0.025, elbregKi=0.014,  elbregKd=0;
RoverArmMotor Wrist(WRIST_PWM, WRIST_ENCODER, CYTRON, 5.0, 141.0, WRIST_DIR);
RoverArmMotor Elbow(WP_PWM, WP_ENCODER, CYTRON, 175.0, 200.0, WP_DIR);
void ConnectionLostISR();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Wrist.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  Elbow.begin(elbaggKp, elbaggKi, elbaggKd, elbregKp, elbregKi, elbregKd);


  Wrist.setMultiplierBool(false, 1);
  Elbow.setMultiplierBool(false, 1);
  Wrist.setGearRatio(48.0/18.0);
  Wrist.setAngleLimits(5.0, 141.0);
  Elbow.setGearRatio(1);
  // Elbow.setAngleLimits(0.0, 360.0);

  // TODO: After the competition, figure out how to
  // get the hardware averager to work!
  // ADCHardwareOversampleConfigure(ADC0_BASE, 64);

  // Set up watchdog timer
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

  // Set watchdog timer to 10 ms
  WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() / 100);
  
  // Attach ISR to watchdog
  WatchdogIntRegister(WATCHDOG0_BASE, &ConnectionLostISR);

  // Enable watchdog
  WatchdogEnable(WATCHDOG0_BASE);

}

void loop() {

  // label:
  //    Serial.print("Elbow encoder: ");Serial.println(Elbow.getCurrentAngle());
  //    Serial.print("Wrist encoder: ");Serial.println(Wrist.getCurrentAngle());
  //    Serial.println();
  //    delay(1000);
  // goto label;

  // Get new setpoint from serial - will be gone when the serial bridge is up
  if(Serial.available()){
    Serial.println("Press A to change setpoint for elbow");
    Serial.println("Press S to see encoder position for elbow");
    Serial.println("Press D to change setpoint for wrist");
    Serial.println("Press F to see the encoder position for wrist");

    if(Serial.available()){

      bool valid_input = false;

      while(!valid_input){
        if(Serial.available()){
        int input_character = Serial.read();

        switch(input_character){
          case 'A':
            Serial.print("Enter new setpoint for elbow: ");
            while(true){
              if(Serial.available()){
                double set = (double) Serial.parseFloat();
                Elbow.newSetpoint(set); 
                delay(1000);
                break;
              }


            }
            Serial.println();
            Serial.print("New setpoint for elbow: ");Serial.println(Elbow.getSetpoint());
            valid_input = true;
            break;
          
          case 'S':
            Serial.print("Current encoder position for elbow: ");
            Serial.println(Elbow.getCurrentAngle());
            delay(200);
            valid_input = true;
            break;
          
          case 'D':
            Serial.print("Enter new setpoint for wrist: ");
            while(true){
              if(Serial.available()){
                double set = (double) Serial.parseFloat();
                Wrist.newSetpoint(set); 
                delay(1000);
                break;
              }


            }
            Serial.println();
            Serial.print("New setpoint for wrist: ");Serial.println(Wrist.getSetpoint());
            valid_input = true;
            break;

          case 'F':
            Serial.print("Current encoder position for wrist: ");
            Serial.println(Wrist.getCurrentAngle());
            delay(200);
            valid_input = true;
            break;
          

        }
     
       }
      } 
   }
  }

  // Reset timer here so .tick() can run


  Wrist.tick();
  Elbow.tick();

}


// If the connection is lost and .tick() is blocked from running for more than 10ms, 
// disable motor until the error is recovered from
void ConnectionLostISR(){
  WatchdogIntClear(WATCHDOG0_BASE);
  analogWrite(WRIST_PWM, 0);
  analogWrite(WP_PWM, 0);
}