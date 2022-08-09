#include <Arduino.h>
#include <RoverArmMotor.h>
#include <tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"

#define HIGHEST_ANGLE_VOLTAGE 3850
#define ZERO_POINT 200

// #define CLAW_PWM PC_5
// #define CLAW_DIR PC_6

#define WR_PWM PB_6
#define WR_DIR PA_4
#define WR_ENCODER PB_5

#define WP_PWM PC_4
#define WP_DIR PB_3
#define WP_ENCODER PB_4

#define SHLDR_PWM PA_7
#define SHLDR_ENCODER PE_3
#define SHLDR_BRAKE PD_6

#define EBLW_PWM PF_3
#define EBLW_ENCODER PE_4
#define EBLW_BRAKE PD_7

#define WAIST_PWM PB_7
#define WAIST_ENCODER PB_1

#define FWD 1
#define REV -1

#define SERIAL_RX_BUFFER_SIZE 64
typedef struct{
  float b[SERIAL_RX_BUFFER_SIZE/sizeof(float)];
  size_t count; 
}FloatBuffer;

FloatBuffer fbuf; 
static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuffer[SERIAL_RX_BUFFER_SIZE];

float positions[5];
char buf[21]; 
//TODO: add array to contain limit switch states 
volatile float wr_position; 
volatile float wp_position; 
volatile float shldr_position; 
volatile float elbw_position; 
volatile float waist_position;

// PID variables and constants
double Setpoint, Input, Output;
double aggKp=0.025, aggKi=0.019,  aggKd=0, elbaggKp=0.025, elbaggKi=0.019,  elbaggKd=0;
double regKp=0.025, regKi=0.014, regKd=0, elbregKp=0.025, elbregKi=0.014,  elbregKd=0;
RoverArmMotor Wrist_Roll(WR_PWM, WR_ENCODER, CYTRON, 5.0, 141.0, WR_DIR, 0);
RoverArmMotor Wrist_Pitch(WP_PWM, WP_ENCODER, CYTRON, 175.0, 200.0, WP_DIR, 0);
RoverArmMotor Shoulder(SHLDR_PWM, SHLDR_ENCODER, BLUE_ROBOTICS, 5.0, 141.0, 0, SHLDR_BRAKE);
RoverArmMotor Elbow(EBLW_PWM, EBLW_ENCODER, BLUE_ROBOTICS, 5.0, 141.0, 0, EBLW_BRAKE);
RoverArmMotor Waist(WAIST_PWM, WAIST_ENCODER, BLUE_ROBOTICS, 5.0, 141.0, 0, 0);
//RoverArmMotor Claw(CLAW_PWM, 0, BLUE_ROBOTICS, 5.0, 141.0, CLAW_DIR, 0);

void ConnectionLostISR();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  Wrist_Roll.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  Wrist_Pitch.begin(elbaggKp, elbaggKi, elbaggKd, elbregKp, elbregKi, elbregKd);
  Shoulder.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  Elbow.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  Waist.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);
  //Claw.begin(aggKp, aggKi, aggKd, regKp, regKi, regKd);

  // Wrist_Roll.setMultiplierBool(true, 1);
  // Wrist_Pitch.setMultiplierBool(false, 1);
  Wrist_Roll.setMultiplierBool(true, 48.0/18.0);
  Wrist_Roll.setAngleLimits(5.0, 141.0);
  // Wrist_Pitch.setGearRatio(1);
  // Wrist_Pitch.setAngleLimits(0.0, 360.0);


  /*TODO: Changee the angle limits and verify the gear reduction*/
  //based on what ben said the gear reduction was 
  Waist.setMultiplierBool(true, 60.0/150.0);
  Waist.setAngleLimits(0, 180);


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
  //    Serial.print( Wrist_Pitch encoder: ");Serial.println Wrist_Pitch.getCurrentAngle());
  //    Serial.print( Wrist_Roll encoder: ");Serial.println Wrist_Roll.getCurrentAngle());
  //    Serial.println();
  //    delay(1000);
  // goto label;

  // Get new setpoint from serial - will be gone when the serial bridge is up
  if(Serial.available()){
    Serial.println("Press A to change setpoint for wrist pitch");
    Serial.println("Press S to see encoder position for wrist pitch");
    Serial.println("Press D to change setpoint for wrist roll");
    Serial.println("Press F to see the encoder position for wrist roll");
    Serial.println("Press G to change setpoint for shoulder");
    Serial.println("Press H to see the encoder position for shoulder");
    Serial.println("Press J to change setpoint for elbow");
    Serial.println("Press K to see the encoder position for elbow");
    Serial.println("Press W to change setpoint for waist");
    Serial.println("Press E to see the encoder position for waist");

    if(Serial.available()){

      bool valid_input = false;

      while(!valid_input){
        if(Serial.available()){
          int input_character = Serial.read();

          switch(input_character){
            case 'A':
              Serial.print("Enter new setpoint for wrist pitch: ");
              while(true){
                if(Serial.available()){
                  double set = (double) Serial.parseFloat();
                Wrist_Pitch.newSetpoint(set); 
                  delay(1000);
                  break;
                }


              }
              Serial.println();
              Serial.print("New setpoint for wrist pitch: ");Serial.println(Wrist_Pitch.getSetpoint());
              valid_input = true;
              break;
            
            case 'S':
              Serial.print("Current encoder position for wrist pitch: ");
              Serial.println(Wrist_Pitch.getCurrentAngle());
              delay(200);
              valid_input = true;
              break;
            
            case 'D':
              Serial.print("Enter new setpoint for wrist roll: ");
              while(true){
                if(Serial.available()){
                  double set = (double) Serial.parseFloat();
                Wrist_Roll.newSetpoint(set); 
                  delay(1000);
                  break;
                }


              }
              Serial.println();
              Serial.print("New setpoint for wrist roll: ");Serial.println(Wrist_Roll.getSetpoint());
              valid_input = true;
              break;

            case 'F':
              Serial.print("Current encoder position for wrist roll: ");
              Serial.println(Wrist_Roll.getCurrentAngle());
              delay(200);
              valid_input = true;
              break;
            
            case 'G':
              Serial.print("Enter new setpoint for shoulder: ");
              while(true){
                if(Serial.available()){
                  double set = (double) Serial.parseFloat();
                  Shoulder.newSetpoint(set); 
                  delay(1000);
                  break;
                }


              }
              Serial.println();
              Serial.print("New setpoint for shoulder: ");Serial.println(Shoulder.getSetpoint());
              valid_input = true;
              break;

            case 'H':
              Serial.print("Current encoder position for wrist roll: ");
              Serial.println(Shoulder.getCurrentAngle());
              delay(200);
              valid_input = true;
              break;
            
            case 'J':
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

            case 'K':
              Serial.print("Current encoder position for elbow: ");
              Serial.println(Elbow.getCurrentAngle());
              delay(200);
              valid_input = true;
              break;

            case 'W':
              Serial.print("Enter new setpoint for waist: ");
              while(true){
                if(Serial.available()){
                  double set = (double) Serial.parseFloat();
                  Waist.newSetpoint(set); 
                  delay(1000);
                  break;
                }


              }
              Serial.println();
              Serial.print("New setpoint for waist: ");Serial.println(Waist.getSetpoint());
              valid_input = true;
              break;

            case 'E':
              Serial.print("Current encoder position for waist: ");
              Serial.println(Waist.getCurrentAngle());
              delay(200);
              valid_input = true;
              break;
          }
        }
      } 
    }
  }

  // Reset timer here so .tick() can run

  if(SerialAPI::update()){
    memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(buffer, sizeof(buffer)); 

    //is it only possible to send 4 float? or can we also send 5?
    memcpy(positions, buffer+1, 20);

    wr_position = positions[0];
    wp_position = positions[1]; 
    shldr_position = positions[2];
    elbw_position = positions[3];
    waist_position = positions[4]; 

    delay(50); 

    buf[0] = '1';
    memcpy(buf+1, positions, 20); 
    SerialAPI::send_bytes('0', buf, 21);
    delay(100); 

    //TODO: add code to send limit switch info
  }
  Wrist_Roll.newSetpoint((double) wr_position);
  Wrist_Roll.tick();

  Wrist_Pitch.newSetpoint((double) wp_position);
  Wrist_Pitch.tick();

  Shoulder.newSetpoint((double) shldr_position);
  Shoulder.disengageBrake(); 
  Shoulder.tick();

  Elbow.newSetpoint((double) elbw_position);
  Elbow.disengageBrake();
  Elbow.tick();

  Waist.newSetpoint((double) waist_position);  
  Waist.tick();
}


// If the connection is lost and .tick() is blocked from running for more than 10ms, 
// disable motor until the error is recovered from
void ConnectionLostISR(){
  WatchdogIntClear(WATCHDOG0_BASE);
  analogWrite(WR_PWM, 0);
  analogWrite(WP_PWM, 0);
}