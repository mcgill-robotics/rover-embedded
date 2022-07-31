#include <Arduino.h>
#include <peripheral.h>
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"
#include <stdio.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include <Rover_SerialAPI.h>
#include <bitset>

#define SERIAL_RX_BUFFER_SIZE 64

#define ARM_ESC__PIN PB_5
#define DC_CNTR_PIN PA_3 
#define SCI_CNTR_PIN PB_6

peripheral Arm(ARM_ESC__PIN);
peripheral DC(DC_CNTR_PIN);
peripheral SCI(SCI_CNTR_PIN);

//peripheral order[] = {Arm, DC, SCI};
void ConnectionLostISR(); 

//static char* systems[3] = {"Arm System", "DC Motors", "Science System"};

void setup() {
  // put your setup code here, to run once:
   //Serial.begin(9600); 
  SerialAPI::init('4', 9600);

  Arm.begin(LOW); 
  DC.begin(LOW); 
  SCI.begin(LOW); 
  

  // SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
  // WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet()/100);
  // WatchdogIntRegister(WATCHDOG0_BASE, &ConnectionLostISR);
  // WatchdogEnable(WATCHDOG0_BASE);
}

static char rx_buffer[64]; 
static char tx_buffer[13]; 
float ctl_floats[4];
float arm, dc, science; 

void loop() {   
    
  // put your main code here, to run repeatedly:
  if(SerialAPI::update()){
    memset(rx_buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(rx_buffer, sizeof(rx_buffer));
    memcpy(ctl_floats, rx_buffer + 1, 16);
    arm = ctl_floats[0]; 
    dc = ctl_floats[1]; 
    science = ctl_floats[2];

    (arm > 0.0f) ? Arm.turnOn() : Arm.turnOff(); 
    (dc > 0.0f)? DC.turnOn() : DC.turnOff(); 
    (science > 0.0f)? SCI.turnOn() : SCI.turnOff();

    tx_buffer[0] = '4';
    memcpy(tx_buffer + 1, &arm, 4);
    memcpy(tx_buffer + 5, &dc, 4); 
    memcpy(tx_buffer + 9, &science, 4);
    SerialAPI::send_bytes('0', tx_buffer, 13);
  
  }
  // if(Serial.available()){
  //   Serial.println("Press A to change the system states");
  //   Serial.println("Press S to view the system states");
  //   if(Serial.available()){
  //     bool valid_input = false;           
  //     while(!valid_input){
  //       if(Serial.available()){
  //         int input_character = (Serial.read());  
  //         switch(input_character){
  //           case 'A':
  //             Serial.println("Enter new system states (three bit number consisting of 1s (on) and 0s (off): ");
  //             while(true){
  //               if(Serial.available()){
  //                 int configuration = Serial.parseInt(); 
  //                 int config[3] = {}; 
  //                 int digit = -1;
  //                 for (int i = 0; i < (int)(sizeof(config) / sizeof(config[0])); i++){
  //                   digit = configuration % 10; 
  //                   if(digit < 2 && digit >= 0){
  //                     Serial.println((String) digit);
  //                     digit == 0 ? order[i].turnOff() : order[i].turnOn();
  //                     // if (order[i].getState() == LOW){
  //                     //   Serial.print(systems[i]);
  //                     //   Serial.println(" is off.");
  //                     // }
  //                     // else{
  //                     //   Serial.print(systems[i]);
  //                     //   Serial.println(" is on.");
  //                     // }
  //                     configuration /= 10; 
  //                   }
  //                   else{ 
  //                     Serial.println("Faulty System Config Attempted");
  //                     break; 
  //                   }
  //                 }
  //                 // for (int i = 0; i < (int)(sizeof(config) / sizeof(config[0])); i++){
  //                 //   if(config[i] == 0 && order[i].getState() != LOW){
  //                 //     order[i].turnOff(); 
  //                 //   }
  //                 //   else if (config[i] == 1 && order[i].getState() != HIGH){
  //                 //     order[i].turnOn();
  //                 //   }
  //                 // }
  //                 Serial.println("Complete");
  //                 break;
  //               }
  //             }  
  //             break; 
  //           case 'S': 
  //             Serial.println("Current System COnfigrations: ");
  //             for (int i = 0; i < (int)(sizeof(order)/ sizeof(order[0])); i++){
  //               if(order[i].getState() == LOW){
  //                 Serial.print("System: ");
  //                 Serial.print(systems[i]);
  //                 Serial.println(" is OFF");
  //               }
  //               else{
  //                 Serial.print("System: ");
  //                 Serial.print(systems[i]);
  //                 Serial.println(" is ON");
  //               }
  //             }  
  //             break;     
  //         } 
  //       }
  //     }
  //   }
  // }
}

//this ISR will be used to turn off the arm and science module
//if connection is lost between the main computer and the power board.
void ConnectionLostISR(){
  
}