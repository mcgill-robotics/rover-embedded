#include <Arduino.h>
#include "BasicStepperDriver.h"
#include <Servo.h>
#include <stdint.h>
#include <stdbool.h>
#include <tm4c123gh6pm.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include <wiring_private.h>
#include <Rover_SerialAPI.h>


// Serial defines
#define SERIAL_RX_BUFFER_SIZE 64
#define SERIAL_TX_BUFFER_SIZE 17
#define SCIENCE_SYSTEM_ID '5'

// Stepper and servo properties
#define MICROSTEPS 16
#define ANGLE_PER_STEP 1.8
#define CLAW_MAX_CLOSED_US 1635
#define CLAW_MAX_OPEN_US 2000
#define UPPER_CAROUSEL_RPM 75
#define LOWER_CAROUSEL_RPM 40
#define MOTOR_STEPS 200

// Stepper wiggle constants
#define UPPER_CAROUSEL_WIGGLE_ANGLE 30
#define UPPER_CAROUSEL_WIGGLE_COUNT 10
#define UPPER_CAROUSEL_WIGGLE_DELAY_MS 25
#define LOWER_CAROUSEL_WIGGLE_ANGLE 15
#define LOWER_CAROUSEL_WIGGLE_COUNT 10
#define LOWER_CAROUSEL_WIGGLE_DELAY_MS 50

// Pins
#define UPPER_CAROUSEL_DIR PD_2
#define UPPER_CAROUSEL_STEP PD_3
#define UPPER_CAROUSEL_ENABLE PD_1
#define UPPER_CAROUSEL_NFAULT PE_1
#define CURRENT_SENSE_PIN PD_0
#define CURRENT_NFAULT_PIN PE_2
#define CLAW_SERVO_PIN PB_6
#define SOCOM_DIR PA_2
#define SOCOM_PWM PD_7
#define LOWER_CAROUSEL_DIR PC_4
#define LOWER_CAROUSEL_STEP PC_5
#define LOWER_CAROUSEL_ENABLE PC_6
#define LOWER_CAROUSEL_NFAULT PE_0

volatile bool UPPER_CAROUSEL_FAULT = false;
volatile bool LOWER_CAROUSEL_FAULT = false;
int upper_carousel_forward = 1;
int lower_carousel_forward = 1;

BasicStepperDriver UpperCarousel(MOTOR_STEPS, UPPER_CAROUSEL_DIR, UPPER_CAROUSEL_STEP, UPPER_CAROUSEL_ENABLE);
BasicStepperDriver LowerCarousel(MOTOR_STEPS, LOWER_CAROUSEL_DIR, LOWER_CAROUSEL_STEP, LOWER_CAROUSEL_ENABLE);
Servo Claw;

// Serial buffers and variables
static char rx_buffer[SERIAL_RX_BUFFER_SIZE];
static char tx_buffer[SERIAL_TX_BUFFER_SIZE];
float ctl_floats[4];

// We're also going to need shutdown command, I remembered what it was for...
float gripperState, shutdownState, upper_stepper_increment, lower_stepper_increment, scom_speed,
      upper_carousel_wiggle, lower_carousel_wiggle, gripperTestVar;

void upperCarouselFaultISR();
void lowerCarouselFaultISR();
void setPinAsOpenDrain(char port, int pin, int output);

void setup() {
  // put your setup code here, to run once:
  SerialAPI::init(SCIENCE_SYSTEM_ID, 9600);
  Claw.attach(CLAW_SERVO_PIN);
  UpperCarousel.setEnableActiveState(LOW);
  UpperCarousel.begin(UPPER_CAROUSEL_RPM, MICROSTEPS);
  UpperCarousel.disable();

  LowerCarousel.setEnableActiveState(LOW);
  LowerCarousel.begin(LOWER_CAROUSEL_RPM, MICROSTEPS);
  LowerCarousel.disable();

  pinMode(UPPER_CAROUSEL_NFAULT, INPUT);
  pinMode(LOWER_CAROUSEL_NFAULT, INPUT);

  pinMode(SOCOM_DIR, OUTPUT);
  pinMode(SOCOM_PWM, OUTPUT);

  attachInterrupt(UPPER_CAROUSEL_NFAULT, upperCarouselFaultISR, GPIO_FALLING_EDGE);
  attachInterrupt(LOWER_CAROUSEL_NFAULT, lowerCarouselFaultISR, GPIO_FALLING_EDGE);

  setPinAsOpenDrain('B', 6, 1);
}

void loop() {
  if(UPPER_CAROUSEL_FAULT || LOWER_CAROUSEL_FAULT){
    // What to do in this situation? Any data to send back to main PC?
  }

  // Busy spin and try to see if the fault condition resolves on its own
  while(UPPER_CAROUSEL_FAULT || LOWER_CAROUSEL_FAULT){
    if(digitalRead(UPPER_CAROUSEL_NFAULT) == HIGH && digitalRead(LOWER_CAROUSEL_NFAULT) == HIGH){
      UPPER_CAROUSEL_FAULT = false;
      LOWER_CAROUSEL_FAULT = false;
      break;
    }

  }

  if(SerialAPI::update()){
    memset(rx_buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(rx_buffer,sizeof(rx_buffer));

    memcpy(ctl_floats, rx_buffer+1, 16);  

    gripperState = ctl_floats[0];
    upper_stepper_increment = ctl_floats[1];
    lower_stepper_increment = ctl_floats[2];
    scom_speed = ctl_floats[3];

    (gripperState > 0.0f) ? gripperTestVar = 1.0f : gripperTestVar = 0.0f;

    delay(50);

    tx_buffer[0] = SCIENCE_SYSTEM_ID;
    memcpy(tx_buffer+1, &gripperState, 4);
    memcpy(tx_buffer+5, &upper_stepper_increment, 4);
    memcpy(tx_buffer+9, &lower_stepper_increment, 4);
    memcpy(tx_buffer+13, &scom_speed, 4);

    SerialAPI::send_bytes('0', tx_buffer, 17);
    delay(100);
  }

  if(abs(upper_stepper_increment) > 0.0f){
    UpperCarousel.rotate(upper_stepper_increment);
  }

  if(abs(lower_stepper_increment) > 0.0f){
    LowerCarousel.rotate(lower_stepper_increment);
  }

  if(upper_carousel_wiggle){
    for(int i=0; i<UPPER_CAROUSEL_WIGGLE_COUNT; i++){
      UpperCarousel.rotate(UPPER_CAROUSEL_WIGGLE_ANGLE);
      delay(UPPER_CAROUSEL_WIGGLE_DELAY_MS);
      UpperCarousel.rotate(-2 * UPPER_CAROUSEL_WIGGLE_ANGLE);
      delay(UPPER_CAROUSEL_WIGGLE_DELAY_MS);
      UpperCarousel.rotate(UPPER_CAROUSEL_WIGGLE_ANGLE);
      delay(UPPER_CAROUSEL_WIGGLE_DELAY_MS);
    }
  }

  if(lower_carousel_wiggle){
    for(int i=0; i<LOWER_CAROUSEL_WIGGLE_COUNT; i++){
      LowerCarousel.rotate(LOWER_CAROUSEL_WIGGLE_ANGLE);
      delay(LOWER_CAROUSEL_WIGGLE_DELAY_MS);
      LowerCarousel.rotate(-2 * LOWER_CAROUSEL_WIGGLE_ANGLE);
      delay(LOWER_CAROUSEL_WIGGLE_DELAY_MS);
      UpperCarousel.rotate(LOWER_CAROUSEL_WIGGLE_ANGLE);
      delay(LOWER_CAROUSEL_WIGGLE_DELAY_MS);
    }
  }

  (gripperState > 0.0f) ? Claw.writeMicroseconds(CLAW_MAX_OPEN_US) : Claw.writeMicroseconds(CLAW_MAX_CLOSED_US);

  if(abs(scom_speed > 0.0f)){
    UpperCarousel.disable();
    LowerCarousel.disable();

    // TODO: Put some thought as to what the motor needs to do here

    UpperCarousel.enable();
    LowerCarousel.enable();
  }

  if(shutdownState > 0.0f){
    UpperCarousel.disable();
    LowerCarousel.disable();
  }else{
    UpperCarousel.enable();
    LowerCarousel.enable();
  }

  
}

void upperCarouselFaultISR(){
  UPPER_CAROUSEL_FAULT = true;
}

void lowerCarouselFaultISR(){
  LOWER_CAROUSEL_FAULT = true;
}

void setPinAsOpenDrain(char port, int pin, int output){
  int pin_idx = pin;

  if(0 > pin || pin > 7){Serial.println("e");return;}
  if('A' > port || port > 'F'){Serial.println("e");return;}
  if(!(output == 0 || output == 1)){return;}

  // PD4, PD5, PB0 and PB1 are not 5V tolerant
  if(port == 'D' && pin == 4){return;}
  if(port == 'D' && pin == 5){return;}
  if(port == 'B' && pin == 0){return;}
  if(port == 'B' && pin == 1){return;}

  switch(port){
    case 'A':
      SYSCTL_RCGCGPIO_R |= 0x01;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0001) == 0){};               // ready?
      GPIO_PORTA_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTA_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTA_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTA_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTA_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTA_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTA_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTA_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;

    case 'B':
      SYSCTL_RCGCGPIO_R |= 0x02;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0002) == 0){};               // ready?
      GPIO_PORTB_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTB_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTB_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTB_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTB_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTB_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTB_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTB_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
    
    case 'C':
      SYSCTL_RCGCGPIO_R |= 0x03;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0003) == 0){};               // ready?
      GPIO_PORTC_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTC_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTC_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTC_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTC_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTC_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTC_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTC_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
   
    case 'D':
      SYSCTL_RCGCGPIO_R |= 0x04;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0004) == 0){};               // ready?
      GPIO_PORTD_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTD_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTD_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTD_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTD_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTD_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTD_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTD_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
   
    case 'E':
      SYSCTL_RCGCGPIO_R |= 0x05;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0005) == 0){};               // ready?
      GPIO_PORTE_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTE_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTE_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTE_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTE_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTD_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTE_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTE_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;
    
    case 'F':
      SYSCTL_RCGCGPIO_R |= 0x06;                              // init port clock
      while((SYSCTL_PRGPIO_R & 0x0006) == 0){};               // ready?
      GPIO_PORTF_AMSEL_R &= ~(1 << pin_idx);                  // disable analog because we're going to be using 5v logic and don't want our adc's to die
      GPIO_PORTF_PCTL_R &= ~(output << pin_idx);              // configure as GPIO
      GPIO_PORTF_DIR_R |= (output << pin_idx);                // either input or output
      GPIO_PORTF_AFSEL_R &= ~(1 << pin_idx);                  // normal function

      GPIO_PORTF_ODR_R |= (1 << pin_idx);                     // open drain enable
      GPIO_PORTD_DR4R_R |= (1<<pin_idx);                      // enable higher current drive as needed

      GPIO_PORTF_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTF_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;

    default:
      return;        
  }
  return;
}