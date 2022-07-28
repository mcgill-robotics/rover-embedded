/*
 * ---[STEPPER UNIT TEST PINOUT]---
 * 
 * PE1 - notFAULT1: High when Stepper #1 is not in a fault condition
 * PE0 - notFAULT2: High when Stepper #2 is not in a fault condition
 * 
 * PD2 - STEPPER1_DIR: Stepper #1 will change direction on the rising edge of this pin.
 * PD3 - STEPPER1_STEP: Stepper #1 will advance one step on the rising edge of this pin.
 * PD1 - STEPPER1_ENABLE: Stepper #1 is enabled when this pin is low.
 * 
 * PF0 - CLAW_SERVO: PWM pin to control the claw
 * 
 * PC4 - LOWER_CAROUSEL_DIR: Stepper #2 will change direction on the rising edge of this pin.
 * PC5 - LOWER_CAROUSEL_STEP: Stepper #2 will advance one step on the rising edge of this pin.
 * PC6 - LOWER_CAROUSEL_ENABLE: Stepper #2 is enabled when this pin is low.
 * 
 * Either these or whichever pinout is more convenient
 * 
  */
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

#define MOTOR_STEPS 200

// Upper stepper faster than lower. Upper stepper is 75 RPM.
#define MICROSTEPS 16
#define ANGLE_PER_STEP 1.8
#define CLAW_MAX_CLOSED_US 1635
#define CLAW_MAX_OPEN_US 2000
#define SERIAL_RX_BUFFER_SIZE 64

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

BasicStepperDriver UpperCarousel(MOTOR_STEPS, UPPER_CAROUSEL_DIR, UPPER_CAROUSEL_STEP, UPPER_CAROUSEL_ENABLE);
BasicStepperDriver LowerCarousel(MOTOR_STEPS, LOWER_CAROUSEL_DIR, LOWER_CAROUSEL_STEP, LOWER_CAROUSEL_ENABLE);
Servo Claw;

int upper_carousel_rpm = 75;
int lower_carousel_rpm = 40;

char buffer[SERIAL_RX_BUFFER_SIZE];
float received_data[4];
char temp_chr;
bool wantClawState;
bool wantShutdown;
bool clawState;
float scom_speed = 0;
float scom_desired_speed;
int scom_real_speed = 0;
int scom_desired_real_speed;
float step1_inc_angle;
float step2_inc_angle;
#define SPEED_DEADBAND 1
#define MOTOR_ACCEL_DELAY_MS 10

// Function signatures, ignore pls
void upperCarouselFaultISR();
void lowerCarouselFaultISR();
void setPinAsOpenDrain(char port, int pin, int output);


void setup() {
  SerialAPI::init(5, 9600);

  // Set PB6 up for open-drain operation and wait while the
  // register changes are committed to IO pins
  Claw.attach(CLAW_SERVO_PIN);
  Claw.writeMicroseconds(1635);
  clawState = true;
  setPinAsOpenDrain('B', 6, 1);

  UpperCarousel.setEnableActiveState(LOW);
  LowerCarousel.setEnableActiveState(LOW);
  UpperCarousel.begin(upper_carousel_rpm, MICROSTEPS);
  LowerCarousel.begin(lower_carousel_rpm, MICROSTEPS);

  // Energize coils, uncomment when you connect the motor
  UpperCarousel.disable();
  LowerCarousel.disable();

  // Input pullups for interrupt pins are very important!
  pinMode(UPPER_CAROUSEL_NFAULT, INPUT);
  pinMode(LOWER_CAROUSEL_NFAULT, INPUT);

  pinMode(SOCOM_DIR, OUTPUT);
  pinMode(SOCOM_PWM, OUTPUT);

  analogReadResolution(12);

  
  // Attach interrupts to falling edges of the notFAULT pins to handle fault conditions
  attachInterrupt(UPPER_CAROUSEL_NFAULT, upperCarouselFaultISR, GPIO_FALLING_EDGE);
  attachInterrupt(LOWER_CAROUSEL_NFAULT, lowerCarouselFaultISR, GPIO_FALLING_EDGE);

}

void loop() {

  if(SerialAPI::update()){
    memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));
    memcpy(received_data, buffer+1, 16);
    memcpy(&temp_chr, &(received_data[0]), 1);
    wantClawState = (bool)(temp_chr & 1<<2);
    wantShutdown = (bool)(temp_chr & 1<<5);
    scom_desired_speed = received_data[1];
    step1_inc_angle = received_data[2];
    step2_inc_angle = received_data[3];

    if(!wantShutdown){
      UPPER_CAROUSEL_FAULT = !digitalRead(UPPER_CAROUSEL_NFAULT);
      LOWER_CAROUSEL_FAULT = !digitalRead(LOWER_CAROUSEL_NFAULT);
      if(!UPPER_CAROUSEL_FAULT) UpperCarousel.rotate(step1_inc_angle);
      if(!LOWER_CAROUSEL_FAULT) LowerCarousel.rotate(step2_inc_angle);

      if(wantClawState == true && clawState == false){
        Claw.writeMicroseconds(1635);
        clawState = true;
      }
      else if (wantClawState == false && clawState == true)
      {
        Claw.writeMicroseconds(2000);
        clawState = false;
      }

      if(abs(scom_desired_speed-scom_speed) >= SPEED_DEADBAND){
        scom_desired_real_speed = (abs(scom_desired_speed)/100.0) * 255;
        if((scom_speed < 0 && scom_desired_speed >= 0)||(scom_speed >= 0 && scom_desired_speed < 0)){
          for(int i = scom_real_speed; i > -1; i--){
            analogWrite(SOCOM_PWM, i);
            scom_real_speed = i;
            delay(MOTOR_ACCEL_DELAY_MS);
          }
          uint8_t pinSend = (scom_speed > 0) ? HIGH : LOW;
          digitalWrite(SOCOM_DIR, pinSend);
        }
        if(scom_desired_real_speed > scom_real_speed){
          for(int i = scom_real_speed; i<= scom_desired_real_speed; i++){
            analogWrite(SOCOM_PWM, i);
            scom_real_speed = i;
            delay(MOTOR_ACCEL_DELAY_MS);
          }
        }else{
          for(int i = scom_real_speed; i>= scom_desired_real_speed; i--){
            analogWrite(SOCOM_PWM, i);
            scom_real_speed = i;
            delay(MOTOR_ACCEL_DELAY_MS);
          }
        }
        scom_speed = (scom_real_speed/255.0) * 100.0;
      }
    }else{
      for(int i = scom_real_speed; i > -1; i--){
            analogWrite(SOCOM_PWM, i);
            scom_real_speed = i;
            delay(MOTOR_ACCEL_DELAY_MS);
          }
      scom_speed = 0;
    }
    char send_buf = 0;
    send_buf |= ((char) clawState) << 2;
    send_buf |= ((char) UPPER_CAROUSEL_FAULT) << 4;
    send_buf |= ((char) LOWER_CAROUSEL_FAULT) << 5;
    SerialAPI::send_bytes('2', &send_buf, 1);
  }
}

// --[PRINT UTILITY FUNCTIONS]--

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
