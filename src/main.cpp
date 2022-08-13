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
#include <driverlib/timer.h>
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/watchdog.h"


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
#define SCOM_DIR PA_2
#define SCOM_PWM PD_7
#define LOWER_CAROUSEL_DIR PC_4
#define LOWER_CAROUSEL_STEP PC_5
#define LOWER_CAROUSEL_ENABLE PC_6
#define LOWER_CAROUSEL_NFAULT PE_0

// SCOM will accelerate, then keep going for this many seconds,
// then decelerate to a rest.
#define CONSTANT_SPEED_DURATION 2

// Defines the resolution of acceleration/deceleration. New PWM
// values are computed every 1 / CLOCK_DIV seconds.
#define CLOCK_DIV 40

// Limit motor power output by (100 * (MAX_PWM / 255) )%
#define MAX_PWM 100

#define PWM_INCREMENT 3

enum ActuationStates{ACCEL_UP, DECEL_UP, CONST_UP, ACCEL_DOWN, DECEL_DOWN, CONST_DOWN, STOPPED};
volatile int actuation_state = STOPPED;
volatile int current_pwm_value = 0;
volatile int coast_ticks = 0;

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
      last_scom_speed, upper_carousel_wiggle, lower_carousel_wiggle, gripperTestVar;

void upperCarouselFaultISR();
void lowerCarouselFaultISR();
void AccelISR();
void setPinAsOpenDrain(char port, int pin, int output);

volatile bool connection_lost = false;

void setup() {
  // put your setup code here, to run once: 

  // Enable timer for SCOM accel control
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)){}
  TimerConfigure(TIMER4_BASE, TIMER_CFG_A_PERIODIC);
  TimerIntRegister(TIMER4_BASE, TIMER_A, AccelISR);
  TimerLoadSet(TIMER4_BASE, TIMER_A, SysCtlClockGet() / CLOCK_DIV);
  TimerEnable(TIMER4_BASE, TIMER_A);
  IntEnable(INT_TIMER4A);
  TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

  Claw.attach(CLAW_SERVO_PIN);

  UpperCarousel.setEnableActiveState(LOW);
  UpperCarousel.begin(UPPER_CAROUSEL_RPM, MICROSTEPS);
  UpperCarousel.disable();
  LowerCarousel.setEnableActiveState(LOW);
  LowerCarousel.begin(LOWER_CAROUSEL_RPM, MICROSTEPS);
  LowerCarousel.disable();

  pinMode(UPPER_CAROUSEL_NFAULT, INPUT);
  pinMode(LOWER_CAROUSEL_NFAULT, INPUT);

  pinMode(SCOM_DIR, OUTPUT);
  pinMode(SCOM_PWM, OUTPUT);

  // Watchdog timer for connection loss, set it for 2 seconds
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
  WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 2);
  WatchdogIntRegister(WATCHDOG0_BASE, &ConnectionLostISR);
  WatchdogEnable(WATCHDOG0_BASE);

  // Watchdog timer to reset MCU if serial connection isn't recovered in 10 seconds.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);

  // Configure this timer to reset the system on its second interrupt (10 seconds)
  WatchdogReloadSet(WATCHDOG1_BASE, SysCtlClockGet() * 5);
  WatchdogResetEnable(WATCHDOG1_BASE);
  WatchdogEnable(WATCHDOG1_BASE);


  // attachInterrupt(UPPER_CAROUSEL_NFAULT, upperCarouselFaultISR, GPIO_FALLING_EDGE);
  // attachInterrupt(LOWER_CAROUSEL_NFAULT, lowerCarouselFaultISR, GPIO_FALLING_EDGE);

  setPinAsOpenDrain('B', 6, 1);
}

void loop() {

  if(UPPER_CAROUSEL_FAULT || LOWER_CAROUSEL_FAULT){
    // What to do in this situation? Any data to send back to main PC?
  }

  // // Busy spin and try to see if the fault condition resolves on its own
  // while(UPPER_CAROUSEL_FAULT || LOWER_CAROUSEL_FAULT){
  //   if(digitalRead(UPPER_CAROUSEL_NFAULT) == HIGH && digitalRead(LOWER_CAROUSEL_NFAULT) == HIGH){
  //     UPPER_CAROUSEL_FAULT = false;
  //     LOWER_CAROUSEL_FAULT = false;
  //     break;
  //   }

  // }

  // Make sure the motor transitions to the deceleration state if connection is lost,
  // after the deceleration state it will automatically go to the stopped state
  if(connection_lost){
    if(actuation_state == ACCEL_UP || actuation_state == CONST_UP) {
        actuation_state = DECEL_UP;
    }else if(actuation_state == ACCEL_DOWN || actuation_state == CONST_DOWN){
        actuation_state = DECEL_DOWN;
    }
  }

  if(SerialAPI::update()){

    // Feed watchdog as soon as serial communication is established
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 2);
    WatchdogReloadSet(WATCHDOG1_BASE, SysCtlClockGet() * 5);
    connection_lost = false;

    memset(rx_buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(rx_buffer,sizeof(rx_buffer));

    memcpy(ctl_floats, rx_buffer+1, 16);  

    gripperState = ctl_floats[0];
    upper_stepper_increment = ctl_floats[1];
    lower_stepper_increment = ctl_floats[2];
    scom_speed = ctl_floats[3];

    (gripperState > 0.0f) ? gripperTestVar = 1.0f : gripperTestVar = 0.0f;

    delay(50);

    // Control logic for SCOM is here because it should be able to turn off steppers
    // before anything else gets to them

    bool going_up = (actuation_state == ACCEL_UP || actuation_state == CONST_UP);
    bool going_down = (actuation_state == ACCEL_DOWN || actuation_state == CONST_DOWN);

    // Only change actuation state if software sends a different command
    if(scom_speed != last_scom_speed){

      if(scom_speed == 1.0f){

        // Up
        if(going_down) {
          actuation_state = DECEL_DOWN;
        }else if(actuation_state == STOPPED){
          actuation_state = ACCEL_UP;
        }

      }else if(scom_speed == 0.0f){

        // Stop
        if(going_up) {
          actuation_state = DECEL_UP;
        }else if(going_down){
          actuation_state = DECEL_DOWN;
        }

      }else if(scom_speed == -1.0f){
        
        // Down
        if(going_up) {
          actuation_state = DECEL_UP;
        }else if(actuation_state == STOPPED){
          actuation_state = ACCEL_DOWN;
        }

      }else{

        // Default to a stop if data is garbled
        if(going_up) {
          actuation_state = DECEL_UP;
        }else if(going_down){
          actuation_state = DECEL_DOWN;
        }

      }
  }
    scom_speed = last_scom_speed;
    
    // Steppers shouldn't run at the same time as SCOM, if they do,
    // EMF badly fucks things up
    if(actuation_state != STOPPED){
      UpperCarousel.disable();
      LowerCarousel.disable();
    }


    // Sent for board enumeration, isn't actually used by software
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

void AccelISR(){
  TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
  
  switch(actuation_state)
  {
    case ACCEL_UP:
      current_pwm_value += PWM_INCREMENT;
      if(current_pwm_value >= MAX_PWM){
        current_pwm_value = MAX_PWM;
        actuation_state = CONST_UP;
      }
      digitalWrite(SCOM_DIR, LOW);
      delayMicroseconds(500);
      analogWrite(SCOM_PWM, current_pwm_value);
      break;
    

    case CONST_UP:
      coast_ticks++;
      if(coast_ticks == (CONSTANT_SPEED_DURATION * CLOCK_DIV)){
          actuation_state = DECEL_UP;
          coast_ticks = 0;
      }
      break;
    
    case DECEL_UP:
      current_pwm_value -= PWM_INCREMENT;
      if(current_pwm_value <= 0){
        current_pwm_value = 0;
        actuation_state = STOPPED;
      }
      digitalWrite(SCOM_DIR, LOW);
      delayMicroseconds(500);
      analogWrite(SCOM_PWM, current_pwm_value);
      break;
    
    case ACCEL_DOWN:
      current_pwm_value += PWM_INCREMENT;
      if(current_pwm_value >= MAX_PWM){
        current_pwm_value = MAX_PWM;
        actuation_state = CONST_DOWN;
      }
      digitalWrite(SCOM_DIR, HIGH);
      delayMicroseconds(500);
      analogWrite(SCOM_PWM, current_pwm_value);
      break;
    

    case CONST_DOWN:
      coast_ticks++;
      if(coast_ticks == (CONSTANT_SPEED_DURATION * CLOCK_DIV)){
          actuation_state = DECEL_DOWN;
          coast_ticks = 0;
      }
      break;
    
    case DECEL_DOWN:
      current_pwm_value -= PWM_INCREMENT;
      if(current_pwm_value <= 0){
        current_pwm_value = 0;
        actuation_state = STOPPED;
      }
      digitalWrite(SCOM_DIR, HIGH);
      delayMicroseconds(500);
      analogWrite(SCOM_PWM, current_pwm_value);
      break;

  }
}

void ConnectionLostISR(){
  connection_lost = true;
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