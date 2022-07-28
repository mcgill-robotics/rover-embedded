#include <Servo.h>
#include <tm4c123gh6pm.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include <Rover_SerialAPI.h>
#include <Arduino.h>
#include <driverlib/timer.h>

/*
 * PINOUT NOTE:
 * 
 * All PWM output pins are configured
 * as open-drain for them to be able to work with 5V logic
 * level. As for Hall Sensor inputs, the TM4C pins are 5V-tolerant
 * to input already.
 * 
 */

#define CW 1
#define CCW -1

#define LB_MOTOR_HALL_A PA_2
#define RB_MOTOR_HALL_A PA_3
#define LF_MOTOR_HALL_A PA_4
#define RF_MOTOR_HALL_A PA_5

#define LB_MOTOR_HALL_B PC_4
#define RB_MOTOR_HALL_B PC_5
#define LF_MOTOR_HALL_B PC_6
#define RF_MOTOR_HALL_B PC_7

#define LB_MOTOR_HALL_C PF_1
#define RB_MOTOR_HALL_C PF_2
#define LF_MOTOR_HALL_C PF_3
#define RF_MOTOR_HALL_C PF_4

#define LB_SERVO_PIN PB_4
#define RB_SERVO_PIN PB_5
#define LF_SERVO_PIN PB_6
#define RF_SERVO_PIN PB_7

#define LB_CURRENT_SENSE_PIN PD_0
#define RB_CURRENT_SENSE_PIN PD_1
#define LF_CURRENT_SENSE_PIN PD_2
#define RF_CURRENT_SENSE_PIN PD_3

#define LB_CURRENT_NFAULT_PIN PE_0
#define RB_CURRENT_NFAULT_PIN PE_1
#define LF_CURRENT_NFAULT_PIN PE_2
#define RF_CURRENT_NFAULT_PIN PE_3


/**
 * Motors and the maximum acceleration we can do without fucking up their gearboxes,
 * expressed in arbitrary -100 to 100 units. Acceleration is per
 * loop (approx. 100ms right now, could go down to 10ms)
 */
Servo LBservo, LFservo, RBservo, RFservo;
#define MAX_ACCEL 5

volatile int direction;
int getRPM(long time_elapsed, long pulses);

void setPinAsOpenDrain(char port, int pin, int output);
void LFHallSensorA(), LFHallSensorB(), LFHallSensorC(),
     LBHallSensorA(), LBHallSensorB(), LBHallSensorC(),
     RFHallSensorA(), RFHallSensorB(), RFHallSensorC(),
     RBHallSensorA(), RBHallSensorB(), RBHallSensorC();

/**
 * Serial structs and function signatures. Could we make these into a library pls
 * 
 */
#define SERIAL_RX_BUFFER_SIZE 64
typedef struct {
    float b[SERIAL_RX_BUFFER_SIZE/sizeof(float)];
    size_t count;
} FloatBuffer;
FloatBuffer fbuf;
void AddFloatToBuffer(FloatBuffer fb, float val);
void print_byte_array(byte* byte_array, size_t size);
void char_to_float(char* str_byte, float* f);
bool syn2master(HardwareSerial Serial);
void AccelInt(void);
static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuffer[SERIAL_RX_BUFFER_SIZE];
volatile int lb_cur_speed = 1500;
volatile int lf_cur_speed = 1500;
volatile int rb_cur_speed = 1500;
volatile int rf_cur_speed = 1500;
#define ID 'a'
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

// the setup routine runs once when you press reset:
void setup() {           
  SerialAPI::init('1', 9600);    
  setPinAsOpenDrain('B', 4, 1);
  setPinAsOpenDrain('B', 5, 1);
  setPinAsOpenDrain('B', 6, 1);
  setPinAsOpenDrain('B', 7, 1);

  pinMode(LB_MOTOR_HALL_A, INPUT);
  pinMode(LB_MOTOR_HALL_B, INPUT);
  pinMode(LB_MOTOR_HALL_C, INPUT);
  pinMode(RB_MOTOR_HALL_A, INPUT);
  pinMode(RB_MOTOR_HALL_B, INPUT);
  pinMode(RB_MOTOR_HALL_C, INPUT);
  pinMode(LF_MOTOR_HALL_A, INPUT);
  pinMode(LF_MOTOR_HALL_B, INPUT);
  pinMode(LF_MOTOR_HALL_C, INPUT);
  pinMode(RF_MOTOR_HALL_A, INPUT);
  pinMode(RF_MOTOR_HALL_B, INPUT);
  pinMode(RF_MOTOR_HALL_C, INPUT);

  pinMode(LB_CURRENT_SENSE_PIN, INPUT);
  pinMode(RB_CURRENT_SENSE_PIN, INPUT);
  pinMode(LF_CURRENT_SENSE_PIN, INPUT);
  pinMode(RF_CURRENT_SENSE_PIN, INPUT);
  pinMode(LB_CURRENT_NFAULT_PIN, INPUT);
  pinMode(RB_CURRENT_NFAULT_PIN, INPUT);
  pinMode(LF_CURRENT_NFAULT_PIN, INPUT);
  pinMode(RF_CURRENT_NFAULT_PIN, INPUT);

  LBservo.attach(LB_SERVO_PIN);
  LFservo.attach(LF_SERVO_PIN);
  RBservo.attach(RB_SERVO_PIN);
  RFservo.attach(RF_SERVO_PIN);

  attachInterrupt(LF_MOTOR_HALL_A, LFHallSensorA, CHANGE);
  attachInterrupt(LF_MOTOR_HALL_B, LFHallSensorB, CHANGE);
  attachInterrupt(LF_MOTOR_HALL_C, LFHallSensorC, CHANGE);

  attachInterrupt(LB_MOTOR_HALL_A, LBHallSensorA, CHANGE);
  attachInterrupt(LB_MOTOR_HALL_B, LBHallSensorB, CHANGE);
  attachInterrupt(LB_MOTOR_HALL_C, LBHallSensorC, CHANGE);

  attachInterrupt(RB_MOTOR_HALL_A, RBHallSensorA, CHANGE);
  attachInterrupt(RB_MOTOR_HALL_B, RBHallSensorB, CHANGE);
  attachInterrupt(RB_MOTOR_HALL_C, RBHallSensorC, CHANGE);

  attachInterrupt(RF_MOTOR_HALL_A, RFHallSensorA, CHANGE);
  attachInterrupt(RF_MOTOR_HALL_B, RFHallSensorB, CHANGE);
  attachInterrupt(RF_MOTOR_HALL_C, RFHallSensorC, CHANGE);
  
  // Lock motors and get ready to go
  LBservo.writeMicroseconds(1500);
  LFservo.writeMicroseconds(1500);
  RBservo.writeMicroseconds(1500);
  RFservo.writeMicroseconds(1500);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)){}
  TimerConfigure(TIMER4_BASE, TIMER_CFG_A_PERIODIC);
  TimerLoadSet(TIMER4_BASE, TIMER_A, 4000); //every 5ms
  TimerIntRegister(TIMER4_BASE, TIMER_A, AccelInt);
  TimerEnable(TIMER4_BASE, TIMER_A);

  delay(3000);
}

volatile long lb_hall_a_interrupts_raw = 0;
volatile long lb_hall_b_interrupts_raw = 0;
volatile long lb_hall_c_interrupts_raw = 0;

volatile long lf_hall_a_interrupts_raw = 0;
volatile long lf_hall_b_interrupts_raw = 0;
volatile long lf_hall_c_interrupts_raw = 0;

volatile long rb_hall_a_interrupts_raw = 0;
volatile long rb_hall_b_interrupts_raw = 0;
volatile long rb_hall_c_interrupts_raw = 0;

volatile long rf_hall_a_interrupts_raw = 0;
volatile long rf_hall_b_interrupts_raw = 0;
volatile long rf_hall_c_interrupts_raw = 0;

volatile int lb_direction = 1, rb_direction = 1, rf_direction = 1, lf_direction = 1;
float lb_regress_value, lf_regress_value, rb_regress_value, rf_regress_value;
float lb_hall_a_interrupts_per_second, rb_hall_a_interrupts_per_second,
    rf_hall_a_interrupts_per_second, lf_hall_a_interrupts_per_second;

char us_buf[20];
float speeds[4];
char buf[17];

float lb_new_us, lf_new_us, rb_new_us, rf_new_us, lb_prev_us=1500, lf_prev_us=1500, rb_prev_us=1500, rf_prev_us=1500;
unsigned long integration_period_start, integration_period_end;
long time_elapsed;
float lb_target_speed, rb_target_speed, lf_target_speed, rf_target_speed;
float filtered_lb_target_speed, filtered_rb_target_speed, filtered_lf_target_speed, filtered_rf_target_speed;
int filtered_lb_us, filtered_lf_us, filtered_rb_us, filtered_rf_us;

int loops = 0;
bool lb_leap_up=false, lb_leap_down=false, rb_leap_up=false, rb_leap_down=false,
     rf_leap_up=false, rf_leap_down=false, lf_leap_up=false, lf_leap_down=false;

void loop() {
  // put your main code here, to run repeatedly:
  lb_cur_speed = LBservo.readMicroseconds();
  lf_cur_speed = LFservo.readMicroseconds();
  rb_cur_speed = RBservo.readMicroseconds();
  rf_cur_speed = RFservo.readMicroseconds();
  integration_period_start = millis();
  delay(100);
  integration_period_end = millis();

  time_elapsed = integration_period_end - integration_period_start;

  lb_hall_a_interrupts_per_second = lb_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  lb_regress_value = (lb_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  rb_hall_a_interrupts_per_second = rb_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  rb_regress_value = (rb_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  lf_hall_a_interrupts_per_second = lf_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  lf_regress_value = (lf_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  rf_hall_a_interrupts_per_second = rf_hall_a_interrupts_raw * (1000.0f / (float)time_elapsed);
  rf_regress_value = (rf_hall_a_interrupts_per_second - 24.995f) / 30.178f;

  lb_hall_a_interrupts_raw = 0;
  rb_hall_a_interrupts_raw = 0;
  lf_hall_a_interrupts_raw = 0;
  rf_hall_a_interrupts_raw = 0;

  /**
   * Get new commands from main computer, and send the actual speed
   * 
   */
   if(SerialAPI::update()){

       memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
       int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));

       //const size_t payload_size = strlen(buffer); //DOESN'T WORK IF THERE ARE ZEROs BECAUSE IT'S CONSIDERED A NULL CHARACTER

       memcpy(speeds, buffer+1, 16);

       lb_target_speed = speeds[0];
       lf_target_speed = speeds[1]; 
       rb_target_speed = speeds[2];
       rf_target_speed = speeds[3];

       delay(50);

       buf[0] = '1';

       memcpy(buf+1, speeds, 16);

       SerialAPI::send_bytes('0', buf, 17);

      us_buf[0] = '1';
      memcpy(us_buf+1, &lb_regress_value, 4);
      memcpy(us_buf+5, &rb_regress_value, 4);
      memcpy(us_buf+9, &lf_regress_value, 4);
      memcpy(us_buf+13, &rf_regress_value, 4);
      SerialAPI::send_bytes('0', us_buf, 17);

      delay(100); 

  }  

  // Max forward speed is 1900us, max backward speed is 1100us
  
//   // If there is a big difference between current and target speeds, accel slowly
//   filtered_lb_target_speed = lb_target_speed;
//   filtered_rb_target_speed = rb_target_speed;
//   filtered_lf_target_speed = lf_target_speed;
//   filtered_rf_target_speed = rf_target_speed;

// ///////////////////////////////////////////////////////////////////////////

//   if( (lb_target_speed - lb_regress_value) > 20){
//     lb_leap_up = true;
//     lb_leap_down = false;
//     filtered_lb_target_speed = lb_regress_value;
//   }

//   if( (lb_target_speed - lb_regress_value) < -20){
//     lb_leap_down = true;
//     lb_leap_up = false;
//     filtered_lb_target_speed = lb_regress_value;
//   }

//   if(lb_leap_up && (loops % 3 == 0) ){
//     filtered_lb_target_speed++;

//   }else if(lb_leap_down && (loops % 3 == 0) ){
//     filtered_lb_target_speed--;
//   }

// /////////////////////////////////////////////////////////////////////////

//   if( (lf_target_speed - lf_regress_value) > 20){
//     lf_leap_up = true;
//     lf_leap_down = false;
//     filtered_lf_target_speed = lf_regress_value;
//   }

//   if( (lf_target_speed - lf_regress_value) < -20){
//     lf_leap_down = true;
//     lf_leap_up = false;
//     filtered_lf_target_speed = lf_regress_value;
//   }

//   if(lf_leap_up && (loops % 3 == 0) ){
//     filtered_lf_target_speed++;

//   }else if(lf_leap_down && (loops % 3 == 0) ){
//     filtered_lf_target_speed--;
//   }

// //////////////////////////////////////////////////////////////////////////

//   if( (rb_target_speed - rb_regress_value) > 20){
//     rb_leap_up = true;
//     rb_leap_down = false;
//     filtered_rb_target_speed = rb_regress_value;
//   }

//   if( (rb_target_speed - rb_regress_value) < -20){
//     rb_leap_down = true;
//     rb_leap_up = false;
//     filtered_rb_target_speed = rb_regress_value;
//   }

//   if(rb_leap_up && (loops % 3 == 0) ){
//     filtered_rb_target_speed++;

//   }else if(rb_leap_down && (loops % 3 == 0) ){
//     filtered_rb_target_speed--;
//   }

// /////////////////////////////////////////////////////////////////////////

//   if( (rf_target_speed - rf_regress_value) > 20){
//     rf_leap_up = true;
//     rf_leap_down = false;
//     filtered_rf_target_speed = rf_regress_value;
//   }

//   if( (rf_target_speed - rf_regress_value) < -20){
//     rf_leap_down = true;
//     rf_leap_up = false;
//     filtered_rf_target_speed = rf_regress_value;
//   }

//   if(rf_leap_up && (loops % 3 == 0) ){
//     filtered_rf_target_speed++;

//   }else if(rf_leap_down && (loops % 3 == 0) ){
//     filtered_rf_target_speed--;
//   }

// //////////////////////////////////////////////////////////////////////////

  lb_new_us = (1500.0f + 4.0f * lb_target_speed);
  lf_new_us = (1500.0f + 4.0f * lf_target_speed);
  rb_new_us = (1500.0f + 4.0f * rb_target_speed);
  rf_new_us = (1500.0f + 4.0f * rf_target_speed);

  // if(lb_new_us != lb_prev_us) LBservo.writeMicroseconds((int) lb_new_us);
  // if(lf_new_us != lf_prev_us) LFservo.writeMicroseconds((int) lf_new_us);
  // if(rb_new_us != rb_prev_us) RBservo.writeMicroseconds((int) rb_new_us);
  // if(rf_new_us != rf_prev_us) RFservo.writeMicroseconds((int) rf_new_us);

  lb_prev_us = lb_new_us;
  lf_prev_us = lf_new_us;
  rb_prev_us = rb_new_us;
  rf_prev_us = rf_new_us;

  loops++;

}

void LFHallSensorA() {        
  lf_direction = (digitalRead(LF_MOTOR_HALL_A) == digitalRead(LF_MOTOR_HALL_B)) ? CW : CCW;   
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

void LFHallSensorB() {
  lf_direction = (digitalRead(LF_MOTOR_HALL_B) == digitalRead(LF_MOTOR_HALL_C)) ? CW : CCW;
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

void LFHallSensorC() {
  lf_direction = (digitalRead(LF_MOTOR_HALL_C) == digitalRead(LF_MOTOR_HALL_A)) ? CW : CCW;
  lf_hall_a_interrupts_raw = lf_hall_a_interrupts_raw + lf_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void LBHallSensorA() {        
  lb_direction = (digitalRead(LB_MOTOR_HALL_A) == digitalRead(LB_MOTOR_HALL_B)) ? CW : CCW;   
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void LBHallSensorB() {
  lb_direction = (digitalRead(LB_MOTOR_HALL_B) == digitalRead(LB_MOTOR_HALL_C)) ? CW : CCW;
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void LBHallSensorC() {
  lb_direction = (digitalRead(LB_MOTOR_HALL_C) == digitalRead(LB_MOTOR_HALL_A)) ? CW : CCW;
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void RBHallSensorA() {        
  lb_direction = (digitalRead(RB_MOTOR_HALL_A) == digitalRead(RB_MOTOR_HALL_B)) ? CW : CCW;   
  lb_hall_a_interrupts_raw = lb_hall_a_interrupts_raw + lb_direction; 
}

void RBHallSensorB() {
  rb_direction = (digitalRead(RB_MOTOR_HALL_B) == digitalRead(RB_MOTOR_HALL_C)) ? CW : CCW;
  rb_hall_a_interrupts_raw = rb_hall_a_interrupts_raw + rb_direction; 
}

void RBHallSensorC() {
  rb_direction = (digitalRead(RB_MOTOR_HALL_C) == digitalRead(RB_MOTOR_HALL_A)) ? CW : CCW;
  rb_hall_a_interrupts_raw = rb_hall_a_interrupts_raw + rb_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void RFHallSensorA() {        
  rf_direction = (digitalRead(RF_MOTOR_HALL_A) == digitalRead(RF_MOTOR_HALL_B)) ? CW : CCW;   
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

void RFHallSensorB() {
  rf_direction = (digitalRead(RF_MOTOR_HALL_B) == digitalRead(RF_MOTOR_HALL_C)) ? CW : CCW;
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

void RFHallSensorC() {
  rf_direction = (digitalRead(RF_MOTOR_HALL_C) == digitalRead(RF_MOTOR_HALL_A)) ? CW : CCW;
  rf_hall_a_interrupts_raw = rf_hall_a_interrupts_raw + rf_direction; 
}

////////////////////////////////////////////////////////////////////////////////////////////

void AccelInt(){
  //lb
  if(lb_new_us > lb_cur_speed){
    LBservo.writeMicroseconds(lb_cur_speed+1);
    lb_cur_speed +=1;
  }
  else if(lb_new_us < lb_cur_speed){
    LBservo.writeMicroseconds(lb_cur_speed-1);
    lb_cur_speed -= 1;
  }
  //lf
  if(lf_new_us > lf_cur_speed){
    LBservo.writeMicroseconds(lf_cur_speed+1);
    lf_cur_speed +=1;
  }
  else if(lf_new_us < lf_cur_speed){
    LBservo.writeMicroseconds(lf_cur_speed-1);
    lf_cur_speed -= 1;
  }
  //rb
  if(rb_new_us > rb_cur_speed){
    LBservo.writeMicroseconds(rb_cur_speed+1);
    rb_cur_speed +=1;
  }
  else if(rb_new_us < rb_cur_speed){
    LBservo.writeMicroseconds(rb_cur_speed-1);
    rb_cur_speed -= 1;
  }
  //rf
  if(rf_new_us > rf_cur_speed){
    LBservo.writeMicroseconds(rf_cur_speed+1);
    rf_cur_speed +=1;
  }
  else if(rf_new_us < rf_cur_speed){
    LBservo.writeMicroseconds(rf_cur_speed-1);
    rf_cur_speed -= 1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////

void setPinAsOpenDrain(char port, int pin, int output){
  int amsel_register, pctl_register, dir_register, afsel_register, odr_register, den_register, data_register, pin_idx = pin;

  if(1 > pin || pin > 7){return;}
  if('A' > port || port > 'F'){return;}
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
      // GPIO_PORTA_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

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
      // GPIO_PORTB_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

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
      // GPIO_PORTC_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

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
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

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
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

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
      // GPIO_PORTD_DR4R_R |= (1<<pin_idx);                   // enable higher current drive as needed

      GPIO_PORTF_ODR_R |= (1 << pin_idx);                     // digital enable
      GPIO_PORTF_DATA_R &= (1 << pin_idx);                    // digital enable 2
      return;

    default:
      return;        
  }
  return;
}