#include <Servo.h>
#include <tm4c123gh6pm.h>
#include <Rover_SerialAPI.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/*
 * PINOUT NOTE:
 * 
 * All PWM output pins are configured
 * as open-drain for them to be able to work with 5V logic
 * level. As for Hall Sensor inputs, the TM4C pins are 5V-tolerant
 * to input already.
 * 
 */

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

/**
 * Hall Sensor ISR's, pulse counters and variables
 */
void LBHallSensorA();
void LBHallSensorB();
void LBHallSensorC();

void RBHallSensorA();
void RBHallSensorB();
void RBHallSensorC();

void LFHallSensorA();
void LFHallSensorB();
void LFHallSensorC();

void RFHallSensorA();
void RFHallSensorB();
void RFHallSensorC();

volatile long lf_pulses=0, lb_pulses=0, rb_pulses=0, rf_pulses=0;
unsigned long integration_period_start=0, integration_period_end=100, time_elapsed=100;
int lb_regress_value=0, lb_rpm=0, rb_regress_value=0, rb_rpm=0;
int lf_regress_value=0, lf_rpm=0, rf_regress_value=0, rf_rpm=0;
float lf_pulses_per_second,lb_pulses_per_second,rf_pulses_per_second,rb_pulses_per_second;
int lb_target_speed, rb_target_speed, lf_target_speed, rf_target_speed;
int getRPM(long time_elapsed, long pulses);

void setPinAsOpenDrain(char port, int pin, int output);

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
static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuffer[SERIAL_RX_BUFFER_SIZE];
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

  attachInterrupt(PA_2, LBHallSensorA, GPIO_RISING_EDGE);
  attachInterrupt(PA_3, RBHallSensorA, GPIO_RISING_EDGE);
  attachInterrupt(PA_4, LFHallSensorA, GPIO_RISING_EDGE);
  attachInterrupt(PA_5, RFHallSensorA, GPIO_RISING_EDGE);

  attachInterrupt(PC_4, LBHallSensorB, GPIO_RISING_EDGE);
  attachInterrupt(PC_5, RBHallSensorB, GPIO_RISING_EDGE);
  attachInterrupt(PC_6, LFHallSensorB, GPIO_RISING_EDGE);
  attachInterrupt(PC_7, RFHallSensorB, GPIO_RISING_EDGE);		

  attachInterrupt(PF_1, LBHallSensorC, GPIO_RISING_EDGE);
  attachInterrupt(PF_2, RBHallSensorC, GPIO_RISING_EDGE);
  attachInterrupt(PF_3, LFHallSensorC, GPIO_RISING_EDGE);
  attachInterrupt(PF_4, RFHallSensorC, GPIO_RISING_EDGE);	
  
  SerialAPI::init(ID, 9600);
  // Lock motors and get ready to go
  LBservo.writeMicroseconds(1500);
  LFservo.writeMicroseconds(1500);
  RBservo.writeMicroseconds(1500);
  RFservo.writeMicroseconds(1500);

  delay(3000);
}

int value = -100;
int value_actual = -100;
bool increasing = true;
int loops = 0;
float speeds[4];

void loop() {

  integration_period_start = millis();

  /**
   * Get new commands from main computer, and send the actual speed
   * 
   */
  if(SerialAPI::update()){

      memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
      int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));

      //const size_t payload_size = strlen(buffer); //DOESN'T WORK IF THERE ARE ZEROs BECAUSE IT'S CONSIDERED A NULL CHARACTER

      memcpy(&speeds, buffer+1, 16);

      lb_target_speed = (int) speeds[0];
      rb_target_speed = (int) speeds[1]; 
      lf_target_speed = (int) speeds[2];
      rf_target_speed = (int) speeds[3];

      delay(100);

      SerialAPI::send_bytes('1', buffer, 5);

  }  

  // Weirdest bug ever...
  value_actual = value;
  if(value == -92) value_actual = -91;
  if(value == -89 || value == -88) value_actual = -87;
  if(value == -60) value_actual = -59;
  if(value == -28) value_actual = -27;
  if(value == 4) value_actual = 3;
  if(value == 36) value_actual = 35;
  if(value == 68) value_actual = 67;
  if(value == 100) value_actual = 99;

  // lb_target_speed = value_actual;
  // lf_target_speed = value_actual;


  // Map value to microseconds and write to ESC's
  int lb_new_us = map(lb_target_speed, -100, 100, 1100, 1900);
  int lf_new_us = map(-lf_target_speed, -100, 100, 1100, 1900);
  int rb_new_us = map(-rb_target_speed, -100, 100, 1100, 1900);
  int rf_new_us = map(-rf_target_speed, -100, 100, 1100, 1900);

  LBservo.writeMicroseconds(lb_new_us);
  LFservo.writeMicroseconds(lf_new_us);
  RBservo.writeMicroseconds(rb_new_us);
  RFservo.writeMicroseconds(rf_new_us);

  loops++;
  integration_period_end = millis();
  time_elapsed = integration_period_end - integration_period_start;

  int lb_rpm = getRPM(time_elapsed, lb_pulses);

  // if(loops % 20 == 0){
  //   Serial.print(value);
  //   Serial.print(",");
  //   Serial.println(lb_rpm);
  // }


  lb_pulses = 0;
  lf_pulses = 0;
  rb_pulses = 0;
  rf_pulses = 0;
}

int getRPM(long time_elapsed, long pulses){
  int interrupts_per_second = pulses * (1000.0f / (float) time_elapsed);
  int regress_value = (int) (interrupts_per_second - 24.995f) / 30.178f;
  return map(regress_value, -100, 100, -4300, 4300);
}

/**
 * Hall Sensor ISR's
 * 
 */
volatile int direction = 1;
#define CW 1
#define CCW -1

/**
 * Serial communication helper functions
 */
void AddFloatToBuffer(FloatBuffer fb,float val)
{
    if(fb.count < SERIAL_RX_BUFFER_SIZE/sizeof(float))
    {
        fb.b[fb.count] = val;
        fb.count++;
    }
}

void decode_msg(char* buffer){

}

void sync(void){
  //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(!SerialAPI::update()) delay(1000);

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  while(!(buffer[1] == 'Y')){
    memset(buffer,0,SERIAL_RX_BUFFER_SIZE);

    //Ask for a retransmit of wrong ID
    SerialAPI::send_retransmit();

    //Wait for answer
    while(!SerialAPI::update()) delay(1000);

    //int tmp = Serial.available();
    //while(Serial.available()==tmp) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  

  //External validation
  for(int i=0;i<5;i++){
    digitalWrite(13,LOW);
    delay(200);
    digitalWrite(13,HIGH);
    delay(200);
  }
}

void print_byte_array(byte* byte_array, size_t size){
  char buffer[1];
  memset(buffer,0,1);
  for (int i = 0; i<size;i++){
    sprintf(buffer, BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(byte(byte_array[i])));
    Serial.print(buffer);
  }
}

void char_to_float(char* str_byte, float* f){
  char array[4];

  #ifdef BIGENDIAN
  array[0] = byte(str_byte[3]);
  array[1] = byte(str_byte[2]);
  array[2] = byte(str_byte[1]);
  array[3] = byte(str_byte[0]);
  #else 
  array[0] = byte(str_byte[0]);
  array[1] = byte(str_byte[1]);
  array[2] = byte(str_byte[2]);
  array[3] = byte(str_byte[3]);
  #endif

  memcpy(f,array,4);
}

void LBHallSensorA() {        
  direction = (digitalRead(LB_MOTOR_HALL_A) == digitalRead(LB_MOTOR_HALL_B)) ? CW : CCW;
  lb_pulses += direction;   
}

void LBHallSensorB() {
  direction = (digitalRead(LB_MOTOR_HALL_B) == digitalRead(LB_MOTOR_HALL_C)) ? CW : CCW;
  lb_pulses += direction; 
}

void LBHallSensorC() {
  direction = (digitalRead(LB_MOTOR_HALL_C) == digitalRead(LB_MOTOR_HALL_A)) ? CW : CCW;
  lb_pulses += direction; 
}

void LFHallSensorA() {        
  direction = (digitalRead(LF_MOTOR_HALL_A) == digitalRead(LF_MOTOR_HALL_B)) ? CW : CCW;
  lb_pulses += direction;   
}

void LFHallSensorB() {
  direction = (digitalRead(LF_MOTOR_HALL_B) == digitalRead(LF_MOTOR_HALL_C)) ? CW : CCW;
  lb_pulses += direction; 
}

void LFHallSensorC() {
  direction = (digitalRead(LF_MOTOR_HALL_C) == digitalRead(LF_MOTOR_HALL_A)) ? CW : CCW;
  lb_pulses += direction; 
}

void RBHallSensorA() {        
  direction = (digitalRead(RB_MOTOR_HALL_A) == digitalRead(RB_MOTOR_HALL_B)) ? CW : CCW;
  lb_pulses += direction;   
}

void RBHallSensorB() {
  direction = (digitalRead(RB_MOTOR_HALL_B) == digitalRead(RB_MOTOR_HALL_C)) ? CW : CCW;
  lb_pulses += direction; 
}

void RBHallSensorC() {
  direction = (digitalRead(RB_MOTOR_HALL_C) == digitalRead(RB_MOTOR_HALL_A)) ? CW : CCW;
  lb_pulses += direction; 
}

void RFHallSensorA() {        
  direction = (digitalRead(RF_MOTOR_HALL_A) == digitalRead(RF_MOTOR_HALL_B)) ? CW : CCW;
  lb_pulses += direction;   
}

void RFHallSensorB() {
  direction = (digitalRead(RF_MOTOR_HALL_B) == digitalRead(RF_MOTOR_HALL_C)) ? CW : CCW;
  lb_pulses += direction; 
}

void RFHallSensorC() {
  direction = (digitalRead(RF_MOTOR_HALL_C) == digitalRead(RF_MOTOR_HALL_A)) ? CW : CCW;
  lb_pulses += direction; 
}

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