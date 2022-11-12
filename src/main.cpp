#include <Arduino.h>
#include <Servo.h>
#include <movingAvg.h>
#include <SimpleKalmanFilter.h>

#define PWM_OUT PB_3
#define HALL_SENSOR_A PA_4
#define HALL_SENSOR_B PA_3
#define HALL_SENSOR_C PA_2

volatile unsigned long hall_a_interrupts = 0;
volatile unsigned long hall_b_interrupts = 0;
volatile unsigned long hall_c_interrupts = 0;

volatile float ips = 0;
volatile int coeff = 1;

volatile int speed = -100;

volatile int dir = 1;

void hallARising();
void hallBRising();
void hallCRising();

void updateSpeed();
void integrateSensors();

Servo motor;
movingAvg hallASmooth(5), hallBSmooth(5), hallCSmooth(5);
SimpleKalmanFilter hallKalmanFilter(1, 1, 0.01);


/*
TODO1: Try to optimize by changing the sampling rate
TODO2: Try to change capacitor values
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PWM_OUT, OUTPUT);
  pinMode(HALL_SENSOR_A, INPUT);
  pinMode(HALL_SENSOR_B, INPUT);
  pinMode(HALL_SENSOR_C, INPUT);
  motor.attach(PWM_OUT);

  attachInterrupt(HALL_SENSOR_A, hallARising, FALLING);
  attachInterrupt(HALL_SENSOR_B, hallBRising, FALLING);
  attachInterrupt(HALL_SENSOR_C, hallCRising, FALLING);

  // Timer1.initialize(1000000);
  // Timer1.attachInterrupt(updateSpeed); // blinkLED to run every 0.15 seconds

  motor.writeMicroseconds(1500);
  delay(3000);
  // motor.writeMicroseconds(1700);
  hallASmooth.begin();
  hallBSmooth.begin();
  hallCSmooth.begin();  
}

unsigned long t = 0;
unsigned long loops = 0;
void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

  // int halla2 = hallASmooth.reading(hall_a_interrupts);
  // int hallb2 = hallBSmooth.reading(hall_b_interrupts);
  // int hallc2 = hallCSmooth.reading(hall_c_interrupts);
  int halla2 = hall_a_interrupts;
  int hallb2 = hall_b_interrupts;
  int hallc2 = hall_c_interrupts;

  hall_a_interrupts = 0;
  hall_b_interrupts = 0;
  hall_c_interrupts = 0;

  float hall = ((float)(halla2 + hallb2 + hallc2) / 3.0f);
  int current_hall = hallKalmanFilter.updateEstimate(hall);
  Serial.print(speed);
  Serial.print(",");

  Serial.println(current_hall);
  // Serial.print(",");
  // Serial.print(hallb2);
  // Serial.print(",");
  // Serial.print(hallc2);
  // Serial.print(",");

  if(hallc2 > hallb2){
    Serial.println("500");
  }else{
    Serial.println("0");
  }

  if(loops % 20 == 0){
    updateSpeed();
    if(speed == 80){speed += (coeff * 1);}
    if(speed == -48){speed += (coeff * 1);}
    if(speed == 16){speed += (coeff * 1);}
    int writeval = (int) (1500 + 4*speed);

    motor.writeMicroseconds(writeval);
  }

  loops++;

}

void hallARising(){
  hall_a_interrupts++;
}

void hallBRising(){
  hall_b_interrupts++;
}

void hallCRising(){
  hall_c_interrupts++;
}

void updateSpeed(){

    if(speed <= -100){
      coeff = 1;
    }else if(speed >= 100){
      coeff = -1;
    }

    speed += coeff;
}