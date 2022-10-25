#include <Arduino.h>
#include "peripheral.h"

peripheral::peripheral(int pin){
    this->pin = pin;
}

void peripheral::begin(int state){
    this->state = state;
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, this->state);  
}

void peripheral::turnOn(){
    digitalWrite(this->pin, HIGH); 
    this->state = HIGH;
}

void peripheral::turnOff(){
    digitalWrite(this->pin, LOW);
    this->state = LOW;
}

int peripheral::getState(){
    return this->state;
}


