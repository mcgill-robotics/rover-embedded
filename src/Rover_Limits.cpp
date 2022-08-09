#include <Arduino.h>
#include <Rover_Limits.h>

Rover_Limits::Rover_Limits(uint8_t out_pin, uint8_t clr_pin, uint8_t s1_pin,uint8_t s0_pin, uint8_t clk_pin, uint8_t sr_pin, uint8_t ex1_pin, uint8_t ex2_pin){
    _out_pin = out_pin;
    _clr_pin = clr_pin;
    _s1_pin = s1_pin;
    _s0_pin = s0_pin;
    _clk_pin = clk_pin;
    _sr_pin = sr_pin;
    _ex1_pin = ex1_pin;
    _ex2_pin = ex2_pin;
    for(int i = 0; i <= 9; i++){
        limit_arr[i] = false;
    }

}

bool Rover_Limits::init(){
    pinMode(_out_pin, INPUT);
    pinMode(_clr_pin, OUTPUT);
    pinMode(_clk_pin, OUTPUT);
    pinMode(_s0_pin, OUTPUT);
    pinMode(_s1_pin, OUTPUT);
    pinMode(_sr_pin, OUTPUT);
    pinMode(_ex1_pin, INPUT);
    pinMode(_ex2_pin, INPUT);
    digitalWrite(_clr_pin, HIGH);
    digitalWrite(_s0_pin, LOW);
    digitalWrite(_s1_pin, LOW);
    digitalWrite(_sr_pin, LOW);
    digitalWrite(_clk_pin, LOW);
    return true;
}
/*
*   getLimitValue() : if out_pin is set to QH' on shift register, then 0-7 in the array is H-A in pin number. 8 is ex1_pin and 9 is ex2_pin.
*/
bool Rover_Limits::getLimitValue(int pos){
    return limit_arr[pos];
}

bool Rover_Limits::updateValues(){
    // reset shift register
    pulsePin(_clr_pin, true);
    //load values into shift_register and set mode to shift right
    digitalWrite(_s0_pin, HIGH);
    pulsePin(_s1_pin);
    //read first value before loop
    limit_arr[0] = digitalRead(_out_pin);
    //7x loop
    for(int i = 1; i <= 7; i++){
        //pulse clock and shift right inputs
        pulsePin(_sr_pin);
        //read pin and write to array
        limit_arr[i] = digitalRead(_out_pin);
    }
    //add two extra limits to bool array
    limit_arr[8] = digitalRead(_ex1_pin);
    limit_arr[9] = digitalRead(_ex2_pin);
    return true;
}

void Rover_Limits::pulsePin(uint8_t pin, bool reversed){
    if(reversed){
        digitalWrite(pin, LOW);
    }
    else{
        digitalWrite(pin, HIGH);
    }
    delay(clk_delay);
    if(pin != _clk_pin){
        pulsePin(_clk_pin);
    }
    if(reversed){
        digitalWrite(pin, HIGH);
    }
    else{
        digitalWrite(pin, LOW);
    }
    delay(clk_delay);
}