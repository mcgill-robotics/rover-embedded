#include <Arduino.h>
#include <Rover_Limits.h>

// Waist : LS_1
// Shoulder : LS_2, LS_3
// Elbow : LS_4, LS_5
// Wrist Pitch : LS_6, LS_7
// Wrist Roll : LS_8
// End Effector : LS_9, LS_10

Rover_Limits::Rover_Limits(uint8_t LS_1, uint8_t LS_2, uint8_t LS_3,uint8_t LS_4, uint8_t LS_5, uint8_t LS_6, uint8_t LS_7, uint8_t LS_8, uint8_t LS_9, uint8_t LS_10){
    this->LS_1 = LS_1;
    this->LS_2 = LS_2;
    this->LS_3 = LS_3;
    this->LS_4 = LS_4;
    this->LS_5 = LS_5;
    this->LS_6 = LS_6;
    this->LS_7 = LS_7;
    this->LS_8 = LS_8; 
    this->LS_9 = LS_9; 
    this->LS_10 = LS_10;
}

bool Rover_Limits::init(){
    pinMode(LS_1, INPUT);
    pinMode(LS_2, INPUT);
    pinMode(LS_3, INPUT);
    pinMode(LS_4, INPUT);
    pinMode(LS_5, INPUT);
    pinMode(LS_6, INPUT);
    pinMode(LS_7, INPUT);
    pinMode(LS_8, INPUT);
    pinMode(LS_9, INPUT);
    pinMode(LS_10, INPUT);
    hasChanged = false;
    return true;
}
/*
*   getLimitValue() : if LS_1 is set to QH' on shift register, then 0-7 in the array is H-A in pin number. 8 is LS_7 and 9 is LS_8, uint8_t LS_9, uint8_t,LS_10.
*/
bool Rover_Limits::getLimitValue(int pos){
    return limit_arr[pos];
}

bool Rover_Limits::updateValues(){
    // reset shift register
    // pulsePin(_clr_pin, true);
    // //load values into shift_register and set mode to shift right
    // digitalWrite(_s0_pin, HIGH);
    // pulsePin(_s1_pin);
    // //read first value before loop
    hasChanged = false;
    limit_arr[0] = digitalRead(LS_1);
    limit_arr[1] = digitalRead(LS_2);
    limit_arr[2] = digitalRead(LS_3);
    limit_arr[3] = digitalRead(LS_4);
    limit_arr[4] = digitalRead(LS_5);
    limit_arr[5] = digitalRead(LS_6);
    limit_arr[6] = digitalRead(LS_7);
    limit_arr[7] = digitalRead(LS_8);
    limit_arr[8] = digitalRead(LS_9);
    limit_arr[9] = digitalRead(LS_10);
    // //7x loop
    // for(int i = 1; i <= 7; i++){
    //     //pulse clock and shift right inputs
    //     pulsePin(_sr_pin);
    //     //read pin and write to array
    //     limit_arr[i] = digitalRead(_out_pin);
    // }
    // //add two extra limits to bool array
    // limit_arr[8] = digitalRead(_ex1_pin);
    // limit_arr[9] = digitalRead(_ex2_pin);
    return true;
}

// void Rover_Limits::pulsePin(uint8_t pin, bool reversed){
//     if(reversed){
//         digitalWrite(pin, LOW);
//     }
//     else{
//         digitalWrite(pin, HIGH);
//     }
//     delay(clk_delay);
//     if(pin != _clk_pin){
//         pulsePin(_clk_pin);
//     }
//     if(reversed){
//         digitalWrite(pin, HIGH);
//     }
//     else{
//         digitalWrite(pin, LOW);
//     }
//     delay(clk_delay);
// }

void Rover_Limits::ls_int(){
    // bool prev_limit_arr[10];
    // for(int i = 0; i <= 9; i++){
    //     prev_limit_arr[i] = limit_arr[i];
    // }
    Rover_Limits::updateValues();
    hasChanged = true;
}