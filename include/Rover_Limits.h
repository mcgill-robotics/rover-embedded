#ifndef Rover_Limits_h
#define Rover_Limits_h

#include <Arduino.h>

class Rover_Limits
{
    public:
        Rover_Limits(uint8_t LS_1, uint8_t LS_2, uint8_t LS_3,uint8_t LS_4, uint8_t LS_5, uint8_t LS_6, uint8_t LS_7, uint8_t LS_8, uint8_t LS_9, uint8_t LS_10);        bool init();
        bool getLimitValue(int pos);
        bool updateValues();
        volatile bool hasChanged;
    
    private:
        uint8_t LS_1;
        uint8_t LS_2;
        uint8_t LS_3;
        uint8_t LS_4;
        uint8_t LS_5;
        uint8_t LS_6;
        uint8_t LS_7;
        uint8_t LS_8;
        uint8_t LS_9; 
        uint8_t LS_10; 
        bool limit_arr[10] = {false};
        const int clk_delay = 1;
        void pulsePin(uint8_t pin, bool reversed = false);
        void ls_int();

};
#endif