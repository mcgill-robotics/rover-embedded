#ifndef Rover_Limits_h
#define Rover_Limits_h

#include <Arduino.h>

class Rover_Limits
{
    public:
        Rover_Limits(uint8_t out_pin, uint8_t clr_pin, uint8_t s1_pin,uint8_t s0_pin, uint8_t clk_pin, uint8_t sr_pin, uint8_t ex1_pin, uint8_t ex2_pin);
        bool init();
        bool getLimitValue(int pos);
        bool updateValues();
    
    private:
        uint8_t _out_pin;
        uint8_t _clr_pin;
        uint8_t _s1_pin;
        uint8_t _s0_pin;
        uint8_t _clk_pin;
        uint8_t _sr_pin;
        uint8_t _ex1_pin;
        uint8_t _ex2_pin;
        bool limit_arr[10];
        const int clk_delay = 1;
        void pulsePin(uint8_t pin, bool reversed = false);

};
#endif