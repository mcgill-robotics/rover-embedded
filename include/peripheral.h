#include <Arduino.h>

class peripheral{
    public: 
        peripheral(int pin); 

        void turnOn(); 
        void turnOff();
        void begin(int state);
        int getState(); 
        int pin; 
        int state; 
};