#include <Arduino.h>
#include <Serial/Serial.h>

void setup() 
{
    SerialAPI::init('a', 9600);
}

void loop() 
{
    if (SerialAPI::update())
    {}

    {
        float speed = 0.5f;
        SerialAPI::send_bytes('d', &speed, sizeof(speed));
    }

    delay(10);
}