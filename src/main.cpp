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
        static float speed = 0.f;
        speed += 0.01f;
        if (speed > 1.f)
        {
            speed = -1.f;
        }
        SerialAPI::send_bytes('d', &speed, sizeof(speed));
    }

    delay(10);
}