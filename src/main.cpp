#include <Arduino.h>
#include <Serial/Serial.h>

void setup() 
{
    SerialAPI::init('a', 9600);
}

void loop() 
{
    if (SerialAPI::update())
    {
        char data[256];
        memset(data, 0, 256);

        if (SerialAPI::read_data(data, 256) != -1)
        {
            Serial.println(data);
        }
        const char* send_data = "Hello world from the MCU";
        const size_t len = strlen(send_data);
        SerialAPI::send_bytes('d', send_data, len);
    }

    {
    }

    delay(10);
}