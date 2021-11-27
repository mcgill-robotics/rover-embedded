#include <Arduino.h>
#include <Serial/Serial.h>

void setup() 
{
    Serial.begin(9600);
    SerialAPI::init('a', 9600);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  float speed;
  if (SerialAPI::update())
    {
        char data[256];
        memset(data, 0, 256);

        uint8_t packet_id=SerialAPI::read_data(data, 256);
        if (packet_id != -1)
        {
            speed = *(float*)data;
            Serial.println(speed);
        }
    }
    delay(10);
}