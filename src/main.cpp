#include <Arduino.h>
#include <Serial/Serial.h>
#include <wiring_private.h>

void setup() 
{
    Serial.begin(9600);
    SerialAPI::init('a', 9600);
}

// TODO: Once the bidirectional ESC is selected, change this
// to accomodate the new control scheme:
// 50% = 0
// 0% = -1.0
// 100% = 1.0
void setMotorSpeed(int pin_number, double normalized_speed){
  
  double y = 0.51 * normalized_speed + 0.47;
  int z = (int) (255 * y);
  PWMWrite(pin_number, 255, z, 490);
  
}

// If you need to reprogram the transmitter TM4C, use the
// code in the "serial" branch instead. This is only for
// the receiver (i.e. the drive board)

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
            // Stopped working right after Roey left. It's not like
            // we changed anything afterwards.
            speed = *(float*)data;

            // Hardcode the maximum to calibrate
            //speed = 1.0f;
            Serial.println(speed);
            setMotorSpeed(PD_3, speed);
        }

    }
    else{
        // Motor should only stop when it gets a 0.0f. Before some
        // adjustments it would stop at 0.2f which defeats the purpose
        // of normalizing the speed
        setMotorSpeed(PD_3, 0.0f);
    }
    delay(10);
}