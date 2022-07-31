#include <Arduino.h>
#include <Rover_SerialAPI.h>
#include <bitset>

#define SERIAL_RX_BUFFER_SIZE 64

void setup() {
  // put your setup code here, to run once:
  SerialAPI::init('4', 9600);

  // blue = drive, red = arm, green = science
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

static char rx_buffer[64];
static char tx_buffer[13];
float ctl_floats[4];
float drive, arm, science;

void loop() {
  // put your main code here, to run repeatedly:
  if(SerialAPI::update()){

    memset(rx_buffer, 0, SERIAL_RX_BUFFER_SIZE);
    int cur_pack_id = SerialAPI::read_data(rx_buffer,sizeof(rx_buffer));
    memcpy(ctl_floats, rx_buffer+1, 16);

    drive = ctl_floats[0];
    arm = ctl_floats[1];
    science = ctl_floats[2];

    (drive > 0.0f) ? digitalWrite(BLUE_LED, HIGH) : digitalWrite(BLUE_LED, LOW);
    (arm > 0.0f) ? digitalWrite(RED_LED, HIGH) : digitalWrite(RED_LED, LOW);
    (science > 0.0f) ? digitalWrite(GREEN_LED, HIGH) : digitalWrite(GREEN_LED, LOW);

    tx_buffer[0] = '4';
    memcpy(tx_buffer+1, &drive, 4);
    memcpy(tx_buffer+5, &arm, 4);
    memcpy(tx_buffer+9, &science, 4);
    SerialAPI::send_bytes('0', tx_buffer, 13);

  }
  
}