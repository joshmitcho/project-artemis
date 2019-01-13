// Stepper - Version: Latest 
#include <Stepper.h>

char in_bytes[10];   // for incoming serial data
const int start_seq = 36;
const int end_seq = 47;
int x_coord = 0;
int y_coord = 0;
int z_coord = 0;
int ttf = 0xffff;
int reply = 0; // for testing serial

int ready = 0;

void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if(!ready){
    digitalWrite(LED_BUILTIN, HIGH);
    // do set-up sequence
    // TODO: move motors to center position using limit switches
    delay(1000); // 1 second for placeholder
    ready = 1;
  }
    while(ready){
      digitalWrite(LED_BUILTIN, LOW);
      while (Serial.available()>=10){
        Serial.readBytes(in_bytes,10);
        if (start_seq == (int)in_bytes[0] && end_seq == (int)in_bytes[9]){
          // a packet has been received so read data
          ttf = ((int)in_bytes[1]<<8 & 0xff00) | ((int)in_bytes[2] & 0xff);
          x_coord = ((int)in_bytes[3]<<8 & 0xff00) | ((int)in_bytes[4] & 0xff);
          y_coord = ((int)in_bytes[5]<<8 & 0xff00) | ((int)in_bytes[6] & 0xff);
          z_coord = ((int)in_bytes[7]<<8 & 0xff00) | ((int)in_bytes[8] & 0xff);
        }
      }
      // convert coordinates to motor angles
      // move motors
      // if ttf
        // fire
        // ready = 0;
    }
}