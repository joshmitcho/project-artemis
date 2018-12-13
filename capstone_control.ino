// Stepper - Version: Latest 
#include <Stepper.h>

char incomingBytes[2];   // for incoming serial data
int out_data = 1;
int x_data = 0;
int y_data = 0;
int z_data = 0;
int t_fire = 0xffff;
int t_remaining = 0xffff;

int ready = 0;
int finished = 0;

void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  // do set-up sequence
  // TODO: move motors to center position using limit switches
  Serial.println("start set-up");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000); // 1 second just for fun
  digitalWrite(LED_BUILTIN, LOW);
  ready = 1;
  finished = 0;
        // send data only when you receive data:
        if(ready){
          Serial.println("ready");
          ready = 0;
          if (Serial.available() > 0) {
            while(!finished){
              Serial.println("start of while loop");
              // read the incoming byte:
              Serial.readBytes(incomingBytes,2);
              out_data = (int)*incomingBytes;
              if(out_data == 0){ // start of a data packet (do we need??)
                Serial.println("start of data");
                Serial.readBytes(incomingBytes,2);
                x_data = (int)*incomingBytes;
                Serial.readBytes(incomingBytes,2);
                y_data = (int)*incomingBytes;
                Serial.readBytes(incomingBytes,2);
                z_data = (int)*incomingBytes;
                Serial.readBytes(incomingBytes,2);
                t_fire = (int)*incomingBytes;
                // TODO: check data. need to determine bounds for the robot
                // maybe some sort of checksum
                
                // convert x,y,z coordinates to angles (Robotics)
                // check angle after calculations to see if it is physically possible
                if(t_remaining==0xffff && t_fire!=0xffff){
                  //t_remaining = t_fire;
                  t_remaining = 10;
                } else { t_remaining = t_remaining-1;}
                // Some timing shit
                if(t_remaining == 0){
                  // fire a projectile
                  // TODO: control servo motor
                  finished = 1;
                }
              }
                // say what you got:
                Serial.print("x_data: ");
                Serial.println(x_data, DEC);
                Serial.print("y_data: ");
                Serial.println(y_data, DEC);
                Serial.print("z_data: ");
                Serial.println(z_data, DEC);
                Serial.print("t_remaining: ");
                Serial.println(t_remaining, DEC);
                finished = 1;
            }
          }
        }
}