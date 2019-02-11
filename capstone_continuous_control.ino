// Stepper - Version: Latest 
#include <Stepper.h>
#include <Servo.h>

Servo myservo;

char in_bytes[8];   // for incoming serial data
const int start_seq = 36;
const int end_seq = 47;
short v_ang = 0;
short h_ang = 0;
int current_step_v = 0;
int current_step_h = 0;
int v_steps = 0;
int h_steps = 0;
int count_h = 0;
int count_v = 0;
short ttf = 0xffff;
int reply = 0; // for testing serial
bool fireFlag = false;

const int pulPin = 10;
const int dirPin = 11;
const int enblPin = 12;
const int pulPin_v = 7;
const int dirPin_v = 8;
const int enblPin_v = 9;

int dly = 500;

int ready_v = 0;
int ready_h = 0;
int finished = 1;
int state = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  
  myservo.attach(4);
  myservo.write(5);
  
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), h_lim_1, LOW); // pin 2
  attachInterrupt(digitalPinToInterrupt(3), h_lim_2, LOW); // pin 3
  pinMode(pulPin_v, OUTPUT);
  pinMode(dirPin_v, OUTPUT);
  pinMode(enblPin_v, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(18), v_lim_1, LOW); // pin 18
  //attachInterrupt(digitalPinToInterrupt(19), v_lim_2, LOW); // pin 19
  
  digitalWrite(pulPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(pulPin_v, LOW);
  digitalWrite(enblPin_v, LOW);
  digitalWrite(dirPin_v, LOW);
  
  digitalWrite(enblPin, HIGH);
  delay(100);
  digitalWrite(enblPin, LOW);
  digitalWrite(enblPin_v, HIGH);
  delay(100);
  digitalWrite(enblPin_v, LOW);
  delay(2000);
}

void loop() {
  if(state == 0){
    myservo.write(5);
    digitalWrite(dirPin, HIGH);
    digitalWrite(dirPin_v, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    while(!ready_v){
      // move limit switches until a limit switch is hit
      digitalWrite(pulPin_v, HIGH);
      digitalWrite(pulPin_v, LOW);
      delayMicroseconds(dly);
    }
    while(!ready_h){
      // move limit switches until a limit switch is hit
      digitalWrite(pulPin, HIGH);
      digitalWrite(pulPin, LOW);
      delayMicroseconds(dly);
    }
    state = 1;
  }
  
  if(state == 1){
    digitalWrite(LED_BUILTIN, LOW);
    // delay(2000);
    while (Serial.available()>=8){
      Serial.readBytes(in_bytes,8);
      if (start_seq == (short)in_bytes[0] && end_seq == (short)in_bytes[7]){
        // a packet has been received so read data
        //finished = 0;
        state = 2;
        ttf = ((short)in_bytes[1]<<8 & 0xff00) | ((short)in_bytes[2] & 0xff);
        h_ang = ((short)in_bytes[3]<<8 & 0xff00) | ((short)in_bytes[4] & 0xff);
        v_ang = ((short)in_bytes[5]<<8 & 0xff00) | ((short)in_bytes[6] & 0xff);
      }
    }
    h_steps = h_ang*27/9;
    v_steps = v_ang*99.51/9;
    if (ttf == 0){
      fireFlag = true;
      
    }
    else{
      fireFlag = false;
    }
    if(h_steps - current_step_h < 0){
      digitalWrite(dirPin, HIGH); 
    } else{
      digitalWrite(dirPin, LOW);
    }
    if(v_steps - current_step_v < 0){
      digitalWrite(dirPin_v, HIGH); 
    } else{
      digitalWrite(dirPin_v, LOW); 
    }
    h_steps = abs(current_step_h - h_steps);
    v_steps = abs(current_step_v - v_steps);
    current_step_h = h_ang*27/9;
    current_step_v = v_ang*99.51/9;
  }
  
  if(state == 2){
    if(count_h<h_steps){ // if not in position yet
      digitalWrite(pulPin, HIGH);
      digitalWrite(pulPin, LOW);
      count_h++;
    }
    if(count_v<v_steps){ // if not in position yet
      digitalWrite(pulPin_v, HIGH);
      digitalWrite(pulPin_v, LOW);
      count_v++;
    }
    delayMicroseconds(dly);
    if (!(count_h<h_steps) && !(count_v<v_steps)){
      if (fireFlag == true){
        myservo.write(90);
        delay(1000);
        myservo.write(5);
        delay(1000);
      }
      state = 1; // later reset with timer
      ready_v = 0; ready_h = 0; // later reset with time
      count_h = 0; count_v = 0; //later reset with time
    }
  } // end of state 2
} // end of loop()

// Note: limit switches drive pins to GND
void v_lim_1(){
  // 1800 for horiz (60), 2488 for vert (22.5)
  digitalWrite(LED_BUILTIN, LOW);
  ready_v = 1;
  dly = 800;
  digitalWrite(dirPin_v, LOW);
  for(int i = 0; i <= 4422; i++){
    digitalWrite(pulPin_v, HIGH);
    digitalWrite(pulPin_v, LOW);
    delayMicroseconds(dly);
  }
  current_step_v = 0;
}

void v_lim_2(){
  // 1800 for horiz (60), 2488 for vert (22.5)
  digitalWrite(LED_BUILTIN, LOW);
  ready_v = 1;
  dly = 800;
  digitalWrite(dirPin_v, LOW);
  for(int i = 0; i <= 4422; i++){
    digitalWrite(pulPin_v, HIGH);
    digitalWrite(pulPin_v, LOW);
    delayMicroseconds(dly);
  }
  current_step_v = 0;
}

void h_lim_1(){
  //horizontal limit switch has been hit, too far cw
  // 1800 for horiz (60), 2488 for vert (22.5)
  ready_h = 1;
  dly = 800;
  digitalWrite(dirPin, HIGH);
  for(int i = 0; i <= 1800; i++){ // move back to center
    digitalWrite(pulPin, HIGH);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(dly);
  }
  current_step_h = 0;
}

void h_lim_2(){
  // 1800 for horiz (60), 2488 for vert (22.5)
  ready_h = 1;
  dly = 800;
  digitalWrite(dirPin, LOW);
  for(int i = 0; i <= 1800; i++){ // move back to center
    digitalWrite(pulPin, HIGH);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(dly);
  }
  current_step_h = 0;
}