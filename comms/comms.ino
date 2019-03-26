#include <SoftwareSerial.h>

static const int RXpin = 6, TXpin = 7;

SoftwareSerial k64(RXpin, TXpin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  k64.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  char k64_buffer;
  char arduino_string[15];
  String k64_buffer_s;
  int ran = 0;
  
  while(k64.available() > 0){
    //Serial.println(k64.available());
    k64_buffer_s = k64.readStringUntil("\n");
    //k64_buffer = k64.read();
    //k64.flush();
    Serial.print("Arduino Received: ");
    Serial.println(k64_buffer_s);
    //Serial.println(k64_buffer);
    //if(k64_buffer){
    if(k64_buffer_s){
      strcpy(arduino_string,"Received\r\n");
    }
    else{
      strcpy(arduino_string,"Not Received\r\n");
    }
    ran = 1;
  }
  if(ran){
    Serial.print("Arduino Sending: ");
    Serial.println(arduino_string);
    k64_buffer = char(0);
    k64_buffer_s = "";
    k64.write(arduino_string);
    ran = 0;
  }
  //delay(1000);
  //k64.write("send");
  //Serial.println(k64.read());
  
  /*
  while(Serial.available() > 0){
    k64_buffer = Serial.read();
    //k64_buffer_s = Serial.readStringUntil("\n");
    Serial.print("Arduino Received: ");
    //Serial.println(k64_buffer_s);
    Serial.println(k64_buffer);
    if(k64_buffer){
    //if(k64_buffer_s){
      strcpy(arduino_string,"Received\n");
    }
    else{
      strcpy(arduino_string,"Not Received\n");
    }
    Serial.print("Arduino Sending: ");
    Serial.println(arduino_string);
    Serial.write(arduino_string);
  }
  k64_buffer ="";*/
}
