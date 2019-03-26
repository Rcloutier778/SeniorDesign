#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin =4;
static const int RXpin = 6, TXpin = 7;

static const uint32_t GPSBaud = 9600;
static const double m_to_ft = 3.28084;
int i=0;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gps_ss(RXPin, TXPin);
SoftwareSerial k64(RXpin, TXpin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){;}
  Serial.println("Ready");
  gps_ss.begin(GPSBaud);
  k64.begin(GPSBaud);
}

void loop() {
  float user_lat;
  float user_long;
  String k64_buffer_s;
  char sign;
  double distance_user;
  double angle_user;
  double difference_angle;
  int gps_ran = 0;
  int k64_ran = 0;
  int test = 1;
  // put your main code here, to run repeatedly:
   while (gps_ss.available() > 0){
    if(gps.encode(gps_ss.read())){
      if (gps.location.isUpdated()){
        Serial.print("Latitude= "); 
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude= "); 
        Serial.println(gps.location.lng(), 6);
        gps_ran = 1;
      }
    }
  }
  while(k64.available() > 0){
    sign = k64.read();
    k64_buffer_s = k64.readStringUntil(",");
    if(sign == '-'){
      user_lat = 0 - atof(k64_buffer_s.c_str());
    }
    else{
      user_lat = atof(k64_buffer_s.c_str());
    }
    Serial.println(user_lat,6);
    sign = k64.read();
    k64_buffer_s = Serial.readStringUntil('\n');
    if(sign == '-'){
      user_long = (float) (0 - atof(k64_buffer_s.c_str()));
    }
    else{
      user_long = (float) atof(k64_buffer_s.c_str());
    }
    Serial.println(user_long);
    k64_ran = 1;
  }
  if(test & (Serial.available()>0)){
    sign = Serial.read();
    k64_buffer_s = Serial.readStringUntil(',');
    if(sign == '-'){
      user_lat = 0 - atof(k64_buffer_s.c_str());
    }
    else{
      user_lat = atof(k64_buffer_s.c_str());
    }
    Serial.println(k64_buffer_s);
    Serial.println(user_lat,9);
    sign = Serial.read();
    k64_buffer_s = Serial.readStringUntil('\n');
    if(sign == '-'){
      user_long = (float) (0 - atof(k64_buffer_s.c_str()));
    }
    else{
      user_long = (float) atof(k64_buffer_s.c_str());
    }
    /*if(sign == '-'){
      user_long = 0.00 - k64_buffer_s.toDouble();
    }
    else{
      user_long = k64_buffer_s.toDouble();
    }
    k64_buffer_s = Serial.readStringUntil('\n');
    if(sign == '-'){
      user_long = user_long - k64_buffer_s.toDouble()/1000000.0;
    }
    else{
      user_long = user_long + k64_buffer_s.toDouble()/1000000.0;
    }*/
    Serial.println(k64_buffer_s);
    Serial.println(user_long,6);
  }
  if(gps_ran & k64_ran & gps.location.isUpdated()){
    gps_ran = 0;
    k64_ran = 0;
    distance_user = gps.distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      user_lat,
      user_long) * m_to_ft;
    angle_user = gps.courseTo(
      gps.location.lat(),
      gps.location.lng(),
      user_lat,
      user_long);
    Serial.println(distance_user);
    k64.write(distance_user);
    k64.write("\r\n");
    Serial.println(gps.cardinal(gps.course.deg()));
    Serial.println(gps.cardinal(angle_user));
    difference_angle = gps.course.deg() - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }
    Serial.println(gps.cardinal(difference_angle));
    k64.write(difference_angle);
    k64.write("\r\n");
  }
  delay(2000);
}
