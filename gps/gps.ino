  #include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin =4;
static const uint32_t GPSBaud = 9600;
int i=0;
double prev_lat=200;
double prev_long=200;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup(){
  Serial.begin(9600);
  while(!Serial){;}
  Serial.println("Ready");
  ss.begin(GPSBaud);
}

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  //Serial.println(gps.satellites.value());
  while (ss.available() > 0){
    //Serial.println(gps.satellites.value());
    /*if(i=1000000000000000){
      byte gpsData = ss.read();
      Serial.write(gpsData);
      Serial.println("");
      Serial.println("GPS Data");
      i=0;
    }*/
    if(gps.encode(ss.read())){
      if (gps.location.isUpdated()){
        Serial.print("Latitude= "); 
        Serial.print(gps.location.lat(), 6);
        Serial.print(" Longitude= "); 
        Serial.println(gps.location.lng(), 6);
        Serial.println(prev_lat,6);
        Serial.println(prev_long,6);
        i++;
      }
      if(i>=1 & prev_lat!=200 & prev_long!=200){
        Serial.print("Moving: ");
        Serial.println(gps.cardinal(gps.courseTo(prev_lat, prev_long, gps.location.lat(), gps.location.lng())));
        i=0;
      }
      prev_lat = gps.location.lat();
      prev_long = gps.location.lng();
      
    }
    
  }
}
