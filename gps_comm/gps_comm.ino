#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gps_ss(3, 4);
SoftwareSerial k64(6, 7);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //while(!Serial){;}
  //Serial.println("Ready");
  gps_ss.begin(9600);
  //k64.begin(9600);
}

void loop() {
  static const double m_to_ft = 3.28084;
  float user_lat;
  float user_long;
  String k64_buffer_s;
  char sign;
  double distance_user;
  double angle_user;
  double difference_angle;
  int gps_ran = 0;
  int k64_ran = 0;
  int test = 0;
  int spoofGPS=0;
  double spoofLat=43.085254;
  double spoofLong=-77.678110;
  char writechar[255];
  if(not spoofGPS){
    //k64.listen();
    // Checks for coordinates over serial
    while(Serial.available() > 0){
      // Format to be read {'+/-'LATITUDE,'+/-'LONGITUDE}
      // Gets the sign for lat
      sign = Serial.read();
      // Reads latitude then puts it into a double
      k64_buffer_s = Serial.readStringUntil(',');
      if(sign == '-'){
        user_lat = 0 - atof(k64_buffer_s.c_str());
      }
      else{
        user_lat = atof(k64_buffer_s.c_str());
      }
      //Serial.println(user_lat,6);
      // Gets the sign for longitude
      sign = Serial.read();
      // Reads the longitude then puts it into a double
      k64_buffer_s = Serial.readStringUntil('\n');
      if(sign == '-'){
        user_long = (0 - atof(k64_buffer_s.c_str()));
      }
      else{
        user_long =  atof(k64_buffer_s.c_str());
      }
      //Serial.println(user_long,6);
      // Flags coordinates received from K64
      k64_ran = 1;
    }
  }
  //if(not spoofGPS){
    gps_ss.listen();
    // Checks for gps data
    while(gps_ss.available() > 0){
      // Converts the NMEA string to the gps object
      if(gps.encode(gps_ss.read())){
        // Checks for an updated location
        if (gps.location.isUpdated()){
          // Shows the latitude and longitude
          Serial.print("Latitude= "); 
          Serial.print(gps.location.lat(), 6);
          Serial.print(" Longitude= "); 
          Serial.println(gps.location.lng(), 6);
          // Flags the gps got data
          gps_ran = 1;
        }
      }
    }
  //}
  
  if(k64_ran & spoofGPS){
    // Resets the flags
    //k64_ran = 0;
    // Calculates the distance between cart and user in m(converted to ft)
    distance_user = gps.distanceBetween(
      spoofLat,
      spoofLong,
      user_lat,
      user_long) * m_to_ft;
    // Calculates the angle between cart and user
    angle_user = gps.courseTo(
      spoofLat,
      spoofLong,
      user_lat,
      user_long);
    Serial.print("Distance to user: ");
    
    // Sends the distance to the k64
    dtostrf(distance_user,8,3,writechar);
    k64.write(writechar);
    k64.write((byte)0x00);
    Serial.println(writechar);
    
    Serial.println(gps.cardinal(gps.course.deg()));
    Serial.println(gps.cardinal(angle_user));
    // Calculates the angle between the user and the course of the cart
    difference_angle = gps.course.deg() - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }
    Serial.print("Difference angle: ");
    dtostrf(difference_angle,8,3,writechar);
    k64.write(writechar);
    k64.write((byte)0x00);
    Serial.println(writechar);
    
  }

  if(gps_ran & spoofGPS){
    // Resets the flags
    //k64_ran = 0;
    // Calculates the distance between cart and user in m(converted to ft)
    distance_user = gps.distanceBetween(
      spoofLat,
      spoofLong,
      gps.location.lat(),
      gps.location.lng()) * m_to_ft;
    // Calculates the angle between cart and user
    angle_user = gps.courseTo(
      spoofLat,
      spoofLong,
      gps.location.lat(),
      gps.location.lng());
    Serial.print("Distance to user: ");
    
    // Sends the distance to the k64
    dtostrf(distance_user,8,3,writechar);
    k64.write(writechar);
    k64.write((byte)0x00);
    Serial.println(writechar);
    
    Serial.println(gps.cardinal(gps.course.deg()));
    Serial.println(gps.cardinal(angle_user));
    // Calculates the angle between the user and the course of the cart
    difference_angle = gps.course.deg() - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }
    Serial.print("Difference angle: ");
    dtostrf(difference_angle,8,3,writechar);
    k64.write(writechar);
    k64.write((byte)0x00);
    Serial.println(writechar);
    
  }
  
  // Checks if cart and user data were received and the cart gps data was updated
  if(gps_ran & k64_ran & gps.location.isUpdated()){
    // Resets the flags
    gps_ran = 0;
    k64_ran = 0;
    // Calculates the distance between cart and user in m(converted to ft)
    distance_user = gps.distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      user_lat,
      user_long) * m_to_ft;
    // Calculates the angle between cart and user
    angle_user = gps.courseTo(
      gps.location.lat(),
      gps.location.lng(),
      user_lat,
      user_long);
    //Serial.print("Distance to user: ");

    // Sends the distance to the k64
    dtostrf(distance_user,8,3,writechar);
    //Serial.println(writechar);
    Serial.write(writechar);
    Serial.write((byte)0x00);
    
    //Serial.println(gps.cardinal(gps.course.deg()));
    //Serial.println(gps.cardinal(angle_user));
    // Calculates the angle between the user and the course of the cart
    difference_angle = gps.course.deg() - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }
    //Serial.println("Difference angle: ");
    dtostrf(difference_angle,8,3,writechar);
    //Serial.println(writechar);
    Serial.write(writechar);
    Serial.write((byte)0x00);
  }
  
  // Delays after calculating and reads from serial
  if(test){
    if(Serial.available()>0){
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
      Serial.println(user_long,6);
    }
    delay(2000);
  }
}
