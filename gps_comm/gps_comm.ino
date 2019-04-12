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
  while(!Serial){;}
  Serial.println("Ready");
  gps_ss.begin(9600);
  k64.begin(9600);
}
float user_lat;
float user_long;
float gps_lat;
float gps_long;
float gps_course_deg;



void loop() {
  static const double m_to_ft = 3.28084;
  
  double distance_user;
  double angle_user;
  double difference_angle;
  int spoofGPS=0; //spoof gps coord
  int spoofK64=0; //spoof k64 coord
  char writechar[255];

  //Setup software serial stuff
  //setup();
  
  while(1){
    k64Calc(spoofK64);
    Serial.println("K64 ran. Waiting on gps.");
    gpsCalc(spoofGPS);
    
    // Checks if cart and user data were received and the cart gps data was updated
    // Calculates the distance between cart and user in m(converted to ft)
    distance_user = gps.distanceBetween(gps_lat,gps_long,user_lat,user_long) * m_to_ft;
    
    // Calculates the angle between cart and user
    angle_user = gps.courseTo(gps_lat,gps_long,user_lat,user_long);
      
    // Sends the distance to the k64
    k64.write(writechar);
    k64.write((byte)0x00);

    Serial.print("Distance to user: ");
    dtostrf(distance_user,8,3,writechar);
    Serial.print(writechar);
    Serial.println(" ft");

    // Calculates the angle between the user and the course of the cart
    difference_angle = gps_course_deg - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }

    // Send angle to k64
    k64.write(writechar);
    k64.write((byte)0x00);

    Serial.print("Difference angle: ");
    dtostrf(difference_angle,8,3,writechar);
    Serial.println(writechar);

    Serial.println();
  }

  
}


void k64Calc(int spoofK64){
  char sign;
  String k64_buffer_s;
  if(spoofK64){ //spoof coords
    user_lat = 43.085254;
    user_long = -77.678110;
  }else{ //Actual code
    //Make software serial listen to k64 if it's not
    if (not k64.isListening()){
      k64.listen();
    }
    // Checks for coordinates over serial
    while(k64.available()<=0){
      delay(100);
    }
    
    while(k64.available() > 0){
      //Serial.println("Got k64");
      // Format to be read {'+/-'LATITUDE,'+/-'LONGITUDE}
      // Gets the sign for lat
      sign = k64.read();
      // Reads latitude then puts it into a double
      k64_buffer_s = k64.readStringUntil(',');
      if(sign == '-'){
        user_lat = 0 - atof(k64_buffer_s.c_str());
      }
      else{
        user_lat = atof(k64_buffer_s.c_str());
      }
      // Gets the sign for longitude
      sign = k64.read();
      // Reads the longitude then puts it into a double
      k64_buffer_s = k64.readStringUntil('\n');
      if(sign == '-'){
        user_long = (0 - atof(k64_buffer_s.c_str()));
      }
      else{
        user_long =  atof(k64_buffer_s.c_str());
      }
      Serial.print("User latitude: ");
      Serial.println(user_lat,6);
      Serial.print("User longitude: ");
      Serial.println(user_long,6);
    }
  }
}


const int movingAvgN = 20;
float gpsLatCum[movingAvgN];
float gpsLongCum[movingAvgN];

int tempMovingAvg=0;

void gpsCalc(int spoofGPS){
  //GPS
  float temp_gps_lat;
  float temp_gps_long;
  int gpsEncoded=0;
  
  if(spoofGPS){ //spoofed gps coords
    gps_lat=50.000000;
    gps_long = -80.000000;
    gps_course_deg = 90.000000;    
  }else{ //actual code
    //Make software serial listen to gps if it's not
    if(not gps_ss.isListening()){
      gps_ss.listen();
    }

    // Checks for gps data
    while(1){
      while(gps_ss.available() > 0){
        gps.encode(gps_ss.read());
      }
      //Serial.println("In GPS Loop");
      // Converts the NMEA string to the gps object
      
      
      // Checks for an updated location
      if(gps.location.isUpdated()){
        
        // Shows the latitude and longitude
        temp_gps_lat = gps.location.lat();
        temp_gps_long = gps.location.lng();

        /*if (gpsLatCum[0]==0.00){
          gps_lat = temp_gps_lat;
          gps_long = temp_gps_long;
          for (int ii=0;ii<movingAvgN;ii++){
            gpsLatCum[ii] = gps_lat;
            gpsLongCum[ii] = gps_long;
          }
        }*/
        if (tempMovingAvg<movingAvgN){
          gpsLatCum[tempMovingAvg] = temp_gps_lat;
          gpsLongCum[tempMovingAvg] = temp_gps_long;
          tempMovingAvg +=1;
          gps_lat = temp_gps_lat;
          gps_long = temp_gps_long;
        }
        else{
          for (int k = 0; k < movingAvgN-1; k++){
            gpsLatCum[k] = gpsLatCum[k+1];
            gpsLongCum[k] = gpsLongCum[k+1];
          }

          //Thresholding
         

          
          gpsLatCum[movingAvgN-1] = temp_gps_lat;
          gpsLongCum[movingAvgN-1] = temp_gps_long;

          gps_lat=0.0f;
          gps_long=0.0f;
          for (int k = 0; k < movingAvgN; k++){
            gps_lat += gpsLatCum[k];
            gps_long += gpsLongCum[k];
          }
          Serial.println(gps_lat);
          Serial.println(movingAvgN);
          gps_lat /= movingAvgN;
          gps_long /= movingAvgN;
        }


        
  
        Serial.print("GPS latitude: ");
        Serial.println(gps_lat,6);
        Serial.print("GPS longitude: ");
        Serial.println(gps_long,6);
        //TODO: Gps.course.deg()?
        return;
      }
      delay(100);
    }
  }
}
