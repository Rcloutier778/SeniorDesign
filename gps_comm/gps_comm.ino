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

  //LED for number of satellites
  pinMode(13,OUTPUT);
}


double user_lat;
double user_long;
double gps_lat;
double gps_long;
double gps_course_deg;

int VERBOSE=0;


void loop() {
  static const double m_to_ft = 3.28084;
  
  double distance_user;
  double angle_user;
  double difference_angle;
  int spoofGPS=1; //spoof gps coord
  int spoofK64=0; //spoof k64 coord
  char writechar[255];
  
  while(1){       
    k64Calc(spoofK64);
    Serial.println("K64 ran. Waiting on gps.");
    gpsCalc(spoofGPS);
    Serial.println("Got GPS, sending to K64");
    // Checks if cart and user data were received and the cart gps data was updated
    // Calculates the distance between cart and user in m(converted to ft)
    distance_user = gps.distanceBetween(gps_lat,gps_long,user_lat,user_long) * m_to_ft;
    
    
    
    
    // Sends the distance to the k64
    dtostrf(distance_user,8,3,writechar);
    k64.write(writechar);
    k64.write((byte)0x00);

    if(VERBOSE==2){
      Serial.print("Distance to user: ");
      Serial.print(writechar);
      Serial.println(" ft");
    }
    

    // Calculates the angle between cart and user
    angle_user = gps.courseTo(gps_lat,gps_long,user_lat,user_long);
    if (angle_user >270.0){
      angle_user -= 180;
    }else{
      angle_user = 90-angle_user;
    }

    /*
    // Calculates the angle between the user and the course of the cart
    difference_angle = gps_course_deg - angle_user;
    if(difference_angle < -180){
      difference_angle += 180;
    }
    else if(difference_angle > 180){
      difference_angle -= 180;
    }
    */
    

    
    dtostrf(angle_user,8,3,writechar);
    // Send angle to k64
    k64.write(writechar);
    k64.write((byte)0x00);

    if(VERBOSE==2){
      Serial.print("Difference angle: ");
      Serial.println(writechar);
    }
    if (VERBOSE){Serial.println();}
  }
}


void k64Calc(int spoofK64){
  if(spoofK64){ //spoof coords
    user_lat = 43.085254;
    user_long = -77.678110;
    Serial.println("!!!Spoofing K64 coordinates!!!");
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
      // Format to be read {LATITUDE,LONGITUDE}
      // Reads latitude then puts it into a double
      user_lat = k64.readStringUntil(',').toDouble();


      // Gets the sign for longitude
      // Reads the longitude then puts it into a double
      user_long = k64.readStringUntil(NULL).toDouble();
      
      if(VERBOSE){
        Serial.print("User latitude: ");
        Serial.println(user_lat,8);
        Serial.print("User longitude: ");
        Serial.println(user_long,8);
      }
    }
  }
}

//Moving avg number
const int movingAvgN = 20;

double gpsLatCum[movingAvgN];
double gpsLongCum[movingAvgN];
double gpsCourseCum[movingAvgN];

int tempMovingAvg=0; //Used to init the moving avg arrays. Only used for the first N vals.

//TODO: use gps.satellites (returns # of visible, participating satellites) 
//      to guage how accurate data is and how large the moving average should be?
//TODO: gps.hdop (horizontal diminution of precision?
//TODO: Thresholding? More info below

void gpsCalc(int spoofGPS){
  //GPS
  double temp_gps_lat;
  double temp_gps_long;
  double temp_gps_course;
  char test_gps;
  
  if(spoofGPS){ //spoofed gps coords
    gps_lat= 43.084514;
    gps_long = -77.678525;
    gps_course_deg = 90.000000;    
    Serial.println("!!!Spoofing GPS coordinates!!!");
  }else{ //actual code
    //Make software serial listen to gps if it's not
    if(not gps_ss.isListening()){
      gps_ss.listen();
    }

    // Checks for gps data
    while(1){
      while(gps_ss.available() > 0){
        test_gps = gps_ss.read();
        //Serial.print(test_gps);
        gps.encode(test_gps);
      }
      
      //Serial.println(gps.location.lat());
      //Serial.println(gps.location.lng());
      
      //Serial.println("In GPS Loop");
      // Converts the NMEA string to the gps object
      
      
      // Checks for an updated location
      if(gps.location.isUpdated()){
        //Turn off no-satellite LED indicator
        digitalWrite(13,LOW);
        
        // Shows the latitude and longitude
        temp_gps_lat = gps.location.lat();
        temp_gps_long = gps.location.lng();
        temp_gps_course = gps.course.deg();

        if (tempMovingAvg<movingAvgN){
          gpsLatCum[tempMovingAvg] = temp_gps_lat;
          gpsLongCum[tempMovingAvg] = temp_gps_long;
          gpsCourseCum[tempMovingAvg] = temp_gps_course;
          tempMovingAvg +=1;
          gps_lat = temp_gps_lat;
          gps_long = temp_gps_long;
          gps_course_deg = temp_gps_course;
        }
        else{
          for (int k = 0; k < movingAvgN-1; k++){
            gpsLatCum[k] = gpsLatCum[k+1];
            gpsLongCum[k] = gpsLongCum[k+1];
            gpsCourseCum[k] = gpsCourseCum[k+1];
          }

          //Thresholding
          //If the distance between the last mv avg point and new point is above X ft, reduce?
         

          
          gpsLatCum[movingAvgN-1] = temp_gps_lat;
          gpsLongCum[movingAvgN-1] = temp_gps_long;
          gpsCourseCum[movingAvgN-1] = temp_gps_course;

          gps_lat=0.0f;
          gps_long=0.0f;
          gps_course_deg = 0.0f;
          for (int k = 0; k < movingAvgN; k++){
            gps_lat += gpsLatCum[k];
            gps_long += gpsLongCum[k];
            gps_course_deg += gpsCourseCum[k];
            
          }
          gps_lat /= movingAvgN;
          gps_long /= movingAvgN;
          gps_course_deg /= movingAvgN;
        }
        if(VERBOSE){
          Serial.print("GPS latitude: ");
          Serial.println(gps_lat,6);
          Serial.print("GPS longitude: ");
          Serial.println(gps_long,6);
          Serial.print("GPS Course degree: ");
          Serial.println(gps_course_deg,6);
        }
        return;
      }
      //DO NOT PUT A DELAY HERE
      //It'll mess up the aquisition of NMEA data from module cause it'll max out the uart buffer and mess up the checksum
    }
  }
}
