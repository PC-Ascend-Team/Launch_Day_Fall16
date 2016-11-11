
/****************************************************
 * Phoenix College Acsend Launch Day Instructions Fall 2016
 *
 * This file contains the code for collecting data from:
 * 
 * +(IMU) - Adafruit 10-DOF IMU Breakout
 * 
 * +(Barometer) - SparkFun Barometric Pressure Sensor Breakout - BMP180
 * +(GPS) - Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - Version 3
 * 
 * Authors: Phoenix College Acsend Team 2015 - 2016
 * 
 * Version 1.2.4
 * 
 * TODO's: 
 *    IMU - Acceleration scale, if we can.
 *    Barometer - Altitude constant
 *  LED outputs.
 *    Sensor errors. - lum, baro, GPS
 *  Finish header comments.
 *    explain output file.
 *  Wiring Guide.
 *  
 *      Move the New line character outside of the GPS 
 *    (If needed) Add a second serial output line per loop iteration.
 *  Real-time status transmissions (Live Reporting):
 *    GPS
 *  Post Processing
 *    Include time stamp in GPS
 *   
 ***************************************************/

/////////////////////////////////////////////////////////
//Includes, (Up to date libraries folder can be downloaded at: https://github.com/PC-Ascend-Team/libraries)
#include <Wire.h>                //IMU, Borometer, GPS

#include <Adafruit_Sensor.h>   //General Adafruit Sensor Library

#include <Adafruit_10DOF.h>    //IMU
#include <Adafruit_LSM303_U.h> //IMU
#include <Adafruit_L3GD20_U.h> //IMU
#include <Adafruit_BMP085_U.h> //IMU
#include <Adafruit_Simple_AHRS.h> //IMU - AHRS conversions

#include <Adafruit_GPS.h>      //GPS
#include <SoftwareSerial.h>    //GPS


//IMU Definitions
/////////////////////////////////////////////////////////
//Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified A   = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   M   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       B   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       G   = Adafruit_L3GD20_Unified(20);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&A, &M);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;



//GPS Definitions
////////////////////////////////////////////////////////
SoftwareSerial mySerial(3, 2); // TX - D3, RX - D2
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false // we do NOT want to echo raw GPS data to serial
boolean usingInterrupt = false;


//Phoenix College Acsend Team variables
////////////////////////////////////////////////////////
int state = 0; //for state machine switch statement
int t = 0;  //time keeper (seconds)

//LEDs
int LD0 = 13; // D13, sysLED

//Funcions Prototypes
 /////////////////////////////////////////////////////////

 // int averageAnalogRead(int pinToRead) - Takes an average of readings on a given pin. Returns the average
 int averageAnalogRead(int pinToRead);

 // void printError(byte error) - Used by the Luminosity sensor to display errors
 void printError(byte error);

 // void useInterrupt(boolean v) - Used by the GPS
 void useInterrupt(boolean);

 // SIGNAL(TIMER0_COMPA_vect) - Used by the GPS
 SIGNAL(TIMER0_COMPA_vect);

 //SETUP
 /////////////////////////////////////////////////////////
 void setup(void){
  Serial.begin(115200); // begin serial conncetion, determined by recomended baud rate for GPS

 //IMU SETUP
 //////////////////////////////////////////////////////
 //Initialise the sensors & check connections
 if(!A.begin()){
   Serial.println(F("No LSM303 detected."));
    // light LED code
 }
 if(!M.begin()){
   Serial.println(F("No LSM303 detected."));
    // light LED code
 }
 if(!B.begin()){
   Serial.print(F("No BMP085 detected."));
    // light LED code
 }
 if(!G.begin()){
   Serial.print(F("No L3GD20 detected."));
    // light LED code
 }
 
 sensor_t sensor;
 // Define sensors
 A.getSensor(&sensor);
 G.getSensor(&sensor);
 M.getSensor(&sensor);
 B.getSensor(&sensor);

 M.enableAutoRange(true); // have mag use auto range

 
 //GPS SETUP
 //////////////////////////////////////////////////////
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
 GPS.sendCommand(PGCMD_ANTENNA);
 useInterrupt(true);

 //Phoenix College Acsend Team SETUP
 //////////////////////////////////////////////////////
 //pinModes
 pinMode(LD0, OUTPUT);
     
 //Output Header
 //////////////////////////////////////////////////////
 //Serial.println(F("Ax,Ay,Az,Mx,My,Mz,Gx,Gy,Gz,Bp,Bt,UVl,UVi,R,G,B,Vl,Il,LUX,T,P,BAlt,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats"));
 Serial.println(F("roll,pitch,yaw,IMUP,IMUT,IMUA,H:M:S.ms,DD/MM/20YY,Fix,FixQ,Lat,Long,LatD,LongD,Speed,Angle,GAlt,Sats"));
 delay(1000);
 
}
/////////////////////////////////////////////////////////
//END SETUP


// GPS STUFF
/////////////////////////////////////////////////////////
// Adafruit put this code here in their GPS parsing example. Im not going to argue with them. ;)
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

 /////////////////////////////////////////////////////////
 // END GPS STUFF 


//LOOP
/////////////////////////////////////////////////////////
 void loop() {

  //Launch Day State Machine
  switch(state) {
    case 0:   //State 00, Waiting State

      state++;
      break;
  
    case 1:   //State 01, Data Collection State

      //digitalWrite(LD0, HIGH); //Light LD0 to indicate start data loging
   
 //IMU OPERATIONS
 //////////////////////////////////////////////////////
 //Get a new sensor event
 //IMU Raw data
 //
 //Raw sensor event data - raw IMU data
 sensors_event_t A_event;
 sensors_event_t M_event;
 sensors_event_t G_event;
 sensors_event_t B_event;
     
 float temperature; //Ambient temperature
   
 //Retrieve sensor data
 A.getEvent(&A_event);
 M.getEvent(&M_event);
 G.getEvent(&G_event);

 //Display Raw Sensor Information: Accel, Mag, Gyro
 Serial.print(A_event.acceleration.x); Serial.print(F(","));
 Serial.print(A_event.acceleration.y); Serial.print(F(","));
 Serial.print(A_event.acceleration.z); Serial.print(F(","));
 Serial.print(M_event.magnetic.x);     Serial.print(F(","));
 Serial.print(M_event.magnetic.y);     Serial.print(F(","));
 Serial.print(M_event.magnetic.z);     Serial.print(F(","));
 Serial.print(G_event.gyro.x);         Serial.print(F(","));
 Serial.print(G_event.gyro.y);         Serial.print(F(","));
 Serial.print(G_event.gyro.z);         Serial.print(F(","));

      //IMU - AHRS Orientation data 
      sensors_vec_t   orientation;   //sensor vector data - calculated IMU-AHRS orientation
      
      //Retrieve IMU sensors orientation data
      if (ahrs.getOrientation(&orientation))

      //Display Sensor Information: Orientation (Roll, Pitch, Heading)
      /* 'orientation' should have valid .roll and .pitch fields */
      Serial.print(orientation.roll);     Serial.print(F(","));
      Serial.print(orientation.pitch);    Serial.print(F(","));
      Serial.print(orientation.heading);  Serial.print(F(","));
       
      //Display Sensor Information: Pressure, Temp, Alt
      B.getEvent(&B_event);
      if (B_event.pressure){
        Serial.print(B_event.pressure);     Serial.print(F(","));
        B.getTemperature(&temperature);
        Serial.print(temperature);          Serial.print(F(","));
      }      
      
  
      //GPS OPERATIONS
      //////////////////////////////////////////////////////
      if (! usingInterrupt) {
        char c = GPS.read();
      }
      
      if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
          return;
      }
       
      Serial.print(GPS.hour, DEC);       Serial.print(F(":")); 
      Serial.print(GPS.minute, DEC);     Serial.print(F(":"));
      Serial.print(GPS.seconds, DEC);    Serial.print(F("."));
      Serial.print(GPS.milliseconds);    Serial.print(F(","));
      Serial.print(GPS.day, DEC);        Serial.print(F("/"));
      Serial.print(GPS.month, DEC);      Serial.print(F("/20"));
      Serial.print(GPS.year, DEC);       Serial.print(F(","));
      Serial.print((int)GPS.fix);        Serial.print(F(","));
      Serial.print((int)GPS.fixquality); Serial.print(F(","));
      //if no GPS fix print commas
      if(!GPS.fix) {
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
                                         Serial.print(F(","));
      }
      if (GPS.fix) {
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);    Serial.print(F(",")); 
        Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);   Serial.print(F(","));
        Serial.print(GPS.latitudeDegrees, 4);                    Serial.print(F(","));
        Serial.print(GPS.longitudeDegrees, 4);                   Serial.print(F(","));
        
        Serial.print(GPS.speed);                                 Serial.print(F(","));
        Serial.print(GPS.angle);                                 Serial.print(F(","));
        Serial.print(GPS.altitude);                              Serial.print(F(","));
        Serial.print((int)GPS.satellites);                       Serial.print(F(","));
      }
  
      Serial.println(F("")); //print new line

      digitalWrite(LD0, LOW); // finished one iteration of data logging

      delay(1000); // delay 1 sec
      //break;
   }
}
/////////////////////////////////////////////////////////
//END LOOP


//FUNCTION DEFINITIONS
//////////////////////////////////////////////////////

//int averageAnalogRead(int pinToRead) - Takes an average of readings on a given pin. Returns the average
//////////////////////////////////////////////////////
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
  runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}


//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SIGNAL(TIMER0_COMPA_vect) - Used by the GPS.
// Adafruit put this code here in their GPS parsing example. Im not going to argue with them. ;)
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

// void useInterrupt(boolean v) - Used by the GPS
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
