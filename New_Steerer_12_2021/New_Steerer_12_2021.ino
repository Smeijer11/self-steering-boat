#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <CSV_Parser.h>
#include <math.h>
#include "Filter.h"

//this n is used as a flag for the CSV reading in the loop
int n = 0;
/* This driver reads raw data from the BNO055, Potentiometer and GPS

   Connections for ADALOGGER
   ===========

   Potentiometer
   _____________
   Connect one side to VCC
   Connect other to common ground
   Connect Middle to A0

   GPS
   _____________
   Connect VIN to VCC
   Connect GROUND to common ground
   Connect GPS TX to RX1 (D0)
   Connect GPS RX to TX1 (D1)

*/

/*********************************************/

/*********************************************/
// Sample delay
const float SAMPLERATE = 10;
// Setup for the differentiation variables
double psierror = 0;
double dt = 0;
double dpsierror_dt = 0;
double previouspsierror = 0;

// timing
int previousMillis = 0;

// PD coefficients
double Kp = 10;
double Kd = 7;

// these define the shape of the arm etc.
float r_rud = 4; //in
float r_arm = 1.9; //in

//Start point, end point and circle enclosure radius ( for testng in warmup area, we want 27.37271944, -82.45137778 at start and 27.36653111, -82.45135278 at end)
double x1 = -71.123505;
double y11 = 42.368745;
double x2 =  -71.126872;
double y2 = 42.373255;
double radius = 0.001;

/*double x1 = -82.4530766667;
  double y11 = 27.376095;
  double x2 = -82.4530116667;
  double y2 = 27.3585083333;
  double radius = 0.001;*/

double m = (y2 - y11) / (x2 - x1); //Gradient of the desired path
double param_g = y11 - (m*x1);
double param_a = 1 + pow(m, 2);
double longitude_los = 0;
double latitude_los = 0;
double desired_heading = 0;

//double length_lat = 111080.41924120882; //number of meters in a degree of latitude at BOSTON (from http://www.csgnetwork.com/degreelenllavcalc.html)
//double length_long = 82370.68369312544; //number of meters in a degree of longitude at BOSTON

double pi = 3.14159265358979323846; //pi
double d = 0; // distance from current location to line of sight point on course


//Setup an instance of the servo
Servo servo;
double pos = 0;

// this variable allows us to count how many times through the loop it has gone.
int count = 0;

ExponentialFilter<float> FilteredGPSsin(10, 0);
double  GPS_sin_filt = 0;

ExponentialFilter<float> FilteredGPScos(10, 0);
double  GPS_cos_filt = 0;

ExponentialFilter<float> Filtereddesiredheading(9, 0);
double  desiredheading_filt = 0;


/*********************************************/
/*GPS setup, variables and definitions*/
/*********************************************
*/
// Setup for using hardware logFile
#define GPSSerial Serial1
#define GPSECHO  false

//Connect to the GPS
Adafruit_GPS GPS(&GPSSerial);

//SD card setup

#define cardSelect 4
File logFile;

//setup for timing
int k = 0;



/*********************************************/
/*
  Error logging
  red (pin 13)
  Error 1: No BNO055 detected
  Error 2: no SD card
  Error 3: Failed to create file
  green  (pin 8)
  Continuous on: not calibrated
*/
/*********************************************/


/*-------------------------------------------*/


void setup(void)
{

  /*********************************************/
  /*Arduino setup*/
  /*********************************************/
  pinMode(8, OUTPUT);
  Serial.begin(115200);


  // Set up the servo

  servo.attach(5);

  /*********************************************/
  /*GPS setup (taken from GPS_HardwareSerial_Parsing.ino)*/
  /*********************************************/

  // Initialise the GPS sensor
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  delay(2000);


  /*********************************************/
  /*SD setup (taken from GPS_HardwareSerial_Parsing.ino)*/
  /*********************************************/

  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.print("err2");
    error(2);

  }

  char filename[15];
  strcpy(filename, "GPSlog00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    //error(3);



  }
  //start the file
  print_headers();


}



/*--------------------------------------------*/

void loop(void)
{
  //-------------------------------------------------------------//
  //Read in waypoint values from CSV file TODO: LINK THIS TO THE REST OF THE CODE! UNUSED CURRENTLY
  //-------------------------------------------------------------//
  if (n < 1){
  CSV_Parser cp(/*format*/ "ff", /*has_header*/ false, /*delimiter*/ ',');

  // The line below (readSDfile) wouldn't work if SD.begin wasn't called before.
  // readSDfile can be used as conditional, it returns 'false' if the file does not exist.
  if (cp.readSDfile("testfile.csv")) {
    float *longitudes = (float*)cp[0];
    float *latitudes = (float*)cp[1];

    if (longitudes && latitudes) {
      ++n;
    } else {
      //in this case there is something wrong with the CSV
      //error(5);
    }
  }
  }
  //-------------------------------------------------------------//
  //GPS read
  //-------------------------------------------------------------//
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another

    //------------------------------------------------------------//
    //Timing
    //------------------------------------------------------------//
    //This starts the timing for the loop, after the GPS read, as that can hang., so this is most accurate.
    int currentMillis = millis();


    //------------------------------------------------------------//
    //   Heading smoothing
    //------------------------------------------------------------//
    // convert the DD MM.MMM to DD.DDD for easier data processing.
    double GPS_longitude = -convertDegMinToDecDeg (GPS.longitude);
    double GPS_latitude = convertDegMinToDecDeg (GPS.latitude);

    //Filter the heading data from the GPS using recursive filter.
    //Its not trivial to average circular values, you must take the sine and cosine of the values, and filter those.
    double GPS_angle_rad = GPS.angle * (PI / 180);
    double sinGps = sin(GPS_angle_rad);
    double cosGps = cos(GPS_angle_rad);

    //Here is where the filtering actually happens.
    FilteredGPSsin.Filter(sinGps);
    GPS_sin_filt = FilteredGPSsin.Current();

    FilteredGPScos.Filter(cosGps);
    GPS_cos_filt = FilteredGPScos.Current();

    //Here we recombine the sine and cosine to a useful angle, but its stil in radians
    double GPS_angle_filt = atan2(GPS_sin_filt, GPS_cos_filt);
    GPS_angle_filt = GPS_angle_filt * (180 / PI);
    if (GPS_angle_filt < 0) {
      GPS_angle_filt += 360;
    }


    //------------------------------------------------------------//
    //Guidance controller
    //------------------------------------------------------------//
    //Now we calculate where to point.
    double param_b = 2 * (m * param_g - m * GPS_latitude - GPS_longitude);
    double param_c = pow(GPS_longitude, 2) + pow(GPS_latitude, 2) + pow(param_g, 2) - (2 * param_g * GPS_latitude) - pow(radius, 2);
    // this is based on the results shown in Fossen handbook
    if (abs(x2 - x1) > 0) {
      if (x2 - x1 > 0) {
        longitude_los = (-param_b + sqrt(pow(param_b, 2) - (4 * param_a * param_c))) / (2 * param_a);
      }
      if (x2 - x1 < 0) {
        longitude_los = (-param_b - sqrt(pow(param_b, 2) - (4 * param_a * param_c))) / (2 * param_a);
      }
      latitude_los = m * (longitude_los - x1) + y11;
    }
    if (x2 - x1 == 0) {
      longitude_los = x1;
      if (y2 - y11 > 0) {
        latitude_los = GPS_latitude + sqrt(pow(radius, 2) - pow((longitude_los - GPS_longitude), 2));
      }
      if (y2 - y11 < 0) {
        latitude_los = GPS_latitude - sqrt(pow(radius, 2) - pow((longitude_los - GPS_longitude), 2));
      }
    }

    //------------------------------------------------------------//
    // heading controller
    //------------------------------------------------------------//


    // THIS IS THE FULL DESIRED HEADING note the length long and length lat. This is from
    //http://www.csgnetwork.com/degreelenllavcalc.html, and is used to normalise the axes
    //so that an angle can be correctly calculated between the current position and the next waypoint.
    //desired_heading = ((180/PI)*atan2(((length_long/length_lat)*(longitude_los - GPS_longitude)),(latitude_los - GPS_latitude)));


    //distance between two points from  (http://edwilliams.org/avform147.htm)
    d = 2 * asin(sqrt(pow(sin((GPS_latitude - latitude_los) / 2), 2) + pow(cos(GPS_latitude) * cos(latitude_los) * sin((GPS_longitude - longitude_los) / 2), 2)));

    //course between points from (http://edwilliams.org/avform147.htm)
    if (cos(GPS_latitude) < 0.0000001) {
      if (GPS_latitude > 0) {
        desired_heading = 180;
      }
      else desired_heading = 0;
    }
    else if (longitude_los - GPS_longitude < 0) {
      desired_heading = acos((sin(latitude_los) - sin(GPS_latitude) * cos(d) / sin(d) * cos(GPS_latitude)));
    }
    else {
      desired_heading = 2 * pi - acos((sin(latitude_los) - sin(GPS_latitude) * cos(d) / sin(d) * cos(GPS_latitude)));
    }



    // due to the quantisation of the gps data, this data is noisy. To fix that, we filter.
    Filtereddesiredheading.Filter(desired_heading);
    desiredheading_filt = Filtereddesiredheading.Current();

    // set up the values for the pid
    // calculate the error in heading
    psierror = fmod((desiredheading_filt - GPS_angle_filt + 540), 360) - 180;
    //calculate the time step
    dt = (millis() - previousMillis) * 0.001;
    //calculate the finite difference (derivative) of the error
    dpsierror_dt = (psierror - previouspsierror) / dt;
    // set the error to be the previous error for the derivative thing
    previouspsierror = psierror;

    // now set the rudder position
    pos = (Kp * psierror) + (Kd * dpsierror_dt);

    //------------------------------------------------------------//
    //SErvo control
    //------------------------------------------------------------//
    // and limit the rudder travel to 20degrees
    if (isnan(pos)) {
      pos = 0;
    }
    if (pos > 20.0) {
      pos = 20.0;
    }
    if (pos < -20.0) {
      pos = -20.0;
    }

    // now send the rudder position to the rudder (NB the minus, this is because the rudder movement is flipped)
    servo.write(((r_rud / r_arm) * -pos));


    //------------------------------------------------------------//
    //Logging
    //------------------------------------------------------------//

    logFile.print(currentMillis - previousMillis);
    logFile.print(",");
    //if we've waited long enough, i.e. force to operate at 10Hz
    if (currentMillis - previousMillis > SAMPLERATE) {
      //reset the times
      previousMillis = currentMillis;
      count++;
      //print all the data
      logFile.print(currentMillis);
      logFile.print(",");
      logFile.print(pos, 8);
      logFile.print(",");
      logFile.print(GPS.speed, 8);
      logFile.print(",");
      logFile.print(GPS.angle, 8);
      logFile.print(",");
      logFile.print(GPS_angle_filt, 8);
      logFile.print(",");
      logFile.print(GPS_latitude, 8);
      logFile.print(",");
      logFile.print(GPS_longitude, 8);
      logFile.print(",");
      logFile.print(GPS.latitude, 8);
      logFile.print(",");
      logFile.print(GPS.longitude, 8);
      logFile.print(",");
      logFile.print(GPS.hour, DEC); logFile.print(':');
      logFile.print(GPS.minute, DEC); logFile.print(':');
      logFile.print(GPS.seconds, DEC); logFile.print('.');
      logFile.print(GPS.milliseconds);
      logFile.print(",");
      logFile.print((int)GPS.fix);
      logFile.print(",");
      logFile.print(desired_heading, 8);
      logFile.print(",");
      logFile.print(desiredheading_filt, 8);
      logFile.print(",");
      logFile.print(psierror, 8);
      logFile.print(",");
      logFile.print(longitude_los, 8);
      logFile.print(",");
      logFile.print(latitude_los, 8);
      logFile.print("\n");

      logFile.flush();

    }
  }
}


void error(uint8_t errno) {
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(8, HIGH);
      delay(100);
      digitalWrite(8, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

void print_headers(void) {

  logFile.print("Loop_Time,");
  logFile.print("ucontrollor_time,");
  logFile.print("Rudder_Position,");
  logFile.print("GPS_Speed/knots,");
  logFile.print("GPS_Angle,");
  logFile.print("GPS_Angle_exponential_filt,");
  logFile.print("GPS_Lat_dec,");
  logFile.print("GPS_Long_dec,");
  logFile.print("GPS_Lat,");
  logFile.print("GPS_Long,");
  logFile.print("Time_UTC,");
  logFile.print("Fix_Quality,");
  logFile.print("Desired_Heading,");
  logFile.print("Desired_Heading_exponential_filt,");
  logFile.print("heading_error,");
  logFile.print("longitude_los,");
  logFile.print("latitude_los\n");
}

// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}
