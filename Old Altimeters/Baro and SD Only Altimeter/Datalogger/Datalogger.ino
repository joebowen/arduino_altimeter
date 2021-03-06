/*
  Flexible Field-Proof Arduino Model Rocket Altimeter

  Author: Joe Bowen
  Date: 12/18/15

  Resources used:
    - http://home.earthlink.net/~david.schultz/rnd/2004/KalmanApogeeII.pdf
    - Adafruit demo for LIS3DH
    - SD Card Datalogger example by Tom Igoe
    - bmp280test.ino
      - Written by Limor Fried & Kevin Townsend for Adafruit Industries.
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <SD.h>               // SD Card module

#include <Adafruit_LIS3DH.h>  // LIS3DH Accel sensor
#include <Adafruit_BMP280.h>  // BMP280 Baro sensor

// Chip select pins for sensors and sd card.
#define BMP_CS 3              // Pin Number
#define SD_CS 4               // Pin Number
#define LIS3DH_CS 6           // Pin Number

// Buzzer pin
#define BUZZER_PIN 5          // Pin Number

// Pin numbers for pyro charges.
#define PYRO1 9               // Pin Number
#define PYRO2 10              // Pin Number

// Duration of pyro charge.
#define PYRO_DURATION 10000    // milliseconds

// Flight start detection settings.
// Values must be greater than all three settings
// to trigger flight start.
#define TRIGGER_ALT 10        // meters
#define TRIGGER_VEL 10        // meters/second
#define TRIGGER_TIME 500      // milliseconds

// Velocity limit to trigger apogee detection
#define APOGEE_VEL 10         // meters/second

// Main (pyro2) altitude limit
#define MAIN_ALT 150          // meters

// Flight end detection height
#define GROUND_ALT 25         // meters

// Set this to the maximum working altitude of the 
// baro sensor.
#define MAX_BARO_ALT 9000     // meters  

// Speed from the accelerometer above which locks 
// out deployment of drogue due to suspected mach effects.
#define MACH_LOCK 100         // meters/second

// Set these to enable/disable features.
// This is useful for reducing flash and ram 
// usage on smaller processors.
#define USE_SD true          // Use SD Card Module?
#define USE_LIS3DH false      // Use LIS3DH accel module?

// Accelerometer smoothing variable for the exponential decaying moving average.
#define SMOOTHING .02         

// Newline defaults
#define NEWLINE true
#define NONEWLINE false

// State machine states
#define S_INIT 0
#define S_FLIGHTSTARTED 1
#define S_WAITFORPYRO2 2
#define S_WAITFORGROUND 3
#define S_FINISHED 4

// Altimeter state variables
int state = S_INIT;           // Current state
unsigned long ts1;            // Milliseconds from first pyro
unsigned long ts2;            // Milliseconds from second pyro
bool fireEvent1 = false;      // First pyro event flag
bool fireEvent2 = false;      // Second pyro event flag
unsigned long flight_num = 0; // Flight Number
unsigned long prevMilli;      // Previous measurement time (milliseconds)
double prevAlt;               // Previous altitude measurement (meters)
double startAlt;              // Initial altitude measurement (meters)

double avgXaccel;             // Average X acceleration (m/s^2)
double avgYaccel;             // Average Y acceleration (m/s^2)
double avgZaccel;             // Average Z acceleration (m/s^2)      

double xAccelAdj = 0;         // Adjustment factor for gravity in the X axis
double yAccelAdj = 0;         // Adjustment factor for gravity in the Y axis
double zAccelAdj = 0;         // Adjustment factor for gravity in the Z axis

double xAccelVel = 0;         // X Velocity based on X Accelerometer (m/s)
double yAccelVel = 0;         // Y Velocity based on Y Accelerometer (m/s)
double zAccelVel = 0;         // Z Velocity based on Z Accelerometer (m/s)  

double prevXaccel = 0;        // Previous X acceleration value (m/s^2)
double prevYaccel = 0;        // Previous Y acceleration value (m/s^2)
double prevZaccel = 0;        // Previous Z acceleration value (m/s^2)

// Sensor Initalization
Adafruit_BMP280 bme(BMP_CS);
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

void setup()
{   
  // see if the SD card is present and can be initialized
  if (USE_SD) {
    if (!SD.begin(SD_CS)) {
      failure_mode("Write failure");
    }
  }
  
  find_flight_num();
  
  // see if the Baro/Temp sensor is present and can be initialized
  if (!bme.begin()) {
    failure_mode("BMP failure");
  }

  if (USE_LIS3DH) {
    // see if the Accelerometer sensor is present and can be initialized  
    if (!lis.begin(0x18)) {
      failure_mode("Accel failure");
    }
  
    // Inialize Accelerometer to +/-16G range
    lis.setRange(LIS3DH_RANGE_16_G); 
  }
  
  bool ret1 = write_to_sd("time(ms),temperature(C),pressure,", NONEWLINE);
  bool ret2 = write_to_sd("absolute altitude(m)(probably wrong),altitude(m),raw velocity(m/s),", NEWLINE);
  // bool ret3 = write_to_sd("Raw X Accel(m/s^2),Raw Y Accel(m/s^2),Raw Z Accel(m/s^2),Vector Accel(m/s^2)", NEWLINE);

  if (!ret1 && !ret2)
  {
    failure_mode("Write failure");
  }
  
  // Need to first read temp and pressure to initialize readAltitude.
  bme.readTemperature();
  bme.readPressure();

  // Get initial altitude measurement.
  startAlt = bme.readAltitude(1013.25);
  prevAlt = startAlt;

  // Initialize clock.
  prevMilli = millis();

  // Initialize initial acceleration moving averages
  if (USE_LIS3DH) {
    sensors_event_t event; 
    lis.getEvent(&event);

    avgXaccel = event.acceleration.x;
    avgYaccel = event.acceleration.y;
    avgZaccel = event.acceleration.z;
  }
}

void loop()
{  
  double temp = bme.readTemperature();
  long int pressure = bme.readPressure();
  double rawAlt = bme.readAltitude(1013.25);
  
  unsigned long curMilli = millis();
  
  double rawVel = (rawAlt - prevAlt) / ((double)(curMilli - prevMilli) / 1000);
  
  double relAlt = rawAlt - startAlt;
  
  prevAlt = rawAlt;
  prevMilli = curMilli;
 
  write_to_sd(String(curMilli) + "," + String(temp) + ",", NONEWLINE);
  write_to_sd(String(pressure) + "," + String(rawAlt) + ",", NONEWLINE);
  write_to_sd(String(relAlt) + "," + String(rawVel) + ",", NONEWLINE);

  double vec_accel = 0;
  double vecAccelVel = 0;

  if (USE_LIS3DH) {
    sensors_event_t event; 
    lis.getEvent(&event);

    // Exponentially decaying moving average
    // The filter uses a fairly small SMOOTHING variable to allow the filter
    // to retain the average gravity a few cycles after launch when the launch code
    // finally detects a launch and saves these values to be used in offsets.
    avgXaccel = SMOOTHING * event.acceleration.x + (1.0 - SMOOTHING) * avgXaccel;
    avgYaccel = SMOOTHING * event.acceleration.y + (1.0 - SMOOTHING) * avgYaccel;
    avgZaccel = SMOOTHING * event.acceleration.z + (1.0 - SMOOTHING) * avgZaccel;

    write_to_sd(String(avgXaccel) + ",", NONEWLINE);
    write_to_sd(String(avgYaccel) + ",", NONEWLINE);
    write_to_sd(String(avgZaccel) + ",", NONEWLINE);
    
    write_to_sd(String(event.acceleration.x) + ",", NONEWLINE);
    write_to_sd(String(event.acceleration.y) + ",", NONEWLINE);
    write_to_sd(String(event.acceleration.z) + ",", NONEWLINE);
  
    vec_accel = abs_vector(event.acceleration.x - xAccelAdj, event.acceleration.y - yAccelAdj, event.acceleration.z - zAccelAdj);

    write_to_sd(String(vec_accel), NONEWLINE);

    // Integrate the accelerometer data to generate velocity values.
    xAccelVel = xAccelVel + (((double)(curMilli - prevMilli) / 1000) * (prevXaccel + event.acceleration.x - xAccelAdj) / 2);
    yAccelVel = yAccelVel + (((double)(curMilli - prevMilli) / 1000) * (prevYaccel + event.acceleration.y - yAccelAdj) / 2);
    zAccelVel = zAccelVel + (((double)(curMilli - prevMilli) / 1000) * (prevZaccel + event.acceleration.z - zAccelAdj) / 2);
    
    prevXaccel = event.acceleration.x - xAccelAdj;
    prevYaccel = event.acceleration.y - yAccelAdj;
    prevZaccel = event.acceleration.z - zAccelAdj;

    vecAccelVel = abs_vector(xAccelVel, yAccelVel, zAccelVel);

    write_to_sd(String(vecAccelVel), NONEWLINE);
  }
  
  altimeter_state_tree(curMilli, vecAccelVel, rawVel, relAlt);

  write_to_sd(" ", NEWLINE);
}

double abs_vector(double accel_x, double accel_y, double accel_z)
{
  return sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
}

int altimeter_state_tree(unsigned long time, double accelVel, double vel, double alt)
{
  switch (state)
  {
    case S_INIT:
      if (time > TRIGGER_TIME)
      {
        if (vel > TRIGGER_VEL)
        {
          if (alt > TRIGGER_ALT)
          {
            xAccelAdj = avgXaccel;
            yAccelAdj = avgYaccel;
            zAccelAdj = avgZaccel;
            
            state = S_FLIGHTSTARTED;
          }
        }
      }
      
      break;
    
    case S_FLIGHTSTARTED:
      if (vel < APOGEE_VEL && accelVel < MACH_LOCK)
      {
        fire_pyro(PYRO1);
        state = S_WAITFORPYRO2;
        ts1 = time;
        fireEvent1 = true;
      }
      
      break;
      
    case S_WAITFORPYRO2:
      if (alt < MAIN_ALT)
      {
        fire_pyro(PYRO2);
        state = S_WAITFORGROUND;
        ts2 = time;
        fireEvent2 = true;
      }
      
      break;
      
    case S_WAITFORGROUND:
      if (alt < GROUND_ALT)
      {
        state = S_FINISHED;
      }
      
      break;
      
    case S_FINISHED:
      failure_mode("Finished!");
           
      break;
  }
  
  check_pyro_event(time, PYRO1, ts1, fireEvent1);
  check_pyro_event(time, PYRO2, ts2, fireEvent2);
  
  return state;
}

void check_pyro_event(unsigned long time, int pyro_pin, unsigned long ts, bool event_flag)
{
  if ((time > ts + PYRO_DURATION) && event_flag)
  {
    turn_off_pyro(pyro_pin);
  }
}

void turn_off_pyro(int pyro_pin)
{
  pinMode(pyro_pin, OUTPUT);
  digitalWrite(pyro_pin, LOW);
}

void find_flight_num()
{
  flight_num = 1;
  if (USE_SD) {
    if (SD.exists("flights.txt")) {
      // Read flights.txt file and find last recorded flight number.
      File flights_read = SD.open("flights.txt");
      
      while (flights_read.available()) {
        char result[7]; 
        uint8_t i = 0; 
        
        char fileStr;
       
        fileStr = flights_read.read();
        
        while (fileStr != '\0' && i < sizeof(result) - 1) 
        {
          result[i] = fileStr;
          fileStr = flights_read.read();
          i++; 
        }
        
        result[i] = '\0';
        flight_num = atol(result) + 1;
      }
      flights_read.close();
      
      File flights_write = SD.open("flights.txt", FILE_WRITE | O_TRUNC);
      flights_write.println(String(flight_num));
      flights_write.close();
      
    } else {
      // Start new flights.txt file
      File flights = SD.open("flights.txt", FILE_WRITE);
      
      flights.println(flight_num);
      
      flights.close(); 
    }
  }
}

bool write_to_sd(String text, bool newLine)
{
  if (USE_SD) {
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    String fileName = "flight" + String(flight_num) + ".csv";
    
    File dataFile = SD.open(fileName.c_str(), FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) 
    {
      if (newLine)
      {
        dataFile.println(text);
      }
      else
      {
        dataFile.print(text);
      }
      
      dataFile.flush();
      dataFile.close();
    }
    else
    {
      // return false on error
      return false;
    }
  }
  // return true on success
  return true;
}

void failure_mode(String errMsg)
{
  write_to_sd(errMsg, NEWLINE);
  
  analogWrite(BUZZER_PIN, 10); 
  
  //turn_off_pyro(PYRO1);
  //turn_off_pyro(PYRO2);
  
  while (1);
}

void fire_pyro(int pyro_ch)
{
  pinMode(pyro_ch, OUTPUT);
  digitalWrite(pyro_ch, HIGH);

  write_to_sd("Successful pyro.", NEWLINE);
}
