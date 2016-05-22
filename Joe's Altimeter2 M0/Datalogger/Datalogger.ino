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
    - http://linux.die.net/man/3/va_arg
    - http://forum.arduino.cc/index.php?topic=87660.0
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <avr/dtostrf.h>
#include <stdlib.h>

#include <SD.h>               // SD Card module

#include <Adafruit_LIS3DH.h>  // LIS3DH Accel sensor
#include <Adafruit_BMP280.h>  // BMP280 Baro sensor

// Chip select pins for sensors and sd card.
#define BMP_CS 13             // Pin Number
#define SD_CS 4               // Pin Number
#define LIS3DH_CS 12          // Pin Number

// Buzzer pin
#define BUZZER_PIN 3          // Pin Number

// Pin numbers for pyro charges.
#define PYRO1 10              // Pin Number
#define PYRO2 11              // Pin Number

// Duration of pyro charge.
#define PYRO_DURATION 2000    // milliseconds

// Flight start detection settings.
// Values must be greater than all three settings
// to trigger flight start.
#define TRIGGER_ALT 10        // meters
#define TRIGGER_VEL 10        // meters/second
#define TRIGGER_TIME 500      // milliseconds

// Velocity limit to trigger apogee detection
#define APOGEE_VEL 10         // meter/second

// Drogue (pyro2) altitude limit
#define DROGUE_ALT 150        // meters

// Flight end detection height
#define GROUND_ALT 10         // meters

// Set this to the maximum working altitude of the
// baro sensor.
#define MAX_BARO_ALT 9000     // meters  

// Set these to enable/disable features.
// This is useful for reducing flash and ram
// usage on smaller processors.
#define USE_SD true           // Use SD Card Module?
#define USE_LIS3DH true       // Use LIS3DH accel module?
#define USE_1D_KALMAN true    // Use baro only calculations?
#define USE_2D_KALMAN true    // Use accel and baro calculations?
#define USE_ADXL true         // Use analog accel inputs for logging? 

// Analog pin numbers for analog accel inputs
#define ADXL_Z_PIN 2
#define ADXL_Y_PIN 1
#define ADXL_X_PIN 0

// Newline defaults
#define NEWLINE true
#define NONEWLINE false

// State machine states
#define S_INIT 0
#define S_FLIGHTSTARTED 1
#define S_WAITFORPYRO2 2
#define S_WAITFORGROUND 3
#define S_FINISHED 4

// 2D Kalman filter variables (Baro and Accel)
float est[3] = {0, 0, 0};
float estp[3] = {0, 0, 0};
float phi[3][3] = {{1, 0, 0},
  {0, 1, 0},
  {0, 0, 1.0}
};

// 2D Kalman Gain Matrix (Baro and Accel)
// Calculated using rkal32.c
float kgain[3][2] = {{0.114263, 0.057967},
  {0.055419, 0.088090},
  {0.019825, 0.085453}
};

// 1D Kalman filter variables (Baro only)
float P = 1.0;
float varP = pow(0.01, 2);
float varM = pow(0.5, 2);
float K = 1.0;
float kalVel = 0.0;

// Altimeter state variables
int state = S_INIT;           // Current state
unsigned long ts1;            // Milliseconds from first pyro
unsigned long ts2;            // Milliseconds from second pyro
bool fireEvent1 = false;      // First pyro event flag
bool fireEvent2 = false;      // Second pyro event flag
unsigned long flight_num = 0; // Flight Number
unsigned long prevMilli;      // Previous measurement time (milliseconds)
float prevAlt;                // Previous altitude measurement (meters)
float startAlt;               // Initial altitude measurement (meters)

// Sensor Initalization
Adafruit_BMP280 bme(BMP_CS);
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

void setup()
{
  // Setup serial debugging
  Serial.begin(9600);
  Serial.println("--- Start Serial Monitor ---");

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

  write_to_sd_str("time(ms),temperature(C),pressure,absolute altitude(m)(probably wrong),altitude(m),raw velocity(m/s),filtered velocity(m/s)");
  write_to_sd_str("Raw X Accel(m/s^2),Raw Y Accel(m/s^2),Raw Z Accel(m/s^2),Vector Accel(m/s^2),Est Alt(m),Est Vel(m/s),Est Accel(m/s^2)");
  write_to_sd_str("Analog Raw X Accel(m/s^2),Analog Raw Y Accel(m/s^2),Analog Raw Z Accel(m/s^2)");
  write_to_sd_new_line();

  // Take and discard first 10 measurements.
  for (int x = 0; x < 10; x++) {
    // Need to first read temp and pressure to initialize readAltitude.
    bme.readTemperature();
    bme.readPressure();
  
    // Get initial altitude measurement.
    startAlt = bme.readAltitude(1013.25);
    prevAlt = startAlt;
  }

  // Initialize clock.
  prevMilli = millis();

  
}

void loop()
{
  float temp = bme.readTemperature();
  long int pressure = bme.readPressure();
  float rawAlt = bme.readAltitude(1013.25);

  unsigned long curMilli = millis();

  float rawVel = (rawAlt - prevAlt) / ((float)(curMilli - prevMilli) / 1000);

  if (USE_1D_KALMAN) {
    oneD_kalman(rawVel);
  }

  float relAlt = rawAlt - startAlt;

  prevAlt = rawAlt;
  prevMilli = curMilli;

  write_to_sd_long(curMilli);
  write_to_sd_float(temp);
  write_to_sd_long((unsigned long) pressure);
  write_to_sd_float(rawAlt);
  write_to_sd_float(relAlt);
  write_to_sd_float(rawVel);
  write_to_sd_float(kalVel);

  float vec_accel = 0;

  if (USE_LIS3DH) {
    sensors_event_t event;
    lis.getEvent(&event);

    write_to_sd_float(event.acceleration.x);
    write_to_sd_float(event.acceleration.y);
    write_to_sd_float(event.acceleration.z);

    vec_accel = abs_accel(event.acceleration.x, event.acceleration.y, event.acceleration.z);
  }

  if (USE_2D_KALMAN) {
    if (USE_LIS3DH) {
      write_to_sd_float(vec_accel);
    }

    twoD_kalman(vec_accel, relAlt);

    write_to_sd_float(est[0]);
    write_to_sd_float(est[1]);
    write_to_sd_float(est[2]);

    altimeter_state_tree(curMilli, est[1], est[0]);
  }
  else {
    if (USE_LIS3DH) {
      write_to_sd_float(vec_accel);
    }

    altimeter_state_tree(curMilli, kalVel, relAlt);
  }

  if (USE_ADXL) {
    write_to_sd_long(analogRead(ADXL_X_PIN));
    write_to_sd_long(analogRead(ADXL_Y_PIN));
    write_to_sd_long(analogRead(ADXL_Z_PIN));
  }

  write_to_sd_new_line();
}

float abs_accel(float accel_x, float accel_y, float accel_z)
{
  return sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
}

int altimeter_state_tree(unsigned long time, float vel, float alt)
{
  switch (state)
  {
    case S_INIT:
      if (time > TRIGGER_TIME) {
        if (vel > TRIGGER_VEL) {
          if (alt > TRIGGER_ALT) {
            state = S_FLIGHTSTARTED;
          }
        }
      }

      break;

    case S_FLIGHTSTARTED:
      if (vel < APOGEE_VEL) {
        fire_pyro(PYRO1);
        state = S_WAITFORPYRO2;
        ts1 = time;
        fireEvent1 = true;
      }

      break;

    case S_WAITFORPYRO2:
      if (alt < DROGUE_ALT) {
        fire_pyro(PYRO2);
        state = S_WAITFORGROUND;
        ts2 = time;
        fireEvent2 = true;
      }

      break;

    case S_WAITFORGROUND:
      if (alt < GROUND_ALT) {
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
  if ((time > ts + PYRO_DURATION) && event_flag) {
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

        while (fileStr != '\0' && i < sizeof(result) - 1) {
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
  Serial.println(flight_num);
  Serial.flush();
}

bool write_to_sd_new_line()
{
  Serial.println(" ");
  Serial.flush(); 

  if (USE_SD) {
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    String fileName = "flight" + String(flight_num) + ".csv";
    
    File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(" ");
      dataFile.flush();
      dataFile.close();
    }
    else {
      // return false on error
      return false;
    }
  }
  
  // return true on success
  return true;
}

bool write_to_sd_str( String data )
{
  Serial.print(data);
  Serial.print(", ");
  Serial.flush(); 
  
  if (USE_SD) {
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    String fileName = "flight" + String(flight_num) + ".csv";
    
    File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(data);
      dataFile.print(", ");
      dataFile.flush();
      dataFile.close();
    } else {
      // return false on error
      return false;
    }
  }

  // return true on success
  return true;
}

bool write_to_sd_float( float data )
{
  Serial.print(data, 6);
  Serial.print(", ");
  Serial.flush(); 
  
  if (USE_SD) {
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    String fileName = "flight" + String(flight_num) + ".csv";
    
    File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(data, 6);
      dataFile.print(", ");
      dataFile.flush();
      dataFile.close();
    } else {
      // return false on error
      return false;
    }
  }

  // return true on success
  return true;
}

bool write_to_sd_long( unsigned long data )
{
  Serial.print(data);
  Serial.print(", ");
  Serial.flush(); 
  
  if (USE_SD) {
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    String fileName = "flight" + String(flight_num) + ".csv";
    
    File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(data);
      dataFile.print(", ");
      dataFile.flush();
      dataFile.close();
    } else {
      // return false on error
      return false;
    }
  }

  // return true on success
  return true;
}

void failure_mode(String errMsg)
{
  write_to_sd_str(errMsg);
  write_to_sd_new_line();

  pinMode(BUZZER_PIN, INPUT_PULLUP);

  turn_off_pyro(PYRO1);
  turn_off_pyro(PYRO2);

  while (1) {
    Serial.println(errMsg);
    Serial.flush();
  }
}

void fire_pyro(int pyro_ch)
{
  pinMode(pyro_ch, OUTPUT);
  digitalWrite(pyro_ch, HIGH);
 
  write_to_sd_str("Successful pyro.");
  write_to_sd_new_line();
}

void oneD_kalman(float value)
{
  P = P + varP;
  K = P / (P + varM);
  kalVel = K * value + (1 - K) * kalVel;
  P = (1 - K) * P;
}

void twoD_kalman(float accel, float alt)
{
  // Compute the innovations
  float alt_inovation = alt - estp[0];
  float accel_inovation = accel - estp[2];

  // Experimental code to modify Mach transition pressure
  // disturbances.
  if ( abs(alt_inovation) > 100 ) {
    // We have a large error in altitude. Now see how fast we are
    // going.
    if ( estp[1] > 256 && estp[1] < 512 ) {
      // Somewhere in the neighborhood of Mach 1. Now check to
      // see if we are slowing down.
      if ( estp[2] < 0 ) {
        // OK, now what do we do? Assume that velocity and
        // acceleration estimates are accurate. Adjust current
        // altitude estimate to be the same as the measured
        // altitude.
        est[0] = alt;
        alt_inovation = 0;
      }
    }
  }

  // Simple check for over-range on pressure measurement.
  // This is just hacked in based on a single data set. Actual
  // flight software needs something more sophisticated.
  // TODO: Fix this...
  if ( alt > MAX_BARO_ALT ) {
    alt_inovation = 0;
  }

  // Propagate state
  estp[0] = phi[0][0] * est[0] + phi[0][1] * est[1] + phi[0][2] * est[2];
  estp[1] = phi[1][0] * est[0] + phi[1][1] * est[1] + phi[1][2] * est[2];
  estp[2] = phi[2][0] * est[0] + phi[2][1] * est[1] + phi[2][2] * est[2];

  // Update state
  est[0] = estp[0] + kgain[0][0] * alt_inovation + kgain[0][1] * accel_inovation;
  est[1] = estp[1] + kgain[1][0] * alt_inovation + kgain[1][1] * accel_inovation;
  est[2] = estp[2] + kgain[2][0] * alt_inovation + kgain[2][1] * accel_inovation;
}
