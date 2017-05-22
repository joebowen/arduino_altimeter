/*
  Flexible Field-Proof Arduino Model Rocket Altimeter

  Author: Joe Bowen
  Date: 1/16/17
*/

// Chip select pins for sensors and sd card.
#define BMP_CS 0              // Pin Number
#define SD_CS 4               // Pin Number
#define LIS3DH_CS 1           // Pin Number

// Buzzer pin
#define BUZZER_PIN 3          // Pin Number

// Pin numbers for pyro charges.
#define PYRO1 10              // Pin Number
#define PYRO2 11              // Pin Number

// Duration of pyro charge.
#define PYRO_DURATION 10000   // milliseconds

// Flight start detection settings.
// Values must be greater than all three settings
// to trigger flight start.
#define TRIGGER_ALT 1         // meters
#define TRIGGER_VEL 1         // meters/second
#define TRIGGER_TIME 50       // milliseconds

// Velocity limit to trigger apogee detection
#define APOGEE_VEL 10         // meters/second

// Main (pyro2) altitude limit
#define MAIN_ALT 150          // meters

// Flight end detection height
#define GROUND_ALT 10         // meters

// Set this to the maximum working altitude of the
// baro sensor.
#define MAX_BARO_ALT 9000     // meters  

// Speed from the accelerometer above which locks 
// out deployment of drogue due to suspected mach effects.
#define MACH_LOCK 100         // meters/second

// Set these to enable/disable features.
// This is useful for reducing flash and ram
// usage on smaller processors.
#define USE_SD true           // Use SD Card Module?
#define USE_LIS3DH true       // Use LIS3DH accel module?
#define USE_ADXL true         // Use analog accel inputs for logging? 

// Analog pin numbers for analog accel inputs
#define ADXL_Z_PIN 2
#define ADXL_Y_PIN 1
#define ADXL_X_PIN 0

// Accelerometer smoothing variable for the exponential decaying moving average.
#define SMOOTHING .02 
