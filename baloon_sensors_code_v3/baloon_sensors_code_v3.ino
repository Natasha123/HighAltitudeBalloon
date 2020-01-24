// Include libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MPU9250_asukiaaa.h>
#include "SparkFunMPL3115A2.h"
#include "SparkFun_Si7021_Breakout_Library.h"

// IMU pin setup
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

// Constants and variables
const int chipSelect = 8;

// Weather sensor constants and variables
float celciusToKelvinAdjustmentAmount = 273.15;
float weatherSensorHumidity = 0;
float weatherTemperature = 0;

// IMU sensor variables
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;

// Pressure sensor constants and variables
const float altitudeAdjustmentFactor = 0.3048;
float altitude, pressure, pressureSensorTemperature;

String filename = "datalog4.csv";

// Sensor objects
MPL3115A2 pressureSensor;
Weather weatherSensor;
MPU9250_asukiaaa imuSensor;

void setup()
{
  // I2C stuff
  Wire.begin();
  Serial.begin(57600);
  while(!Serial);

  // Initialize IMU pins
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  imuSensor.setWire(&Wire);
  #endif

  // Start communication with sensors
  SD.begin(chipSelect);
  pressureSensor.begin();
  weatherSensor.begin();
  imuSensor.beginAccel();
  imuSensor.beginGyro();
  imuSensor.beginMag();

  // Configure sensors
  int sampleRateCode = 7;  // Set Oversample to the recommended 128
  pressureSensor.setModeAltimeter(); // Measure altitude above sea level in meters
  pressureSensor.setOversampleRate(sampleRateCode); // Set Oversample to the recommended 128
  pressureSensor.enableEventFlags(); // Enable all three pressure and temp event flags 

  // Write csv title row to SD card
  File fd = SD.open(filename, FILE_WRITE);

  String titleRow = "MillisecondsSinceBoardStarted,Altitude,Pressure,pressureSensorTemperature,weatherSensorTemperature,Humidity,gX,gY,gZ,aX,aY,aZ,mX,mY,mZ"; 
  fd.println(titleRow);
  fd.close();
}

void getPressureSensorData()
{
  altitude = pressureSensor.readAltitudeFt()* altitudeAdjustmentFactor;
  pressure = pressureSensor.readPressure();
  pressureSensorTemperature = pressureSensor.readTemp() + celciusToKelvinAdjustmentAmount;
}

void getWeatherSensorData()
{
  weatherSensorHumidity = weatherSensor.getRH();
  weatherTemperature = weatherSensor.getTemp()+ celciusToKelvinAdjustmentAmount;
}


void getImuSensorData()
{
  // update acceleration values if they've changed, othewise don't change their existing values
  if (imuSensor.accelUpdate() == 0) {
    aX = imuSensor.accelX();
    aY = imuSensor.accelY();
    aZ = imuSensor.accelZ();  
  }
  if (imuSensor.gyroUpdate() == 0) {
    gX = imuSensor.gyroX();
    gY = imuSensor.gyroY();
    gZ = imuSensor.gyroZ();
  } 
  if (imuSensor.magUpdate() == 0) {
    mX = imuSensor.magX();
    mY = imuSensor.magY();
    mZ = imuSensor.magZ();
  }
}

void loop()
{
  uint8_t sensorId;
  
  // Read values from sensors
  getPressureSensorData();
  getWeatherSensorData();
  getImuSensorData();

  File fd = SD.open(filename, FILE_WRITE); // Opening the SD card file to write
  
  // Write sensor readings as csv row
  String csvRow = "";
  csvRow += String(millis()) + ",";
  csvRow += String(altitude) + "," + String(pressure) + ",";
  csvRow += String(pressureSensorTemperature) + "," + String(weatherTemperature) + "," + String(weatherSensorHumidity) + ",";
  csvRow += String(gX) + "," + String(gY) + "," + String(gZ) + ",";
  csvRow += String(aX) + "," + String(aY) + "," + String(aZ) + ",";
  csvRow += String(mX) + "," + String(mY) + "," + String(mZ);
  
  fd.println(csvRow);

  delay(100);
  fd.close();
  delay(100);
}
