/*
   AltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include <Wire.h>
// IMU
#include <LSM6DSM.h>
// Barometer
#include <LPS22HB.h>
// Rangefinder
#include <VL53L1X.h>
// estimator
#include "zestimation.h"

uint8_t LED_PIN = 38;
// --- IMU related variables and functions ---
// LSM6DSM settings
static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_2G;
static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_2000DPS;
static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_1660Hz;
static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_1660Hz;

// Biases computed by Kris
float ACCEL_BIAS[3] = {0.0, 0.0, 0.0};
float GYRO_BIAS[3]  = {0.0, 0.0, 0.0};


LSM6DSM lsm6dsm = LSM6DSM(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);

static void imuRead(float gyro[3], float accel[3])
{
    if (lsm6dsm.checkNewData()) {
        float _ax, _ay, _az, _gx, _gy, _gz;
        lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);

        // Negate to support board orientation
        _ax = -_ax;
        _gy = -_gy;
        _gz = -_gz;

        // Copy gyro values back out in rad/sec
        gyro[0] = _gx * M_PI / 180.0f;
        gyro[1] = _gy * M_PI / 180.0f;
        gyro[2] = _gz * M_PI / 180.0f;
        // and acceleration values
        accel[0] = _ax;
        accel[1] = _ay;
        accel[2] = _az;

    } 
}

// --- Barometer related variables and functions ---
// Pressure and temperature oversample rate
static LPS22HB::Rate_t ODR = LPS22HB::P_75Hz;     
static LPS22HB lps22hb = LPS22HB(ODR);

// --- Rangefinder related variables and functions ---
static VL53L1X distanceSensor;

// Altitude estimator
static zestimation::AltitudeEstimator altitude = zestimation::AltitudeEstimator(
        20.0,   // gain
        0.0005, // sigma Accel
        0.0005, // sigma Gyro
        0.05,  // sigma Baro
        0.5,    // ca
        0.1);   // accelThreshold

float pastTime = millis();
float currentTime = millis();

void setup(void)
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);
    // initialize sensors
    lsm6dsm.begin();
    lsm6dsm.calibrate(GYRO_BIAS, ACCEL_BIAS);
    lps22hb.begin();
    if (distanceSensor.begin() == false) {
        while (true) {
            Serial.println("Sensor offline!");
            delay(200);
        }
    }
    // Begin serial comms
    Serial.begin(115200);
    // Set up the interrupt pin, it's set as active high, push-pull
    digitalWrite(LED_PIN, HIGH);
}

void loop(void)
{
  currentTime = millis();
  if ((currentTime - pastTime) > 50)
  {
    uint32_t timestamp = micros();
    // get all necessary data
    float pressure = lps22hb.readPressure();
    float rangeHeight = (float)distanceSensor.getDistance() / 1000.0f;
    float accelData[3];
    float gyroData[3];
    imuRead(gyroData, accelData);
    altitude.estimate(accelData, gyroData, rangeHeight, pressure, timestamp);
    Serial.print(altitude.twoStep.getAltitude());
    Serial.print(",");
    Serial.print(altitude.range.getAltitude());
    Serial.print(",");
    Serial.println(altitude.getAltitude());
    //Serial.print(",");
    //Serial.println(altitude.getVerticalAcceleration());
    pastTime = currentTime;
  }
}