//FC 1.0
// sensor and pids implemention

int DEBUGE = 1;
int IMU_LIB = 1;

#include <PID_v1.h>

///////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Wire.h"

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/////////////////////////////////////////////////////////////////////////////////   VERSION 2
#include "I2Cdev.h"
//namespace foo {
//   
//    #include <MPU6050_light.h>
//}
 #include "MPU6050.h"
 
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>

#if IMU_LIB == 0 
  MPU6050 accelgyro;
#endif

Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

int IMU_MODE = 2;
double gyro_x, gyro_y, gyro_z;
/////////////////////////////////////////////////////////////////////////////////   VERSION 2

///////////////////////////////////////////////////////////////////////////////// VERSION 3 BEGINE
//foo::MPU6050 mpu(Wire);

long timer = 0;

double angle_x = 0;
double angle_y = 0;
double angle_z = 0;
///////////////////////////////////////////////////////////////////////////////// VERSION 3 END


//MPU6050 mpu(Wire);

int RX = 11;
int TX = 10;
TinyGPSPlus gps;
SoftwareSerial gpsPort(RX, TX);
double gps_x0 = 0;
double gps_y0 = 0;
double gps_x = 0;
double gps_y = 0;
double gps_thrust0 = 0;
double gps_thrust = 0;
double thrust0 = 0;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
double heading = 0;
double declinationAngle = 0.22;
double headingDegrees = 0;

///////////////////////////////////

// motor pins definition
#define M1 0 //front left
#define M2 1 //front right
#define M3 2 //back left
#define M4 3

// pid variables for roll pitch yaw and thrust
// roll variables
double roll_setpoint, roll_input, roll_output;
double roll_Kp = 2, roll_Ki = 5, roll_Kd = 1;
PID roll_pid(&roll_input, &roll_output, &roll_setpoint, roll_Kp, roll_Ki, roll_Kd, DIRECT);

//pitch
double pitch_setpoint, pitch_input, pitch_output;
double pitch_Kp = 2, pitch_Ki = 5, pitch_Kd = 1;
PID pitch_pid(&pitch_input, &pitch_output, &pitch_setpoint, pitch_Kp, pitch_Ki, pitch_Kd, DIRECT);

//yaw
double yaw_setpoint, yaw_input, yaw_output;
double yaw_Kp = 2, yaw_Ki = 5, yaw_Kd = 1;
PID yaw_pid(&yaw_input, &yaw_output, &yaw_setpoint, yaw_Kp, yaw_Ki, yaw_Kd, DIRECT);

//thrust
double thrust_setpoint, thrust_input, thrust_output;
double thrust_Kp = 2, thrust_Ki = 5, thrust_Kd = 1;
PID thrust_pid(&thrust_input, &thrust_output, &thrust_setpoint, thrust_Kp, thrust_Ki, thrust_Kd, DIRECT);

// motor pwm
double m1 = 0;
double m2 = 0;
double m3 = 0;
double m4 = 0;

void setup() {
  // put your setup code here, to run once:
  //set up pid variables here setpoint, input



  ///////////////////////////////////////////////////////////////////////////////// VERSION 3 BEGINE
//  Serial.begin(9600);
//  Wire.begin();
//
//  byte status = mpu.begin();
//  Serial.print(F("MPU6050 status: "));
//  Serial.println(status);
//  while (status != 0) { } // stop everything if could not connect to MPU6050
//
//  Serial.println(F("Calculating offsets, do not move MPU6050"));
//  delay(1000);
//  mpu.calcOffsets(true, true); // gyro and accelero
//  Serial.println("Done!\n");
  ///////////////////////////////////////////////////////////////////////////////// VERSION 3 END


  ///////////////////////////////////////////////////////////////////////////////// VERSION 2 BEGINE
  if (IMU_MODE == 2) {
    Serial.begin(9600);
    Wire.begin();
    gpsPort.begin(9600);

    // initialize devices
    Serial.println("Initializing I2C devices...");

    // initialize bmp085
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }

    // initialize mpu6050
    accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L


    // initialize hmc5883l
    Compass.SetDeclination(23, 35, 'E');
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_130);
    Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);

    if (gpsPort.available() > 0)
      if (gps.encode(gpsPort.read()))
      {
        gps_x0 = gps.location.lat();
        gps_y0 = gps.location.lng();
        gps_thrust0 = gps.altitude.meters();
      }
    thrust0 = bmp.readAltitude(101500);
    // configure Arduino LED for checking activity
    pinMode(LED_PIN, OUTPUT);
  }
  /////////////////////////////////////////////////////////////////////////////////   VERSION 2 END


  // turning pids on
  roll_pid.SetMode(AUTOMATIC);
  pitch_pid.SetMode(AUTOMATIC);
  yaw_pid.SetMode(AUTOMATIC);
  thrust_pid.SetMode(AUTOMATIC);
}

void loop() {


  //input variable for roll pitch and yaw must be set here if the input varible is not called by refrence.
  // roll_input = sensor() ...

  ///////////////////////////////////////////////////////////////////////////////// VERSION 3 BEGINE
//  mpu.update();
//  if (DEBUGE == 1 && IMU_LIB == 1) {
//    if (millis() - timer > 1000) { // print data every second
//      Serial.print(F("TEMPERATURE: ")); Serial.println(mpu.getTemp());
//      Serial.print(F("ACCELERO  X: ")); Serial.print(mpu.getAccX());
//      Serial.print("\tY: "); Serial.print(mpu.getAccY());
//      Serial.print("\tZ: "); Serial.println(mpu.getAccZ());
//
//      Serial.print(F("GYRO      X: ")); Serial.print(mpu.getGyroX());
//      Serial.print("\tY: "); Serial.print(mpu.getGyroY());
//      Serial.print("\tZ: "); Serial.println(mpu.getGyroZ());
//
//      Serial.print(F("ACC ANGLE X: ")); Serial.print(mpu.getAccAngleX());
//      Serial.print("\tY: "); Serial.println(mpu.getAccAngleY());
//
//      Serial.print(F("ANGLE     X: ")); Serial.print(mpu.getAngleX());
//      Serial.print("\tY: "); Serial.print(mpu.getAngleY());
//      Serial.print("\tZ: "); Serial.println(mpu.getAngleZ());
//      Serial.println(F("=====================================================\n"));
//      timer = millis();
//    }
//  }
//  if (IMU_LIB == 1) {
//    roll_input  = mpu.getAccX();
//    pitch_input = mpu.getAccY();
//    yaw_input   = mpu.getAccZ();
//
//    gyro_x = mpu.getGyroX();
//    gyro_y = mpu.getGyroY();
//    gyro_z = mpu.getGyroZ();
//
//    angle_x = mpu.getAngleX();
//    angle_y = mpu.getAngleY();
//    angle_z = mpu.getAngleZ();
//  }
  ///////////////////////////////////////////////////////////////////////////////// VERSION 3 END

  /////////////////////////////////////////////////////////////////////////////////   VERSION 2 BEGINE
  if (IMU_MODE == 2) {
    if (DEBUGE == 1 && IMU_LIB == 0) {
      Serial.print("Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pa");

      // Calculate altitude assuming 'standard' barometric
      // pressure of 1013.25 millibar = 101325 Pascal
      Serial.print("Altitude = ");
      Serial.print(bmp.readAltitude());
      Serial.println(" meters");
      Serial.print("Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");
      Serial.print("Real altitude = ");
      Serial.print(bmp.readAltitude(101500));
      Serial.println(" meters");

    }

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //    accelgyro.getMotion6(&roll_input, &pitch_input, &yaw_input, &gyro_x, &gyro_y, &gyro_z);

    if (IMU_LIB == 0) {
      roll_input  = double(ax);
      pitch_input = double(ay);
      yaw_input   = double(az);

      gyro_x = gx;
      gyro_y = gy;
      gyro_z = gx;
    }

    if (DEBUGE == 1 && IMU_LIB == 0) {
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    }
    thrust_input = bmp.readAltitude(101500);

    //    if (millis() > 5000 && gps.charsProcessed() < 10)
    //      Serial.println("No GPS Device; Try Again Later.");
    if (gpsPort.available() > 0)
      if (gps.encode(gpsPort.read()))
      {
        gps_x = gps.location.lat();
        gps_y = gps.location.lng();
      }
  }

  //Compass
  double heading = Compass.GetHeadingDegrees();
  //    Serial.print("Heading: \t");
  //    Serial.println( heading );


  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  if (DEBUGE == 1) {
    delay(1000);
  }

  //}
  /////////////////////////////////////////////////////////////////////////////////   VERSION 2 END


  // pid.cumpute must be called every loop
  roll_pid.Compute();
  pitch_pid.Compute();
  yaw_pid.Compute();
  thrust_pid.Compute();
  //setting motor pwms
  m1 = thrust_output - yaw_output - pitch_output - roll_output;
  m2 = thrust_output + yaw_output - pitch_output + roll_output;
  m3 = thrust_output + yaw_output + pitch_output - roll_output;
  m4 = thrust_output - yaw_output + pitch_output + roll_output;
  // motor ports
  //analogWrite(, );
  Serial.print("M1: "); Serial.print(m1); Serial.print(" M2: "); Serial.print(m2); Serial.print(" M3: "); Serial.print(m3); Serial.print(" M4: "); Serial.println(m4);
}
