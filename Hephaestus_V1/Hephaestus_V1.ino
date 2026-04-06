#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"

/*
* Hephaestus flight computer code
* 1.0
* Flight count - 0
* Galalumga board
* Altitude, Temperature, Pressure from BMP280 breakout board
* Averaged gyro and acceleration from dual GY-251 MPU6050 breakout boards
* Written by CR_DGD
*/

//Pins
const int buzzer = 8;
const int ledrxtx = 6;
const int ledon = 9;
const int goled = 13;

//Log time value
int logstep = 0;

//------------------------------------------- SENSOR DECLARE ---------------------------------------------------

//MPU1
MPU6050 mpu1(0x68, &Wire2);
//Found device at 0x68

//MPU2
MPU6050 mpu2(0x69, &Wire2);
//Found device at 0x69

#define CHIP_ID_REG 0xD0
//BMP
//Found device at 0x76
Adafruit_BMP280 bmp(&Wire2);
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


void setup() 
{
//------------------------------------------- SERIAL AND PIN INIT -------------------------------------------
 //SERIAL & I2C
  Serial.begin(9600);
  Wire2.begin();
  
//PINOUT
  pinMode(buzzer, OUTPUT);
  pinMode(ledrxtx, OUTPUT);
  pinMode(ledon, OUTPUT);
  pinMode(goled, OUTPUT);
  Serial.println("Initializing Pins");

//------------------------------------------- SENSOR INIT ---------------------------------------------------
//------------------------------------------- INITALIZE BMP -------------------------------------------------
  unsigned status;
  Serial.println("Initializing BMP");
  status = bmp.begin(0x76);

  if (!status) 
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  }
  Serial.print("BMP ID was: 0x"); Serial.println(bmp.sensorID(),16);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);
  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);                


//------------------------------------------- INITALIZE MPU1 -------------------------------------------------
  Serial.println("Initializing MPU1");
  mpu1.initialize();
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu1.setFullScaleAccelRange(2);

  mpu1.CalibrateAccel(30);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu1.CalibrateGyro(10);
  Serial.println("These are the Active offsets: ");
  mpu1.PrintActiveOffsets();//Get expected DMP packet size for later comparison

  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);
  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);

//------------------------------------------- INITALIZE MPU2 -------------------------------------------------
  Serial.println("Initializing MPU2");
  mpu2.initialize();
  mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu1.setFullScaleAccelRange(2);


  mpu2.CalibrateAccel(30);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu2.CalibrateGyro(10);
  Serial.println("These are the Active offsets: \n");
  mpu2.PrintActiveOffsets();

  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);
  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);

  Wire2.beginTransmission(0x76);
  Wire2.write(CHIP_ID_REG);
  Wire2.endTransmission();


//------------------------------------------- BMP TEST -------------------------------------------------
  Wire2.requestFrom(0x76, 1);
  if (Wire2.available()) {
    byte id = Wire2.read();
    Serial.print("Chip ID: 0x");
    Serial.println(id, HEX);

    if (id == 0x58) {
      Serial.println("BMP280 Online");
    } else {
      Serial.println("Unexpected chip.");
    }
  } else {
    Serial.println("No response from BMP280.");
  }


//------------------------------------------- MPU TEST -------------------------------------------------
//MPU1
  if (!mpu1.testConnection()) 
  {
    Serial.println("MPU1 Connection Failed");
  }
  else
  {
    Serial.println("MPU1 Online");
  }
//MPU2
  if (!mpu2.testConnection()) 
  {
    Serial.println("MPU2 Connection Failed");
  }
  else
  {
    Serial.println("MPU2 Online");
  }

  digitalWrite(goled,HIGH);
  delay(600);
  digitalWrite(goled,LOW);
  delay(1200);
  digitalWrite(goled,HIGH);
  Serial.println("System check complete. Logging started");
}

void loop()
{
  logstep++;
//------------------------------------------- MPU LOG -------------------------------------------------
  int16_t ax1, ay1, az1, gx1, gy1, gz1; //MPU1
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  int16_t ax2, ay2, az2, gx2, gy2, gz2; //MPU2
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

  float avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz;
  avg_ax = avg(ax1,ax2);
  avg_ay = avg(ay1,ay2);
  avg_az = avg(az1,az2);
  avg_gx = avg(gx1,gx2);
  avg_gy = avg(gy1,gy2);
  avg_gz = avg(gz1,gz2);

//------------------------------------------- BMP LOG -------------------------------------------------
  float f = bmp.readTemperature();
  float P = bmp.readPressure()/100;
  float A = bmp.readAltitude(1013);


//------------------------------------------- FORMAT LOG -------------------------------------------------
  formatpacket(logstep,f,A,P,avg_ax / 16384.0,avg_ay / 16384.0,avg_az / 16384.0,avg_gx / 131,avg_gy  / 131,avg_gz  / 131);
  delay(100);
}

float avg(int16_t val1, int16_t val2)
{
  int16_t total = val1+val2;
  return ((float)total)/2;
}

void formatpacket(int16_t time, int16_t temp, int16_t alt, int16_t baro, float ax, float ay, float az, float gx, float gy, float gz)
{
  char buffer[120];
  sprintf(buffer, "%06i | %04d | %04d | %04d",time,temp,alt,baro);
  //Serial.print(buffer);
  //Serial.print(" | ");
  Serial.print(ax , 4);
  Serial.print(" | ");
  Serial.print(ay , 4);
  Serial.print(" | ");
  Serial.print(az , 4);
  Serial.print(" | ");
  Serial.print(gx , 4);
  Serial.print(" | ");
  Serial.print(gy , 4);
  Serial.print(" | ");
  Serial.println(gz);
}

