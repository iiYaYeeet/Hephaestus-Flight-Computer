#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define CHIP_ID_REG 0xD0
#define G 9.80665

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
unsigned long prevTime = 0;
float dt;

//output values
float pitch;
float roll;
float pitchAcc;
float rollAcc;
float P_launch = 0;

//vec3 struct
struct Vector3 {
  float x;
  float y;
  float z;
};

//------------------------------------------- SENSOR DECLARE ------------------------------------------------------

/*
* MPU1 at 0x68
* MPU2 at 0x69, ADO High
* BMP280 at 0x76
*/

//MPU1
MPU6050 mpu1(0x68, &Wire2);
//Found device at 0x68

//MPU2
MPU6050 mpu2(0x69, &Wire2);
//Found device at 0x69

//BMP
Adafruit_BMP280 bmp(&Wire2);
//Found device at 0x76


void setup() 
{
#pragma region ------------------------------------------- SERIAL AND PIN INIT ----------------------------------------------
 //SERIAL & I2C
  Serial.begin(9600);
  Wire2.begin();
  Wire2.setClock(100000); 
  
//PINOUT
  pinMode(buzzer, OUTPUT);
  pinMode(ledrxtx, OUTPUT);
  pinMode(ledon, OUTPUT);
  pinMode(goled, OUTPUT);
  Serial.println("Initializing Pins");

#pragma endregion

#pragma region ------------------------------------------- SENSOR INIT ---------------------------------------------------

  #pragma region ------------------------------------------- INITALIZE BMP -------------------------------------------------
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

  P_launch = bmp.readPressure() / 100.0F; // convert Pa to hPa
  Serial.print("Launch pressure: "); Serial.println(P_launch);                

  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);
  digitalWrite(goled,HIGH);
  delay(60);
  digitalWrite(goled,LOW);
  delay(60);                
  #pragma endregion

  #pragma region ------------------------------------------- INITALIZE MPU1 -------------------------------------------------
  Serial.println("Initializing MPU1");
  mpu1.initialize();
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  mpu1.setFullScaleAccelRange(3);
  mpu1.setDLPFMode(1);
  delay(100);

  mpu1.CalibrateAccel(30);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu1.CalibrateGyro(30);
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
  #pragma endregion

  #pragma region ------------------------------------------- INITALIZE MPU2 -------------------------------------------------
  Serial.println("Initializing MPU2");
  mpu2.initialize();
  mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  mpu2.setFullScaleAccelRange(3);
  mpu2.setDLPFMode(1);
  delay(100);

  mpu2.CalibrateAccel(30);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu2.CalibrateGyro(30);
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
  #pragma endregion

#pragma endregion

#pragma region ------------------------------------------- BMP TEST -------------------------------------------------
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
#pragma endregion

#pragma region ------------------------------------------- MPU TEST -------------------------------------------------
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
#pragma endregion
  
  
  digitalWrite(goled,HIGH);
  delay(600);
  digitalWrite(goled,LOW);
  delay(1200);
  digitalWrite(goled,HIGH);
  Serial.println("System check complete. Logging started");

}


Vector3 remap(Vector3 raw) 
{
    Vector3 mapped;

    // Pick axis with largest magnitude as vertical
    float absX = abs(raw.x), absY = abs(raw.y), absZ = abs(raw.z);

    if (absX > absY && absX > absZ) 
    {// X is vertical
        mapped.x = raw.x; mapped.y = raw.y; mapped.z = raw.z;
    } 
    else if (absY > absZ) 
    {// Y is vertical
        mapped.x = raw.y; mapped.y = raw.x; mapped.z = raw.z;
        if (raw.y < 0) mapped.x *= -1;
    } 
    else 
    {// Z is vertical
        mapped.x = raw.x; mapped.y = raw.y; mapped.z = raw.z;
    }
    return mapped;
}

void loop()
{
  logstep++;

  if (prevTime == 0) 
  {
  prevTime = micros();
  return;
  }

  unsigned long currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

#pragma region ------------------------------------------- MPU LOG -------------------------------------------------

  int16_t ax1, ay1, az1, gx1, gy1, gz1; //MPU1
  int16_t ax2, ay2, az2, gx2, gy2, gz2; //MPU2

  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  delayMicroseconds(500);
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

  Vector3 a1 = { ax1/2048.0, ay1/2048.0, az1/2048.0 };
  Vector3 a2 = { ax2/2048.0, ay2/2048.0, az2/2048.0 };
  Vector3 g1 = { gx1/32.8, gy1/32.8, gz1/32.8 };
  Vector3 g2 = { gx2/32.8, gy2/32.8, gz2/32.8 };


  Vector3 accel = { (a1.x+a2.x)/2, (a1.y+a2.y)/2, (a1.z+a2.z)/2 };

  float mag = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
  accel.x /= mag; accel.y /= mag; accel.z /= mag;

  Vector3 gyro =  { (g1.x+g2.x)/2, (g1.y+g2.y)/2, (g1.z+g2.z)/2 };

  accel = remap(accel);

#pragma endregion

#pragma region ------------------------------------------- BMP LOG -------------------------------------------------

  float P_now = bmp.readPressure() / 100.0F;// current pressure in hPa
  float temp = bmp.readTemperature();// temperature in °C
  float altitude = 8434.0 * log(P_launch / P_now); // meters

  temp = temp * 9.0 / 5.0 + 32.0;
  altitude = altitude * 3.28084;

#pragma endregion

#pragma region ------------------------------------------- FORMAT LOG -------------------------------------------------

  formatpacket(logstep,temp,altitude,P_now,accel,gyro);
  delay(10);

#pragma endregion

}

void formatpacket(int16_t time, int16_t temp, int16_t alt, int16_t baro, Vector3 accel, Vector3 gyro)
{

#pragma region ------------------------------------------- COMPLEMENT FILTER ------------------------------------------------- 
  float pitchAcc = atan2(accel.y, accel.z) * RAD_TO_DEG;
  float rollAcc  = atan2(-accel.x, accel.z) * RAD_TO_DEG;

  pitch = 0.998 * (pitch + gyro.x * dt) + 0.002 * pitchAcc;
  roll  = 0.998 * (roll  + gyro.y * dt) + 0.002 * rollAcc;
#pragma endregion

#pragma region ------------------------------------------- BUFFER LOGGING (TIME TEMP ALT BARO) -------------------------------------------------
  char buffer[120];
  sprintf(buffer, "%06i | %04d | %04d | %04d",time,temp,alt,baro);
  //temp,alt,baro
  Serial.print(buffer);
  Serial.print(" | ");
#pragma endregion
#pragma region ------------------------------------------- PITCH ROLL LOG -------------------------------------------------
  Serial.print(pitch , 3);
  Serial.print(" | ");
  Serial.print(roll , 3);
  Serial.print(" | ");
#pragma endregion  
#pragma region ------------------------------------------- ACCEL X Y Z LOG -------------------------------------------------
  Serial.print(accel.x , 4);
  Serial.print(" | ");
  Serial.print(accel.y , 4);
  Serial.print(" | ");
  Serial.print(accel.z , 4);
  Serial.print(" | ");
#pragma endregion
#pragma region ------------------------------------------- GYRO X Y Z LOG -------------------------------------------------
  Serial.print(gyro.x , 4);
  Serial.print(" | ");
  Serial.print(gyro.y , 4);
  Serial.print(" | ");
  Serial.println(gyro.z , 4);
#pragma endregion
}

