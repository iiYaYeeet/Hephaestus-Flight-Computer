#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"

const int buzzer = 8;
const int ledrxtx = 6;
const int ledon = 9;
const int goled = 13;
//Found device at 0x68
//Found device at 0x69
//Found device at 0x76

uint8_t FIFOBuffer[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


//MPU1
MPU6050 mpu1(0x68, &Wire2);
bool DMPReady1 = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus1;   // Holds actual interrupt status byte from MPU
uint8_t devStatus1;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // Expected DMP packet size (default is 42 bytes)
/*---Orientation/Motion Variables---*/ 
Quaternion q1;           // [w, x, y, z]         Quaternion container
VectorInt16 aa1;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy1;         // [x, y, z]            Gyro sensor measurements


//MPU2
MPU6050 mpu2(0x69, &Wire2);
bool DMPReady2 = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus2;   // Holds actual interrupt status byte from MPU
uint8_t devStatus2;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize2;    // Expected DMP packet size (default is 42 bytes)/*---Orientation/Motion Variables---*/ 
Quaternion q2;           // [w, x, y, z]         Quaternion container
VectorInt16 aa2;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy2;         // [x, y, z]            Gyro sensor measurements

#define CHIP_ID_REG 0xD0
//BMP
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() 
{

 //SERIAL & I2C
  Serial.begin(9600);
  Wire2.begin();
  
//PINOUT
  pinMode(buzzer, OUTPUT);
  pinMode(ledrxtx, OUTPUT);
  pinMode(ledon, OUTPUT);
  pinMode(goled, OUTPUT);
  Serial.println("Initializing pins");

//INITALIZE BMP
  Serial.println("Initializing BMP");
  bmp.begin(0x76, &Wire2);


//INITALIZE MPU1
  Serial.println("Initializing MPU1");
  mpu1.initialize();
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

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

//INITALIZE MPU2
  Serial.println("Initializing MPU2");
  mpu2.initialize();
  mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_250);


  mpu2.CalibrateAccel(30);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu2.CalibrateGyro(30);
  Serial.println("These are the Active offsets: ");
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

//BMP Test
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

//MPU Test
  if (!mpu1.testConnection()) 
  {
    Serial.println("MPU1 Connection Failed");
  }
  else
  {
    Serial.println("MPU1 Online");
  }
  if (!mpu2.testConnection()) 
  {
    Serial.println("MPU2 Connection Failed");
  }
  else
  {
    Serial.println("MPU2 Online");
  }
}

void loop()
{
  int16_t ax1, ay1, az1, gx1, gy1, gz1; //MPU1
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  int16_t ax2, ay2, az2, gx2, gy2, gz2; //MPU2
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);


  int16_t avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz;

  avg_ax = avg(ax1,ax2);
  avg_ay = avg(ay1,ay2);
  avg_az = avg(az1,az2);
  avg_gx = avg(gx1,gx2);
  avg_gy = avg(gy1,gy2);
  avg_gz = avg(gz1,gz2);

  Serial.print("AccelX: "); 
  Serial.println(avg_ax);
  Serial.print("AccelY: "); 
  Serial.println(avg_ay);
  Serial.print("AccelZ: "); 
  Serial.println(avg_az);
  Serial.print("GyroX: "); 
  Serial.println(avg_gx/60);
  Serial.print("GyroY: "); 
  Serial.println(avg_gy/60);
  Serial.print("GyroZ: "); 
  Serial.println(avg_gz/60);
  delay(50);

  /*Serial.print("MPU1 AccelX: "); Serial.print(ax1);
  Serial.print(" AccelY: "); Serial.print(ay1);
  Serial.print(" AccelZ: "); Serial.print(az1);
  Serial.print(" GyroX: "); Serial.print(gx1);
  Serial.print(" GyroY: "); Serial.print(gy1);
  Serial.print(" GyroZ: "); Serial.println(gz1);

  Serial.print("MPU2 AccelX: "); Serial.print(ax2);
  Serial.print(" AccelY: "); Serial.print(ay2);
  Serial.print(" AccelZ: "); Serial.print(az2);
  Serial.print(" GyroX: "); Serial.print(gx2);
  Serial.print(" GyroY: "); Serial.print(gy2);
  Serial.print(" GyroZ: "); Serial.println(gz2);*/
}

int16_t avg(int16_t val1, int16_t val2)
{
  int16_t total = val1+val2;
  return total/2;
}

