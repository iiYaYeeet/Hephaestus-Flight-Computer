#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

const int buzzer = 8;
const int ledrxtx = 6;
const int ledon = 9;
//Found device at 0x68
//Found device at 0x69
//Found device at 0x76
MPU6050 mpu1(0x68, &Wire2);
MPU6050 mpu2(0x69, &Wire2);

#define CHIP_ID_REG 0xD0
//BMP
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() 
{
  
  //set pins up
  pinMode(buzzer, OUTPUT);
  pinMode(ledrxtx, OUTPUT);
  pinMode(ledon, OUTPUT);

  Serial.begin(115200);
  Wire2.begin();

  bmp.begin(0x76, &Wire2);
  
  Serial.println("Initializing MPU1");
  mpu1.initialize();
  

  Serial.println("Initializing MPU2");
  mpu2.initialize();

  Wire2.beginTransmission(0x76);
  Wire2.write(CHIP_ID_REG);
  Wire2.endTransmission();

  // Request 1 byte from the BMP280
  Wire2.requestFrom(0x76, 1);
  if (Wire2.available()) {
    byte id = Wire2.read();
    Serial.print("Chip ID: 0x");
    Serial.println(id, HEX);

    if (id == 0x58) {
      Serial.println("BMP280 detected!");
    } else {
      Serial.println("Unexpected chip.");
    }
  } else {
    Serial.println("No response from BMP280.");
  }

  if (!mpu1.testConnection()) 
  {
    Serial.println("MPU1 Connection Failed");
  }
  else
  {
    Serial.println("YAYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
  }
  if (!mpu2.testConnection()) 
  {
    Serial.println("MPU2 Connection Failed");
  }
  else
  {
    Serial.println("YAYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
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

  Serial.print("MPU_avg AccelX: "); 
  Serial.print(avg_ax);
  Serial.print(" AccelY: "); 
  Serial.print(avg_ay);
  Serial.print(" AccelZ: "); 
  Serial.print(avg_az);
  Serial.print(" GyroX: "); 
  Serial.print(avg_gx);
  Serial.print(" GyroY: "); 
  Serial.print(avg_gy);
  Serial.print(" GyroZ: "); 
  Serial.println(avg_gz);
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

