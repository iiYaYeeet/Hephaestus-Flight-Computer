#include "Wire.h"
#include "MPU6050.h"
#include "BMP280.h"

const int buzzer = 8;
const int led-rxtx = 6;
const int led-on = 9;


void setup() 
{
  //set pins up
  pinMode(buzzer, OUTPUT);
  pinMode(led-rxtx, OUTPUT);
  pinMode(led-on, OUTPUT);

  Wire.begin()
}

void loop() 
{

}