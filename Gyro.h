// Gyro.h - MPU9250 gyroscope setup and angle tracking
#ifndef GYRO_H
#define GYRO_H

#include <MPU9250_WE.h>
#include <SPI.h>
#include "GlobalConfig.h"

SPIClass SPI_2(GYRO_MOSI, GYRO_MISO, GYRO_SCLK); 
MPU9250_WE myMPU = MPU9250_WE(&SPI_2, GYRO_CS, true);

float angle_x = 0;
float offset_x = 0;
unsigned long lastTimeGyro = 0;

void setupGyro() {
  pinMode(GYRO_CS, OUTPUT);
  digitalWrite(GYRO_CS, HIGH);
  SPI_2.begin();

  myMPU.init();
  myMPU.setGyrRange(MPU9250_GYRO_RANGE_2000); 
  myMPU.setGyrDLPF(MPU9250_DLPF_3);

  // Calibrate: average 2000 samples to find drift offset
  float sumX = 0;
  for(int i = 0; i < 2000; i++) {
    xyzFloat g = myMPU.getGyrValues();
    sumX += g.x;
    delay(1);
  }
  offset_x = sumX / 2000.0f; 
  
  lastTimeGyro = micros();
}

// Call this as fast as possible in the main loop
void updateGyro() {
  unsigned long currentTime = micros();
  float dt = (float)(currentTime - lastTimeGyro) / 1000000.0f;
  lastTimeGyro = currentTime;

  xyzFloat g = myMPU.getGyrValues();
  float Gx = g.x - offset_x;

  // Deadzone filter to prevent drift
  if (abs(Gx) > 0.5) { 
    angle_x -= Gx * dt; 
  }
}

#endif // GYRO_H
