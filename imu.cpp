#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "SensorFusion.h"
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include "imu.h"

#define MS_TO_G 0.101972

void fusionTask(void *ctx)
{
  float deltat;

  SF fusion;

  // Init IMU
  Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(SPI0_CS0, SPI0_CS1);
  Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
  lsm.begin();

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

  while (1)
  {
    // Store fusion results
    if (xSemaphoreTake(imuSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
    {
      // Read sensor
      lsm.read();
      // Get a new sensor event
      lsm.getEvent(&a, &m, &g, &temp);

      // Sensor fusion
      deltat = fusion.deltatUpdate();
      fusion.MahonyUpdate(g.gyro.x * DEG_TO_RAD, g.gyro.y * DEG_TO_RAD, g.gyro.z * DEG_TO_RAD, a.acceleration.x * MS_TO_G, a.acceleration.y * MS_TO_G, a.acceleration.z * MS_TO_G, m.magnetic.x, m.magnetic.y, m.magnetic.z, deltat);

      pitch = fusion.getPitchRadians();
      roll = fusion.getRollRadians();
      yaw = fusion.getYawRadians();
      
      xSemaphoreGive(imuSemaphore);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
