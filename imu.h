#ifndef IMU_H
#define IMU_H

#include <semphr.h>
#include <Adafruit_Sensor.h>

inline SemaphoreHandle_t imuSemaphore = NULL;
inline sensors_event_t a, m, g, temp;
inline float pitch, roll, yaw;

void fusionTask(void *ctx);

#endif
