#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>

inline SemaphoreHandle_t gpsSemaphore = NULL;

inline TinyGPSPlus gps;

void gpsTask(void *ctx);

#endif // !GPS_H