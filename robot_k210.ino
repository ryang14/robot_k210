#include "bsp.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "mavlinkTask.h"
#include "imu.h"
#include "ui.h"
#include "gps.h"

int core1_function(void *ctx)
{
  vTaskStartScheduler();
}

void setup()
{
  Serial.begin(115200);
  imuSemaphore = xSemaphoreCreateMutex();
  gpsSemaphore = xSemaphoreCreateMutex();
  xTaskCreate(mavlinkTask, "mavlink", 2048, NULL, 3, NULL);
  xTaskCreate(lcdTask, "LCD", 2048, NULL, 2, NULL);
  xTaskCreate(fusionTask, "fusion", 2048, NULL, 1, NULL);
  xTaskCreate(gpsTask, "gps", 2048, NULL, 1, NULL);
  register_core1(core1_function, NULL);
  vTaskStartScheduler();
}

void loop() {}
