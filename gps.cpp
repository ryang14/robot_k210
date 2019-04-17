#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "gps.h"

void gpsTask(void *ctx)
{
    Serial2.begin(9600, 9, 10);

    while (1)
    {
        if (xSemaphoreTake(gpsSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
        {
            while(Serial2.available()) gps.encode(Serial2.read());

            xSemaphoreGive(gpsSemaphore);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}