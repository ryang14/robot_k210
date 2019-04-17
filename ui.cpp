#include <FreeRTOS.h>
#include <task.h>
#include <SPI.h>
#include <Sipeed_ST7789.h>
#include "imu.h"

void lcdTask(void *ctx)
{
  SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
  Sipeed_ST7789 lcd(320, 240, spi_);

  lcd.begin(15000000, COLOR_BLACK);
  lcd.setRotation(0);

  while (1)
  {
    if (xSemaphoreTake(imuSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
    {
      lcd.fillScreen(COLOR_BLACK);
      lcd.setTextSize(2);
      lcd.setTextColor(COLOR_WHITE);
      lcd.setCursor(1, 1);
      lcd.print("Pitch:\t");
      lcd.println(pitch);
      lcd.print("Roll:\t");
      lcd.println(roll);
      lcd.print("Yaw:\t");
      lcd.println(yaw);

      lcd.print("Accel X: ");
      lcd.print(a.acceleration.x);
      lcd.println(" m/s^2");
      lcd.print("Y: ");
      lcd.print(a.acceleration.y);
      lcd.println(" m/s^2 ");
      lcd.print("Z: ");
      lcd.print(a.acceleration.z);
      lcd.println(" m/s^2 ");

      lcd.print("Mag X: ");
      lcd.print(m.magnetic.x);
      lcd.println(" gauss");
      lcd.print("Y: ");
      lcd.print(m.magnetic.y);
      lcd.println(" gauss");
      lcd.print("Z: ");
      lcd.print(m.magnetic.z);
      lcd.println(" gauss");

      lcd.print("Gyro X: ");
      lcd.print(g.gyro.x);
      lcd.println(" dps");
      lcd.print("Y: ");
      lcd.print(g.gyro.y);
      lcd.println(" dps");
      lcd.print("Z: ");
      lcd.print(g.gyro.z);
      lcd.println(" dps");

      xSemaphoreGive(imuSemaphore);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
