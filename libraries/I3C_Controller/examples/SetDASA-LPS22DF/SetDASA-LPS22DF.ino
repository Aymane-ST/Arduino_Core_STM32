#include <Arduino.h>
#include "I3C_Controller.h"
#include "LPS22DFSensor.h"

static const uint8_t LPS22DF_STATIC_ADDR_7BIT = 0x5D;
static const uint8_t LPS22DF_DYN_ADDR_7BIT    = 0x30;

#if defined(STM32H5xx)
  LPS22DFSensor ps(&I3C1Bus, LPS22DF_STATIC_ADDR_7BIT, LPS22DF_DYN_ADDR_7BIT);
  #define I3C_BUS I3C1Bus
  #define I3C_SCL PB6
  #define I3C_SDA PB7
#elif defined(STM32U3xx)
  LPS22DFSensor ps(&I3C2Bus, LPS22DF_STATIC_ADDR_7BIT, LPS22DF_DYN_ADDR_7BIT);
  #define I3C_BUS I3C2Bus
  #define I3C_SCL PB13_ALT1
  #define I3C_SDA PB14
#endif

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\n=== Test SetDASA Begin ===");

#if defined(STM32H5xx)
  if (!I3C_BUS.begin(I3C_SDA, I3C_SCL, 1000000)) {
#elif defined(STM32U3xx)
  if (!I3C_BUS.begin(I3C_SDA, I3C_SCL, 1000000)) {
#endif

    Serial.println("I3C_BUS.begin FAILED");
    while (1) {}
  }


  Serial.println("I3CBus initialized");

  if (ps.begin() != LPS22DF_OK) {
    Serial.println("LPS22DF begin FAILED");
    while (1) {}
  }
  Serial.println("LPS22DF begin OK");

  if (ps.Enable() != LPS22DF_OK) {
    Serial.println("LPS22DF Enable FAILED");
    while (1) {}
  }
  Serial.println("LPS22DF Enable OK");

}


void loop()
{
  float p, t;

  if (ps.GetPressure(&p) == LPS22DF_OK &&
      ps.GetTemperature(&t) == LPS22DF_OK) {
    Serial.print("P = ");
    Serial.print(p, 3);
    Serial.print(" hPa   |   T = ");
    Serial.print(t, 2);
    Serial.println(" °C");
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
