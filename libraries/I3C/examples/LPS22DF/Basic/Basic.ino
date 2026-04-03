#include "I3C.h"
#include "LPS22DFSensor.h"

#define I3C_BUS I3C1Bus

static const uint8_t LPS22DF_DYN_ADDR = 0x30;

LPS22DFSensor sensor(&I3C_BUS, LPS22DF_I3C_ADD_H, LPS22DF_DYN_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22DF SETDASA ===");

  if (!I3C_BUS.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C_BUS.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }

  if (sensor.begin() != LPS22DF_OK) {
    Serial.println("sensor.begin() failed");
    while (1) {}
  }

  if (sensor.Enable() != LPS22DF_OK) {
    Serial.println("sensor.Enable() failed");
    while (1) {}
  }

  Serial.println("LPS22DF ready");
}

void loop() {
  float p = 0.0f;
  float t = 0.0f;

  if (sensor.GetPressure(&p) == LPS22DF_OK && sensor.GetTemperature(&t) == LPS22DF_OK) {
    Serial.print("P = ");
    Serial.print(p, 2);
    Serial.print(" hPa   T = ");
    Serial.print(t, 1);
    Serial.println(" C");
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
