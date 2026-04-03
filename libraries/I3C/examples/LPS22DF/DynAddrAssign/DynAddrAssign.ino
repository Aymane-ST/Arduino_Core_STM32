#include "I3C.h"
#include "LPS22DFSensor.h"

#define I3C_BUS I3C1Bus

LPS22DFSensor sensor(&I3C_BUS, 0x00);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LPS22DF DAA ===");

  if (!I3C_BUS.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  I3CDiscoveredDevice devices[8]{};
  size_t found = 0;

  if (I3C_BUS.discover(devices, 8, &found)) {
    Serial.println("discover() failed");
    while (1) {}
  }

  uint8_t lpsDynAddr = 0U;

  for (size_t i = 0; i < found; ++i) {
    Serial.println(devices[i].pid,HEX);
    if (devices[i].pid == LPS22DF_I3C_PID_H) {
      lpsDynAddr = devices[i].dynAddr;
      break;
    }
  }

  sensor.set_address(lpsDynAddr);

  if (lpsDynAddr == 0U) {
    Serial.println("Sensor not found failed");
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
