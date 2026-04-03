#include "I3C.h"
#include "LPS22DFSensor.h"

#define I3C_BUS I3C1Bus

LPS22DFSensor sensor(&I3C_BUS, LPS22DF_I3C_ADD_H, 0x30);
static volatile bool ibiPending = false;



void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  I3C_BUS.begin(I3C_SDA, I3C_SCL, 1000000U);
  I3C_BUS.onIbi([](const I3CControllerIbiInfo&, void*) {
    ibiPending = true;
  },
                nullptr);

  sensor.begin();
  sensor.SetOutputDataRate(10.0f);
  sensor.ConfigureDataReadyOnI3cIbi();
  sensor.EnableIbiOnBus();
  sensor.Enable();

  Serial.println("Waiting for IBI  ...");
}

void loop() {
  if (!ibiPending && !I3C_BUS.hasIbi()) {
    return;
  }

  ibiPending = false;

  I3CControllerIbiInfo ibi{};
  if (!I3C_BUS.readIbi(ibi)) {
    return;
  }

  float p, t;
  if (sensor.GetPressure(&p) == LPS22DF_OK && sensor.GetTemperature(&t) == LPS22DF_OK) {
    Serial.print("  P = ");
    Serial.print(p, 2);
    Serial.print(" hPa, T = ");
    Serial.print(t, 1);
    Serial.println(" C");
  }
}
