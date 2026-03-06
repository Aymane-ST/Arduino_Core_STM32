#include "I3C_Controller.h"
#include "lps22df.h"

#if defined(STM32H5xx)
  #define I3C_BUS       I3C1Bus
  #define I3C_SDA_PIN   PB7
  #define I3C_SCL_PIN   PB6
#elif defined(STM32U3xx)
  #define I3C_BUS       I3C2Bus
  #define I3C_SDA_PIN   PB14
  #define I3C_SCL_PIN   PB13_ALT1
#else
  #error "Unsupported STM32 family"
#endif

Lps22dfI3cCtx  g_lpsCtx;
lps22df_ctx_t  g_lpsDrvCtx;
uint8_t        g_dynAddr = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\n=== Test ENTDAA begin ===");

  if (!I3C_BUS.begin(I3C_SDA_PIN, I3C_SCL_PIN, 1000000)) {
    Serial.println("I3CBus.begin FAILED");
    while (1) {}
  }
  Serial.println("I3CBus initialized");

  // ENTDAA + init LPS22DF driver C
  if (!LPS22DF_I3C_EntdaaInit(I3C_BUS,
                              0x30,        // first DA to use
                              g_dynAddr,
                              g_lpsDrvCtx,
                              g_lpsCtx)) {
    Serial.println("LPS22DF ENTDAA init FAILED");
    while (1) {}
  }

  Serial.print("LPS22DF ready at DA=0x");
  Serial.println(g_dynAddr, HEX);
}

void loop()
{
  lps22df_data_t data;

  if (lps22df_data_get(&g_lpsDrvCtx, &data) == LPS22DF_OK) {
    Serial.print("P = ");
    Serial.print(data.pressure.hpa, 3);
    Serial.print(" hPa   |   T = ");
    Serial.print(data.heat.deg_c, 2);
    Serial.println(" °C");
  } else {
    Serial.println("lps22df_data_get FAILED");
  }

  delay(500);
}
