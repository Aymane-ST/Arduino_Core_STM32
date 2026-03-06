#include "lps22df.h"

// limit the discovery
static const size_t MAX_I3C_DEVICES = 4;

// Low-level callbacks for C driver
static int32_t lps_i3c_write(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
  auto *ctx = static_cast<Lps22dfI3cCtx*>(handle);
  int rc = ctx->bus->writeRegisterBlock(ctx->dynAddr, reg, buf, len, 1000);
  return (rc == 0) ? 0 : -1;
}

static int32_t lps_i3c_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
  auto *ctx = static_cast<Lps22dfI3cCtx*>(handle);
  int rc = ctx->bus->readRegisterBlock(ctx->dynAddr, reg, buf, len, 1000);
  return (rc == 0) ? 0 : -1;
}

bool LPS22DF_I3C_EntdaaInit(I3CBus &bus,
                            uint8_t firstDynAddr,
                            uint8_t &outDynAddr,
                            lps22df_ctx_t &outDrvCtx,
                            Lps22dfI3cCtx &outCtx)
{
  outDynAddr = 0;

  // 1) Optional: global RSTDAA
  bus.rstdaaOnce();

    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
    bus.isI2CDeviceReady(addr, 2, 10);
  }


  // 2) ENTDAA: discovery of I3C devices on the bus
  I3CDiscoveredDevice devices[MAX_I3C_DEVICES];
  int n = bus.entdaa(devices, MAX_I3C_DEVICES, firstDynAddr, 1000);
  if (n < 0) {
    Serial.print("ENTDAA FAILED, rc=");
    Serial.println(n);
    return false;
  }

  Serial.print("ENTDAA found ");
  Serial.print(n);
  Serial.println(" I3C device(s)");

  // 3) Identify the LPS22DF via WHO_AM_I
  for (int i = 0; i < n; ++i) {
    uint8_t id = 0;
    int rc = bus.readRegister(devices[i].dynAddr, LPS22DF_WHO_AM_I, id, 1000);
    Serial.print("WHO_AM_I @DA 0x"); Serial.print(devices[i].dynAddr, HEX);
    Serial.print(" rc="); Serial.print(rc);
    Serial.print(" id=0x"); Serial.println(id, HEX);

    if (rc == 0 && id == LPS22DF_ID) {
      outDynAddr = devices[i].dynAddr;
      Serial.print("LPS22DF detected at DA=0x");
      Serial.println(outDynAddr, HEX);
      break;
    }
  }

  if (outDynAddr == 0) {
    Serial.println("No LPS22DF detected via ENTDAA");
    return false;
  }

  // 4) Prepare contexts for the C driver
  outCtx.bus     = &bus;
  outCtx.dynAddr = outDynAddr;

  outDrvCtx.write_reg = lps_i3c_write;
  outDrvCtx.read_reg  = lps_i3c_read;
  outDrvCtx.handle    = &outCtx;

  // 5) Recommended init (BDU + IF_INC etc.)
  if (lps22df_init_set(&outDrvCtx, LPS22DF_DRV_RDY) != LPS22DF_OK) {
    Serial.println("lps22df_init_set FAILED");
    return false;
  }

  // 6) Configure mode (ODR, AVG, LPF)
  lps22df_md_t md;
  md.odr = lps22df_md_t::LPS22DF_25Hz;
  md.avg = lps22df_md_t::LPS22DF_4_AVG;
  md.lpf = lps22df_md_t::LPS22DF_LPF_ODR_DIV_4;

  if (lps22df_mode_set(&outDrvCtx, &md) != LPS22DF_OK) {
    Serial.println("lps22df_mode_set FAILED");
    return false;
  }

  Serial.println("LPS22DF configured via ENTDAA path.");
  return true;
}
