#include "tinyusb/src/tusb_option.h"

#if CFG_TUD_ENABLED

#include "TinyUSB_API.h"
#include "Arduino.h"
#include "tinyusb/src/tusb.h"

extern "C" {

uint32_t tusb_time_millis_api(void) {
  return millis();
}

void TinyUSB_Device_Init(uint8_t rhport) {
  static bool initialized = false;
  if (initialized || tud_inited()) {
    return;
  }

  TinyUSB_Port_InitDevice(rhport);

  tusb_rhport_init_t dev_init = {
    .role  = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };

  tusb_init(rhport, &dev_init);
  initialized = true;
}

void TinyUSB_Device_Task(void) {
  if (tud_inited()) {
    tud_task();
  }
}

void TinyUSB_Device_FlushCDC(void) {
  if (tud_inited()) {
    tud_cdc_write_flush();
  }
}

} // extern "C"

#endif
