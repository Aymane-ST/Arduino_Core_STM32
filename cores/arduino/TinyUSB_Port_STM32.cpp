#if defined(USE_TINYUSB)

#include "Arduino.h"
#include "TinyUSB_API.h"

#ifndef TINYUSB_IRQ_PRIO
  #ifdef USBD_IRQ_PRIO
    #define TINYUSB_IRQ_PRIO USBD_IRQ_PRIO
  #else
    #define TINYUSB_IRQ_PRIO 1
  #endif
#endif

#ifndef TINYUSB_IRQ_SUBPRIO
  #ifdef USBD_IRQ_SUBPRIO
    #define TINYUSB_IRQ_SUBPRIO USBD_IRQ_SUBPRIO
  #else
    #define TINYUSB_IRQ_SUBPRIO 0
  #endif
#endif

// Sélec rapide du contrôleur USB selon les flags du core/menu
#if defined(USE_USB_HS)
  #define TINYUSB_USE_HS   1
  #define TINYUSB_RHPORT   1
#else
  #define TINYUSB_USE_HS   0
  #define TINYUSB_RHPORT   0
#endif

extern "C" {

// vrai symbole TinyUSB
void tusb_int_handler(uint8_t rhport, bool in_isr);

static void tinyusb_port_init_fs(void) {
#if defined(USB_OTG_FS)
  const PinMap *map = PinMap_USB_OTG_FS;
  while (map != nullptr && map->pin != NC) {
    pin_function(map->pin, map->function);
    map++;
  }

  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  HAL_NVIC_SetPriority(OTG_FS_IRQn, TINYUSB_IRQ_PRIO, TINYUSB_IRQ_SUBPRIO);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif
}

static void tinyusb_port_init_hs(void) {
#if defined(USB_OTG_HS)
  const PinMap *map = PinMap_USB_OTG_HS;
  while (map != nullptr && map->pin != NC) {
    pin_function(map->pin, map->function);
    map++;
  }

  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

  #if !defined(USE_USB_HS_IN_FS)
    #ifdef __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE
      __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
    #endif
  #endif

  HAL_NVIC_SetPriority(OTG_HS_IRQn, TINYUSB_IRQ_PRIO, TINYUSB_IRQ_SUBPRIO);
  HAL_NVIC_EnableIRQ(OTG_HS_IRQn);

  USB_OTG_HS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
#endif
}

void TinyUSB_Port_InitDevice(uint8_t rhport) {
  (void) rhport;


  if (TINYUSB_USE_HS) {
    tinyusb_port_init_hs();
  } else {
    tinyusb_port_init_fs();
  }
}

void TinyUSB_Port_EnterDFU(void) {
}

uint8_t TinyUSB_Port_GetSerialNumber(uint8_t serial_id[16]) {
  uint32_t uid0 = HAL_GetUIDw0();
  uint32_t uid1 = HAL_GetUIDw1();
  uint32_t uid2 = HAL_GetUIDw2();

  serial_id[0]  = (uid0 >>  0) & 0xFF;
  serial_id[1]  = (uid0 >>  8) & 0xFF;
  serial_id[2]  = (uid0 >> 16) & 0xFF;
  serial_id[3]  = (uid0 >> 24) & 0xFF;

  serial_id[4]  = (uid1 >>  0) & 0xFF;
  serial_id[5]  = (uid1 >>  8) & 0xFF;
  serial_id[6]  = (uid1 >> 16) & 0xFF;
  serial_id[7]  = (uid1 >> 24) & 0xFF;

  serial_id[8]  = (uid2 >>  0) & 0xFF;
  serial_id[9]  = (uid2 >>  8) & 0xFF;
  serial_id[10] = (uid2 >> 16) & 0xFF;
  serial_id[11] = (uid2 >> 24) & 0xFF;

  return 12;
}

#if defined(USB_OTG_FS)
void OTG_FS_IRQHandler(void) {
  tusb_int_handler(0, true);
}
#endif

#if defined(USB_OTG_HS)
void OTG_HS_IRQHandler(void) {
  tusb_int_handler(1, true);
}
#endif

void yield(void) {
  TinyUSB_Device_Task();
  TinyUSB_Device_FlushCDC();
}

} // extern "C"

#endif
