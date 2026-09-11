#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// API appelée par le sketch/core
void TinyUSB_Device_Init(uint8_t rhport);
void TinyUSB_Device_Task(void);
void TinyUSB_Device_FlushCDC(void);

// API fournie par le core STM32Duino
void TinyUSB_Port_InitDevice(uint8_t rhport);
void TinyUSB_Port_EnterDFU(void);
uint8_t TinyUSB_Port_GetSerialNumber(uint8_t serial_id[16]);

#ifdef __cplusplus
}
#endif
