#include "tusb_option.h"

#if CFG_TUD_ENABLED

#include "TinyUSBSerial.h"
#include "TinyUSB_API.h"
#include "tusb.h"
#include "Arduino.h"

TinyUSBSerial SerialTinyUSB;

TinyUSBSerial::TinyUSBSerial() {}

void TinyUSBSerial::begin(unsigned long baud) {
  (void) baud;

  if (!tud_inited()) {
    TinyUSB_Device_Init(0);
  }
}

void TinyUSBSerial::end(void) {
}

bool TinyUSBSerial::connected(void) const {
  return tud_cdc_connected();
}

bool TinyUSBSerial::mounted(void) const {
  return tud_mounted();
}

TinyUSBSerial::operator bool() {
  TinyUSB_Device_Task();
  return tud_cdc_connected();
}

int TinyUSBSerial::available(void) {
  TinyUSB_Device_Task();
  return (int) tud_cdc_available();
}

int TinyUSBSerial::peek(void) {
  TinyUSB_Device_Task();
  uint8_t ch;
  return tud_cdc_peek(&ch) ? (int) ch : -1;
}

int TinyUSBSerial::read(void) {
  TinyUSB_Device_Task();
  return tud_cdc_read_char();
}

size_t TinyUSBSerial::read(uint8_t* buffer, size_t size) {
  TinyUSB_Device_Task();
  return tud_cdc_read(buffer, (uint32_t) size);
}

size_t TinyUSBSerial::readBytes(char* buffer, size_t length) {
  size_t count = 0;
  unsigned long start = millis();

  while (count < length) {
    TinyUSB_Device_Task();

    uint32_t avail = tud_cdc_available();
    if (avail > 0) {
      uint32_t remaining = (uint32_t)(length - count);
      uint32_t to_read = (avail > remaining) ? remaining : avail;
      uint32_t n = tud_cdc_read((uint8_t*)buffer + count, to_read);
      count += n;
      start = millis();
    } else {
      if ((millis() - start) >= _timeout) {
        break;
      }
    }
  }

  return count;
}

void TinyUSBSerial::flush(void) {
  TinyUSB_Device_Task();
  tud_cdc_write_flush();
}

size_t TinyUSBSerial::write(uint8_t ch) {
  return write(&ch, 1);
}

size_t TinyUSBSerial::write(const uint8_t* buffer, size_t size) {
  size_t remain = size;

  while (remain && tud_cdc_connected()) {
    TinyUSB_Device_Task();

    size_t n = tud_cdc_write(buffer, (uint32_t) remain);
    remain -= n;
    buffer += n;

    if (remain) {
      TinyUSB_Device_Task();
      tud_cdc_write_flush();
    }
  }

  return size - remain;
}

int TinyUSBSerial::availableForWrite(void) {
  TinyUSB_Device_Task();
  return (int) tud_cdc_write_available();
}

#endif
