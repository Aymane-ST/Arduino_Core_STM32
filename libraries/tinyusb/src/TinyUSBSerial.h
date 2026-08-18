#pragma once

#include <stdint.h>
#include <stddef.h>
#include "Stream.h"

class TinyUSBSerial : public Stream {
public:
  TinyUSBSerial();

  void begin(unsigned long baud = 115200);
  void end(void);

  bool connected(void) const;
  bool mounted(void) const;
  operator bool();

  int available(void) override;
  int peek(void) override;
  int read(void) override;

  size_t read(uint8_t* buffer, size_t size);
  size_t readBytes(char* buffer, size_t length);

  void flush(void) override;
  size_t write(uint8_t ch) override;
  size_t write(const uint8_t* buffer, size_t size);

  int availableForWrite(void);

  using Print::write;
};
