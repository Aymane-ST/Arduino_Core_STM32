# v3.21 implemented semantic changes regarding $<TARGET_OBJECTS:...>
# See https://cmake.org/cmake/help/v3.21/command/target_link_libraries.html#linking-object-libraries-via-target-objects
cmake_minimum_required(VERSION 3.21)

add_library(Adafruit_TinyUSB_Arduino INTERFACE)
add_library(Adafruit_TinyUSB_Arduino_usage INTERFACE)

target_include_directories(Adafruit_TinyUSB_Arduino_usage INTERFACE
  src
)

target_link_libraries(Adafruit_TinyUSB_Arduino_usage INTERFACE
  base_config
)

target_link_libraries(Adafruit_TinyUSB_Arduino INTERFACE Adafruit_TinyUSB_Arduino_usage)



add_library(Adafruit_TinyUSB_Arduino_bin OBJECT EXCLUDE_FROM_ALL
  # Arduino wrapper layer
  src/arduino/Adafruit_TinyUSB_API.cpp
  src/arduino/Adafruit_USBD_CDC.cpp
  src/arduino/Adafruit_USBD_Device.cpp
  src/arduino/Adafruit_USBD_Interface.cpp
  src/arduino/ports/stm32/Adafruit_TinyUSB_stm32.cpp

  # TinyUSB core
  src/tusb.c
  src/common/tusb_fifo.c
  src/device/usbd.c

  # Device classes
  src/class/cdc/cdc_device.c

  # STM32 low-level driver
  src/portable/synopsys/dwc2/dcd_dwc2.c
  src/portable/synopsys/dwc2/dwc2_common.c

  # If your STM32 target uses DWC2 instead of stm32_fsdev,
  # replace the 2 files above with the 2 lines below:
  # src/portable/synopsys/dwc2/dcd_dwc2.c
  # src/portable/synopsys/dwc2/dwc2_common.c
)

target_link_libraries(Adafruit_TinyUSB_Arduino_bin PUBLIC Adafruit_TinyUSB_Arduino_usage)

target_link_libraries(Adafruit_TinyUSB_Arduino INTERFACE
  Adafruit_TinyUSB_Arduino_bin
  $<TARGET_OBJECTS:Adafruit_TinyUSB_Arduino_bin>
)

