cmake_minimum_required(VERSION 3.21)

add_library(tinyusb INTERFACE)
add_library(tinyusb_usage INTERFACE)

set(TINYUSB_ROOT ${CMAKE_CURRENT_LIST_DIR}/src/tinyusb)
set(TINYUSB_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

target_include_directories(tinyusb_usage INTERFACE
  ${TINYUSB_SRC_DIR}
  ${TINYUSB_ROOT}/src
)

target_link_libraries(tinyusb_usage INTERFACE
  base_config
)

add_library(tinyusb_bin OBJECT EXCLUDE_FROM_ALL
  # TinyUSB core
  ${TINYUSB_ROOT}/src/tusb.c
  ${TINYUSB_ROOT}/src/common/tusb_fifo.c
  ${TINYUSB_ROOT}/src/device/usbd.c

  # CDC device only
  ${TINYUSB_ROOT}/src/class/cdc/cdc_device.c

  # STM32 DWC2 device port
  ${TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c
  ${TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c

  # Arduino layer
  ${TINYUSB_SRC_DIR}/TinyUSB_API.cpp
  ${TINYUSB_SRC_DIR}/TinyUSBSerial.cpp
  ${TINYUSB_SRC_DIR}/usb_descriptors.c
)

target_link_libraries(tinyusb_bin PUBLIC tinyusb_usage)

set_target_properties(tinyusb_bin PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED YES
  C_EXTENSIONS YES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
)

target_compile_definitions(tinyusb_bin PUBLIC
  USE_TINYUSB=1
)

target_link_libraries(tinyusb INTERFACE
  tinyusb_usage
  tinyusb_bin
  $<TARGET_OBJECTS:tinyusb_bin>
)
