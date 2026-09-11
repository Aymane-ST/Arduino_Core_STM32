cmake_minimum_required(VERSION 3.21)

add_library(tinyusb INTERFACE)
add_library(tinyusb_usage INTERFACE)

set(TINYUSB_ROOT ${CMAKE_CURRENT_LIST_DIR}/src)
set(TINYUSB_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src/wrapper)

# Export TinyUSB public headers globally to all Arduino-related targets
# since WSerial.h includes TinyUSBSerial.h when USE_TINYUSB is enabled.
target_include_directories(base_config INTERFACE
  ${TINYUSB_ROOT}
  ${TINYUSB_SRC_DIR}
)

target_include_directories(tinyusb_usage INTERFACE
  ${TINYUSB_ROOT}
  ${TINYUSB_SRC_DIR}
)

target_link_libraries(tinyusb_usage INTERFACE
  base_config
)

# add_library(tinyusb_bin OBJECT EXCLUDE_FROM_ALL
#   # TinyUSB core
#   ${TINYUSB_ROOT}/tusb.c
#   ${TINYUSB_ROOT}/common/tusb_fifo.c
#   ${TINYUSB_ROOT}/device/usbd.c

#   # CDC device only
#   ${TINYUSB_ROOT}/class/cdc/cdc_device.c

#   # STM32 DWC2 device port
#   ${TINYUSB_ROOT}/portable/synopsys/dwc2/dwc2_common.c
#   ${TINYUSB_ROOT}/portable/synopsys/dwc2/dcd_dwc2.c

#   # Arduino layer
#   ${TINYUSB_SRC_DIR}/TinyUSB_API.cpp
#   ${TINYUSB_SRC_DIR}/TinyUSBSerial.cpp
#   ${TINYUSB_SRC_DIR}/usb_descriptors.c
# )

add_library(tinyusb_bin OBJECT EXCLUDE_FROM_ALL
  # TinyUSB core
  ${TINYUSB_ROOT}/mw-tusb.c
  ${TINYUSB_ROOT}/mw-tusb_fifo.c
  ${TINYUSB_ROOT}/mw-usbd.c

  # CDC device only
  ${TINYUSB_ROOT}/mw-cdc_device.c

  # STM32 DWC2 device port
  ${TINYUSB_ROOT}/mw-dwc2_common.c
  ${TINYUSB_ROOT}/mw-dcd_dwc2.c

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
  $<TARGET_OBJECTS:tinyusb_bin>
)
