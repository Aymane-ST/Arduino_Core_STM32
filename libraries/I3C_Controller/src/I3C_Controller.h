#pragma once

#include <Arduino.h>
#if defined(STM32H5xx)
  #include "stm32h5xx_util_i3c.h"
#elif defined(STM32U3xx)
  #include "stm32u3xx_util_i3c.h"
#else
  #error "I3C: unsupported STM32 family"
#endif
#include "stm32_def.h"

// -----------------------------------------------------------------------------
// ENTDAA discovered device descriptor (BCR+DCR+PID packed payload)
// -----------------------------------------------------------------------------
struct I3CDiscoveredDevice {
  uint8_t  dynAddr;     // Assigned dynamic address (7-bit)
  uint64_t dcrBcrPid;   // Raw ENTDAA payload (BCR + DCR + PID packed)
};

// -----------------------------------------------------------------------------
// I3CBus: lightweight I3C controller wrapper for STM32 (H5/U3)
//
// Features (aligned with AN5879):
// - Bus / clock / GPIO initialization
// - Private transfers (I3C / legacy I2C-like)
// - Legacy I2C device readiness checks on an I3C bus
// - CCC broadcast / direct (RSTDAA, DISEC, SETDASA)
// - Dynamic addressing via ENTDAA + SetDynAddr (HAL_I3C_Ctrl_DynAddrAssign)
// - Bus timing configuration via I3C_CtrlTimingComputation
// -----------------------------------------------------------------------------
class I3CBus {
  public:
    explicit I3CBus(I3C_TypeDef *inst = I3C1) : _instance(inst) {}

    bool begin(uint32_t sda, uint32_t scl, uint32_t freq = 1000000);
    bool begin(uint32_t freq = 1000000);

    // ---------------------------------------------------------------------------
    // Private transfers (I3C / legacy I2C-like)
    // ---------------------------------------------------------------------------
    // 8-bit register read/write, I2C-style transactions
    int writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value, uint32_t timeout = 1000);
    int readRegister(uint8_t devAddr, uint8_t reg, uint8_t &value, uint32_t timeout = 1000);

    int writeRegisterBlock(uint8_t devAddr, uint8_t reg, const uint8_t *pData, size_t len, uint32_t timeout = 1000);
    int readRegisterBlock(uint8_t devAddr, uint8_t reg, uint8_t *pData, size_t len, uint32_t timeout = 1000);

    // ---------------------------------------------------------------------------
    //  Device readiness checks
    // ---------------------------------------------------------------------------
    bool isI2CDeviceReady(uint8_t addr, uint32_t trials  = 3, uint32_t timeout = 1000);
    bool isI3CDeviceReady(uint8_t dynAddr, uint32_t trials  = 3, uint32_t timeout = 1000);

    // ---------------------------------------------------------------------------
    // CCC broadcast & direct commands   // api naming to be discussed !!
    // ---------------------------------------------------------------------------
    // Broadcast CCC:
    //  - RSTDAA (0x06): reset dynamic addresses of all I3C devices
    int rstdaaOnce();
    //  - DISEC (0x01): disable some target capabilities (HotJoin, events, etc.)
    int disecOnce(const uint8_t *pCCCData, uint16_t length);

    // Direct CCC:
    //  - SETDASA (0x87): assign a dynamic address to a target known by its static address
    //    staticAddr: 7-bit static address
    //    dynAddr:   7-bit dynamic address to assign
    int setdasa(uint8_t staticAddr, uint8_t dynAddr);

    // ---------------------------------------------------------------------------
    // Dynamic addressing (ENTDAA + SetDynAddr)
    // ---------------------------------------------------------------------------
    // ENTDAA: run dynamic address assignment sequence (0x7E)
    //  - devices: output array of discovered devices (dynAddr + ENTDAA payload)
    //  - maxDevices: size of devices[] array
    //  - firstDynAddr: first dynamic address to use (default 0x10)
    //  - timeout: HAL polling timeout in ms
    int entdaa(I3CDiscoveredDevice *devices, size_t maxDevices, uint8_t firstDynAddr = 0x10, uint32_t timeout = 1000);

    // ---------------------------------------------------------------------------
    // Bus timing / frequency (via stm32xx_util_i3c I3C_CtrlTimingComputation)
    // ---------------------------------------------------------------------------
    // hz: desired I3C push-pull bus frequency (Hz). 0 = keep current timings.
    int setBusFrequency(uint32_t hz);

    I3C_HandleTypeDef *halHandle()
    {
      return &_hi3c;
    }

  private:
    I3C_HandleTypeDef _hi3c{};
    bool _initialized = false;
    I3C_TypeDef *_instance;

    PinName _sdaPin = NC;
    PinName _sclPin = NC;

    bool initClocks();
    bool initGPIO();
};

// Global instances
#if defined(I3C1)
  extern I3CBus I3C1Bus;
#endif
#if defined(I3C2)
  extern I3CBus I3C2Bus;
#endif
