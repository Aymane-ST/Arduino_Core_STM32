#include "I3C_Controller.h"
#include <cstring>

#if defined(I3C1)
  I3CBus I3C1Bus(I3C1);
#endif

#if defined(I3C2)
  I3CBus I3C2Bus(I3C2);
#endif


#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

// ============================================================================
// 1. GPIO & clock initialization (AN5879: I3C controller setup)
// ============================================================================

bool I3CBus::initGPIO()
{
  if (_sdaPin == NC || _sclPin == NC) {
    return false;
  }

  // Check that pins are mapped to an I3C instance via PinMap
  void *periphSDA = pinmap_peripheral(_sdaPin, PinMap_I3C_SDA);
  void *periphSCL = pinmap_peripheral(_sclPin, PinMap_I3C_SCL);
  void *periph    = pinmap_merge_peripheral(periphSDA, periphSCL);

  if (periph == (void *)NP) {
    // Invalid SDA/SCL combination for I3C on this variant
    return false;
  }

  // Ensure these pins map to the selected instance
  PinName sdaInst  = pinmap_pin((void *) _instance, PinMap_I3C_SDA);
  PinName sclInst  = pinmap_pin((void *) _instance, PinMap_I3C_SCL);
  (void)sdaInst;
  (void)sclInst;

  uint32_t sdaPortIndex = STM_PORT(_sdaPin);
  uint32_t sclPortIndex = STM_PORT(_sclPin);
  uint32_t sdaPinIndex  = STM_PIN(_sdaPin);
  uint32_t sclPinIndex  = STM_PIN(_sclPin);

  GPIO_TypeDef *sdaPort = get_GPIO_Port(sdaPortIndex);
  GPIO_TypeDef *sclPort = get_GPIO_Port(sclPortIndex);

  if (!sdaPort || !sclPort || sdaPort != sclPort) {
    // Only same-port SDA/SCL supported here
    return false;
  }

  uint32_t sdaMask = (1U << sdaPinIndex);
  uint32_t sclMask = (1U << sclPinIndex);
  uint32_t pinMask = sclMask | sdaMask;

  // Retrieve correct AF for each
  uint32_t fnSDA = pinmap_function(_sdaPin, PinMap_I3C_SDA);
  uint32_t fnSCL = pinmap_function(_sclPin, PinMap_I3C_SCL);
  uint32_t afSDA = STM_PIN_AFNUM(fnSDA);
  uint32_t afSCL = STM_PIN_AFNUM(fnSCL);

  if (afSDA != afSCL) {
    return false;
  }

  // Enable GPIO port clock
  switch (sdaPortIndex) {
    case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
    case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
    case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
    case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
#if defined(GPIOE)
    case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
#endif
#if defined(GPIOF)
    case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
#endif
#if defined(GPIOG)
    case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
#endif
#if defined(GPIOH)
    case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
#endif
    default: return false;
  }

  GPIO_InitTypeDef GPIO_InitStruct{};

  // Phase 1: AF with pull-up (to stabilize bus at power-up)
  GPIO_InitStruct.Pin       = pinMask;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = afSDA;
  HAL_GPIO_Init(sdaPort, &GPIO_InitStruct);

  HAL_Delay(1);

  // Phase 2: AF no-pull (I3C normal operation)
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sdaPort, &GPIO_InitStruct);

  return true;
}

bool I3CBus::initClocks()
{
  HAL_StatusTypeDef status;

  if (_instance == I3C1) {
#if defined(I3C1)
    RCC_PeriphCLKInitTypeDef PeriphClkInit {};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I3C1;
    PeriphClkInit.I3c1ClockSelection   = RCC_I3C1CLKSOURCE_PCLK1;
    status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    if (status != HAL_OK) {
      return false;
    }

    __HAL_RCC_I3C1_CLK_ENABLE();

    HAL_NVIC_SetPriority(I3C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
    HAL_NVIC_SetPriority(I3C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C1_ER_IRQn);
#else
    return false;
#endif
  }
#if defined(I3C2)
  else if (_instance == I3C2) {
    RCC_PeriphCLKInitTypeDef PeriphClkInit{};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I3C2;
#if defined(STM32U3xx)
    PeriphClkInit.I3c2ClockSelection   = RCC_I3C2CLKSOURCE_PCLK2;
#elif defined(STM32H5xx)
    PeriphClkInit.I3c2ClockSelection   = RCC_I3C2CLKSOURCE_PCLK3;
#endif
    status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    if (status != HAL_OK) {
      return false;
    }

    __HAL_RCC_I3C2_CLK_ENABLE();

    HAL_NVIC_SetPriority(I3C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C2_EV_IRQn);
    HAL_NVIC_SetPriority(I3C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C2_ER_IRQn);
  }
#endif
  else {
    return false;
  }

  return true;
}

// ============================================================================
// 2. Bus initialization (Arduino-like begin)
// ============================================================================

bool I3CBus::begin(uint32_t sda, uint32_t scl, uint32_t freq)
{
  _sdaPin = digitalPinToPinName(sda);
  _sclPin = digitalPinToPinName(scl);
  return begin(freq);
}

bool I3CBus::begin(uint32_t freq)
{

  if (_initialized) {
    return true;
  }

  if (!initClocks()) {
    return false;
  }
  if (!initGPIO()) {
    return false;
  }

  std::memset(&_hi3c, 0, sizeof(_hi3c));
  _hi3c.Instance =  _instance;
  _hi3c.Mode     = HAL_I3C_MODE_CONTROLLER;

  if (HAL_I3C_Init(&_hi3c) != HAL_OK) {
    return false;
  }

  // Configure FIFO thresholds and disable Control/Status FIFOs (private transfers)
  I3C_FifoConfTypeDef fifoCfg{};
  fifoCfg.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_4_4;
  fifoCfg.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_4_4;
  fifoCfg.ControlFifo     = HAL_I3C_CONTROLFIFO_DISABLE;
  fifoCfg.StatusFifo      = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(&_hi3c, &fifoCfg) != HAL_OK) {
    return false;
  }

  // Generic controller configuration (no hot-join, no stall)
  I3C_CtrlConfTypeDef ctrlCfg{};
  ctrlCfg.DynamicAddr     = 0;
  ctrlCfg.StallTime       = 0x00;
  ctrlCfg.HotJoinAllowed  = DISABLE;
  ctrlCfg.ACKStallState   = DISABLE;
  ctrlCfg.CCCStallState   = DISABLE;
  ctrlCfg.TxStallState    = DISABLE;
  ctrlCfg.RxStallState    = DISABLE;
  ctrlCfg.HighKeeperSDA   = DISABLE;

  if (HAL_I3C_Ctrl_Config(&_hi3c, &ctrlCfg) != HAL_OK) {
    return false;
  }

  _initialized = true;

  // Apply bus frequency (timing computation) if requested
  if (freq == 0) {
    freq = 1000000;  // default to 1 MHz mixed I3C+I2C
  }

  if (setBusFrequency(freq) != 0) {
    return false;
  }
  return true;
}

// ============================================================================
// 3. Private transfers
// ============================================================================

int I3CBus::writeRegisterBlock(uint8_t devAddr, uint8_t reg, const uint8_t *pData, size_t len, uint32_t timeout)
{
  if (!_initialized || pData == nullptr || len == 0 || len > 31) {
    return -1;
  }

  I3C_XferTypeDef    context{};
  I3C_PrivateTypeDef priv{};
  uint32_t           controlBuffer[0xF];
  uint8_t            data[32] = {0};

  // Prefix first byte with register address
  data[0] = reg;
  memcpy(&data[1], pData, len);

  priv.TargetAddr    = devAddr;
  priv.TxBuf.pBuffer = data;
  priv.TxBuf.Size    = static_cast<uint32_t>(1 + len);
  priv.RxBuf.pBuffer = nullptr;
  priv.RxBuf.Size    = 0;
  priv.Direction     = HAL_I3C_DIRECTION_WRITE;

  memset(&context, 0, sizeof(context));
  context.CtrlBuf.pBuffer = controlBuffer;
  context.CtrlBuf.Size    = 1;
  context.TxBuf.pBuffer   = data;
  context.TxBuf.Size      = static_cast<uint32_t>(1 + len);

  HAL_StatusTypeDef st = HAL_I3C_AddDescToFrame(
                           &_hi3c,
                           nullptr,
                           &priv,
                           &context,
                           context.CtrlBuf.Size,
                           I3C_PRIVATE_WITH_ARB_RESTART
                         );
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  st = HAL_I3C_Ctrl_Transmit(&_hi3c, &context, timeout);
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  return 0;
}

int I3CBus::readRegisterBlock(uint8_t devAddr, uint8_t reg, uint8_t *pData, size_t len, uint32_t timeout)
{
  if (!_initialized || pData == nullptr || len == 0 || len > 32) {
    return -1;
  }

  I3C_XferTypeDef    context{};
  I3C_PrivateTypeDef priv{};
  uint32_t           controlBuffer[0xF];
  uint8_t            regBuf[1] = { reg };
  uint8_t            data[32]  = {0};

  // 1) Write register address
  priv.TargetAddr    = devAddr;
  priv.TxBuf.pBuffer = regBuf;
  priv.TxBuf.Size    = 1;
  priv.RxBuf.pBuffer = nullptr;
  priv.RxBuf.Size    = 0;
  priv.Direction     = HAL_I3C_DIRECTION_WRITE;

  memset(&context, 0, sizeof(context));
  context.CtrlBuf.pBuffer = controlBuffer;
  context.CtrlBuf.Size    = 1;
  context.TxBuf.pBuffer   = regBuf;
  context.TxBuf.Size      = 1;

  HAL_StatusTypeDef st = HAL_I3C_AddDescToFrame(
                           &_hi3c,
                           nullptr,
                           &priv,
                           &context,
                           context.CtrlBuf.Size,
                           I3C_PRIVATE_WITH_ARB_RESTART
                         );
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  st = HAL_I3C_Ctrl_Transmit(&_hi3c, &context, timeout);
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  // 2) Read data
  priv.TargetAddr    = devAddr;
  priv.TxBuf.pBuffer = nullptr;
  priv.TxBuf.Size    = 0;
  priv.RxBuf.pBuffer = data;
  priv.RxBuf.Size    = static_cast<uint32_t>(len);
  priv.Direction     = HAL_I3C_DIRECTION_READ;

  memset(&context, 0, sizeof(context));
  context.CtrlBuf.pBuffer = controlBuffer;
  context.CtrlBuf.Size    = 1;
  context.RxBuf.pBuffer   = data;
  context.RxBuf.Size      = static_cast<uint32_t>(len);

  st = HAL_I3C_AddDescToFrame(
         &_hi3c,
         nullptr,
         &priv,
         &context,
         context.CtrlBuf.Size,
         I3C_PRIVATE_WITH_ARB_STOP
       );
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  st = HAL_I3C_Ctrl_Receive(&_hi3c, &context, timeout);
  if (st != HAL_OK) {
    return -static_cast<int>(HAL_I3C_GetError(&_hi3c));
  }

  memcpy(pData, data, len);
  return 0;
}


int I3CBus::writeRegister(uint8_t devAddr, uint8_t reg,
                          uint8_t value, uint32_t timeout)
{
  return writeRegisterBlock(devAddr, reg, &value, 1, timeout);
}

int I3CBus::readRegister(uint8_t devAddr, uint8_t reg,
                         uint8_t &value, uint32_t timeout)
{
  return readRegisterBlock(devAddr, reg, &value, 1, timeout);
}

// ============================================================================
// 4. Device readiness helpers
// ============================================================================

bool I3CBus::isI2CDeviceReady(uint8_t addr, uint32_t trials, uint32_t timeout)
{
  if (!_initialized) {
    return false;
  }
  HAL_StatusTypeDef st = HAL_I3C_Ctrl_IsDeviceI2C_Ready(&_hi3c, addr, trials, timeout);
  return (st == HAL_OK);
}

bool I3CBus::isI3CDeviceReady(uint8_t dynAddr, uint32_t trials, uint32_t timeout)
{
  if (!_initialized) {
    return false;
  }
  HAL_StatusTypeDef st = HAL_I3C_Ctrl_IsDeviceI3C_Ready(&_hi3c, dynAddr, trials, timeout);
  return (st == HAL_OK);
}

// ============================================================================
// 5. CCC broadcast / direct (RSTDAA, DISEC, SETDASA)
// ============================================================================

// RSTDAA (0x06) – broadcast CCC: reset dynamic addresses of all I3C targets.
// Ignores CE2 (broadcast 7'h7E NACK) because this is expected when no target
// has a dynamic address yet.
int I3CBus::rstdaaOnce()
{
  static uint8_t rstdaa_done = 0;
  if (rstdaa_done) {
    return 0;
  }

  uint32_t        aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers{};
  uint8_t         aTxBuffer[0x0F];

  I3C_CCCTypeDef aSET_CCC_RST[] = {
    { 0x00, 0x06, { nullptr, 0 }, HAL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size    = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer   = aTxBuffer;
  aContextBuffers.TxBuf.Size      = 4;

  HAL_StatusTypeDef st = HAL_I3C_AddDescToFrame(
                           &_hi3c,
                           aSET_CCC_RST,
                           nullptr,
                           &aContextBuffers,
                           COUNTOF(aSET_CCC_RST),
                           I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART
                         );
  if (st != HAL_OK) {
    return -1;
  }

  st = HAL_I3C_Ctrl_TransmitCCC(&_hi3c, &aContextBuffers, 1000);
  if (st != HAL_OK) {
    uint32_t err = HAL_I3C_GetError(&_hi3c);

    // CE2: broadcast 7'h7E NACKed – can be ignored in most scenarios.
    if (err & HAL_I3C_ERROR_CE2) {
    } else {
      return -1;
    }
  }

  rstdaa_done = 1;
  return 0;
}

// DISEC (0x01) – broadcast CCC: disable some target capabilities (e.g. HotJoin).
// pCCCData: optional data bytes for DISEC (e.g. 0x08 ).
int I3CBus::disecOnce(const uint8_t *pCCCData, uint16_t length)
{
  static uint8_t disec_done = 0;
  if (disec_done) {
    return 0;
  }

  uint32_t        aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers{};
  uint8_t         aTxBuffer[0x0F];

  I3C_CCCTypeDef aSET_CCC_DISEC[] = {
    { 0x00, 0x01, { const_cast<uint8_t *>(pCCCData), length }, HAL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size    = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer   = aTxBuffer;
  aContextBuffers.TxBuf.Size      = 4;

  uint32_t option = (length > 0)
                    ? I3C_BROADCAST_WITH_DEFBYTE_RESTART
                    : I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART;

  HAL_StatusTypeDef st = HAL_I3C_AddDescToFrame(
                           &_hi3c,
                           aSET_CCC_DISEC,
                           nullptr,
                           &aContextBuffers,
                           COUNTOF(aSET_CCC_DISEC),
                           option
                         );
  if (st != HAL_OK) {
    return -1;
  }

  st = HAL_I3C_Ctrl_TransmitCCC(&_hi3c, &aContextBuffers, 1000);
  if (st != HAL_OK) {
    return -1;
  }

  disec_done = 1;
  return 0;
}

// SETDASA (0x87) – direct CCC: assign dynAddr7 to target at staticAddr7.
// staticAddr7: 7-bit static address
// dynAddr7:    7-bit dynamic address; DA byte = (dynAddr7 << 1) on the wire.
int I3CBus::setdasa(uint8_t staticAddr7, uint8_t dynAddr7)
{
  if (!_initialized) {
    return -1;
  }

  uint32_t        aControlBuffer[0xFF];
  I3C_XferTypeDef aContextBuffers{};
  uint8_t         aTxBuffer[0x0F];

  // DA byte: bits [7:1] = dynAddr, bit0 = 0 (as per I3C spec)
  uint8_t daByte = static_cast<uint8_t>((dynAddr7 & 0x7F) << 1);

  I3C_CCCTypeDef aSET_DASA_Desc[] = {
    { static_cast<uint8_t>(staticAddr7 & 0x7F), 0x87, { &daByte, 1 }, HAL_I3C_DIRECTION_WRITE },
  };

  aContextBuffers.CtrlBuf.pBuffer = aControlBuffer;
  aContextBuffers.CtrlBuf.Size    = COUNTOF(aControlBuffer);
  aContextBuffers.TxBuf.pBuffer   = aTxBuffer;
  aContextBuffers.TxBuf.Size      = 4;

  HAL_StatusTypeDef st = HAL_I3C_AddDescToFrame(
                           &_hi3c,
                           aSET_DASA_Desc,
                           nullptr,
                           &aContextBuffers,
                           COUNTOF(aSET_DASA_Desc),
                           I3C_DIRECT_WITHOUT_DEFBYTE_RESTART
                         );
  if (st != HAL_OK) {
    return -1;
  }

  st = HAL_I3C_Ctrl_TransmitCCC(&_hi3c, &aContextBuffers, 1000);
  if (st != HAL_OK) {
    return -1;
  }

  return 0;
}

// ============================================================================
// 6. Dynamic addressing (ENTDAA + SetDynAddr)
// ============================================================================

int I3CBus::entdaa(I3CDiscoveredDevice *devices, size_t maxDevices, uint8_t firstDynAddr, uint32_t timeout)
{
  if (!_initialized || devices == nullptr || maxDevices == 0) {
    return -1;
  }

  HAL_StatusTypeDef st;
  uint64_t entdaaPayload = 0;
  size_t count = 0;

  // First dynamic address to assign; default to 0x10 if 0
  uint8_t nextDynAddr = (firstDynAddr != 0) ? firstDynAddr : 0x10;

  do {
    entdaaPayload = 0ULL; // HAL will OR into this field

    st = HAL_I3C_Ctrl_DynAddrAssign(&_hi3c,
                                    &entdaaPayload,
                                    I3C_RSTDAA_THEN_ENTDAA, // RSTDAA + ENTDAA
                                    timeout);
    if (st == HAL_BUSY) {
      // A target responded: entdaaPayload contains BCR+DCR+PID packed
      if (count >= maxDevices) {
        return -2; // device array too small
      }

      uint8_t da = nextDynAddr;
      nextDynAddr += 2; // leave a gap of 2 for convenience

      if (HAL_I3C_Ctrl_SetDynAddr(&_hi3c, da) != HAL_OK) {
        return -3;
      }

      devices[count].dynAddr   = da;
      devices[count].dcrBcrPid = entdaaPayload;
      count++;}
    else if (st != HAL_OK) {
      uint32_t err = HAL_I3C_GetError(&_hi3c);
      Serial.print("ENTDAA: DynAddrAssign failed, st=");
      Serial.print((int)st);
      Serial.print(" err=hhh0x");
      Serial.println(err, HEX);
      return -4;
    }

  } while (st == HAL_BUSY);

  // st == HAL_OK: no more I3C targets to discover
  return static_cast<int>(count);
}

// ============================================================================
// 7. Bus timing / frequency configuration
// ============================================================================

int I3CBus::setBusFrequency(uint32_t i3cFreq)
{
#if !defined(HAL_I3C_MODULE_ENABLED)
  (void)i3cFreq;
  return 0;
#else
  if (!_initialized) {
    // Option: could allow pre-init timing configuration, but not supported here
    return -1;
  }

  if (i3cFreq == 0) {
    // 0 = keep current timings
    return 0;
  }

  I3C_CtrlTimingTypeDef     inTiming{};
  LL_I3C_CtrlBusConfTypeDef outCtrl{};

  // Peripheral clock frequency
  uint32_t srcFreq = 0;

  if (_instance == I3C1) {
    srcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C1);
  }
#if defined(I3C2)
  else if (_instance == I3C2) {
    srcFreq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I3C2);
  }
#endif

  if (srcFreq == 0U) {
    return -1;
  }

  // Fill timing input structure
  inTiming.clockSrcFreq = srcFreq;
  inTiming.i3cPPFreq = i3cFreq;              // I3C push-pull speed
  inTiming.i2cODFreq = (i3cFreq >= 1000000U) // I2C open-drain speed
                       ? 1000000U
                       : 400000U;
  inTiming.dutyCycle = 50;                  // ~50% duty cycle SDR
  inTiming.busType = I3C_MIXED_BUS;         // I3C + legacy I2C on same bus

  // Compute TIMINGR0/TIMINGR1
  if (I3C_CtrlTimingComputation(&inTiming, &outCtrl) != SUCCESS) {
    return -1;
  }

  // Apply these timings to the controller
  if (HAL_I3C_Ctrl_BusCharacteristicConfig(&_hi3c, &outCtrl) != HAL_OK) {
    return -1;
  }

  return 0;
#endif
}
