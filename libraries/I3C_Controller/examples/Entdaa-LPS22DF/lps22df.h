#pragma once

#include "I3C_Controller.h"
#include "LPS22DFSensor.h"


// Context for LPS22DF in I3C mode (C driver)
struct Lps22dfI3cCtx {
  I3CBus  *bus;     // pointer to the I3C bus used
  uint8_t  dynAddr; // dynamic address assigned via ENTDAA
};

// Initialize LPS22DF via ENTDAA + C driver on dynamic address.
//
// - bus          : I3CBus instance (I3C1Bus or I3C2Bus)
// - firstDynAddr : first DA to try
// - outDynAddr   : receives the DA that was ultimately assigned
// - outDrvCtx    : C driver context lps22df_ctx_t
// - outCtx       : backend context (bus + dynAddr) to keep globally
//
// Return:
//   true  : LPS22DF detected and initialized
//   false : ENTDAA, WHO_AM_I or driver init failed
bool LPS22DF_I3C_EntdaaInit(I3CBus &bus, uint8_t firstDynAddr, uint8_t &outDynAddr, lps22df_ctx_t &outDrvCtx, Lps22dfI3cCtx &outCtx);
