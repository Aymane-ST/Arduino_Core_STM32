/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2013 by Paul Stoffregen <paul@pjrc.com> (delayMicroseconds)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_H_
#define _WIRING_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Arduino.h"

#include "avr/dtostrf.h"
#include "itoa.h"


#include <stdarg.h>
#include "clock.h"
#include "dwt.h"
#include <sys/time.h> // for struct timeval

// analog
#ifdef __cplusplus
extern "C" {
#endif
/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);
/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(int res);
/*
 * \brief Set the frequency of analogWrite. Default is PWM_FREQUENCY (1000) in Hertz.
 *
 * \param freq
 */
extern void analogWriteFrequency(uint32_t freq);
extern void analogOutputInit(void) ;
#ifdef __cplusplus
}
#endif

//constants
#define DEFAULT 1
#define EXTERNAL 0

#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

#ifndef _BV
  #define _BV(bit) (1 << (bit))
#endif
#ifndef cbi
  #define cbi(reg, bitmask) *reg &= ~bitmask
#endif
#ifndef sbi
  #define sbi(reg, bitmask) *reg |= bitmask
#endif

typedef bool boolean __attribute__((deprecated));

//digital
#ifdef __cplusplus
extern "C" {
#endif
/**
 * \brief Toggle the value from a specified digital pin.
 *
 * \param ulPin The number of the digital pin you want to toggle (int)
 */
extern void digitalToggle(pin_size_t pinNumber) ;
#ifdef __cplusplus
}
#endif


//time
#ifdef __cplusplus
extern "C" {
#endif
/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param us the number of microseconds to pause (uint32_t)
 */
static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
#if defined(DWT_BASE) && !defined(DWT_DELAY_DISABLED)
  int32_t start  = dwt_getCycles();
  int32_t cycles = us * (SystemCoreClock / 1000000);

  while ((int32_t)dwt_getCycles() - start < cycles);
#else
  __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks :
                    oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
#endif
}

/**
 * \brief gives the number of seconds and microseconds since the Epoch
 *
 *        based on millisecond since last power on.
 *
 * \note  The function is declared as weak  to be overwritten  in case of other
 *        implementations in user file (using RTC values for example).
 *
 * \param tv argument is a struct timeval
 * \param tz argument is a struct timezone (unused)
 *
 * \return 0
 */
int __attribute__((weak)) _gettimeofday(struct timeval *tv, void *tz)
{
  (void)tz;
  tv->tv_sec = getCurrentMillis() / 1000;
  tv->tv_usec = getCurrentMicros() - (tv->tv_sec * 1000000);  // get remaining microseconds
  return 0;
}

#ifdef __cplusplus
}
#endif

#include <board.h>

#ifdef __cplusplus
  #include "HardwareTimer.h"
  #include "WCharacter.h"
  #include "WSerial.h"
#endif // __cplusplus


#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )

#endif /* _WIRING_H_ */
