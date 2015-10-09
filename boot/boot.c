/*
Copyright (c) 2015, Vinayak Kariappa Chettimada
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdint.h>

/* Peripheral bus base */
#define APB0_BASE                 (0x06000000)

/* IO base */
#define PIO_BASE                  (APB0_BASE + 0x00000800)

/* Configuration takes 4 bits, hence 8 pins can be configured in 32-bit reg */
#define PIO_REG_CFG(b, port, p)   ((uint32_t volatile *)(b + ((port) * 0x24) + (((p) >> 3) << 2) + 0x00))
#define PIO_REG_CFG_POS(pin)      (((pin) & 0x07) << 2)

/* Data takes 1 bit, hence all 32 pins can be configured in 32-bit reg */
#define PIO_REG_DAT(b, port)      ((uint32_t volatile *)(b + ((port) * 0x24) + 0x10))

/* Level takes 2 bits, hence 16 pins can be configured in 32-bit reg */
#define PIO_REG_LVL(b, port, p)   ((uint32_t volatile *)(b + ((port) * 0x24) + (((p) >> 4) << 2) + 0x14))
#define PIO_REG_LVL_POS(pin)      (((pin) & 0x0F) << 1)

/* Pull takes 2 bits, hence 16 pins can be configured in 32-bit reg */
#define PIO_REG_PUL(b, port, p)   ((uint32_t volatile *)(b + ((port) * 0x24) + (((p) >> 4) << 2) + 0x1C))
#define PIO_REG_PUL_POS(pin)      (((pin) & 0x0F) << 1)

/* Configuration value, 0 - input, 1 -output, 2-7 - I/O function */
#define PIO_REG_CFG_VAL_INPUT     (0)
#define PIO_REG_CFG_VAL_OUTPUT    (1)

/* Level values */
/** @todo Find what 0-3 mean */
#define PIO_REG_LVL_VAL_0         (0)
#define PIO_REG_LVL_VAL_1         (1)
#define PIO_REG_LVL_VAL_2         (2)
#define PIO_REG_LVL_VAL_3         (3)

/* Pull values */
#define PIO_REG_PUL_VAL_NONE      (0)
#define PIO_REG_PUL_VAL_HIGH      (1)
#define PIO_REG_PUL_VAL_LOW       (2)

void main(void)
{
  uint32_t port, pin, val;

  /* A80, Cubieboard4, Red LED On (PH06) */
  port = 'H';
  pin = 6;

  /* PIO Configure */
  port -= 'A';
  val = *(PIO_REG_CFG(PIO_BASE, port, pin));
  val &= ~(0x07 << PIO_REG_CFG_POS(pin));
  val |= (PIO_REG_CFG_VAL_OUTPUT << PIO_REG_CFG_POS(pin));
  *(PIO_REG_CFG(PIO_BASE, port, pin)) = val;

  /* PIO data */
  val = *(PIO_REG_DAT(PIO_BASE, port));
  val &= ~(1 << pin);
  val |= (1 << pin);
  *(PIO_REG_DAT(PIO_BASE, port)) = val;
 
  while(1);
}
