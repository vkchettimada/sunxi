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
#define PIO_REG_CFG_VAL_IO        (2)

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

/******************************************************************************
* Magic ...
******************************************************************************/
#define CCMC_REGS_BASE            (APB0_BASE + 0x00000)    //ccm pll controller0
#define CCMM_REGS_BASE            (APB0_BASE + 0x00400)    //ccm module controller0

#define CCM_CPU_SOURCECTRL		    (*((uint32_t volatile *)(CCMC_REGS_BASE+0x050)))

#define CCM_PLL4_PERP0_CTRL    	  (*((uint32_t volatile *)(CCMC_REGS_BASE+0x00C)))
#define CCM_PLL12_PERP1_CTRL      (*((uint32_t volatile *)(CCMC_REGS_BASE+0x02C)))

#define CCMU_REG_o_AVS            (0xD4)
#define CCMU_REG_AVS					    (*((uint32_t volatile *)(CCMM_REGS_BASE + CCMU_REG_o_AVS)))

#define TMRC_REGS_BASE		        (APB0_BASE + 0x00C00)    //tmr controller0

#define TMRC_AVS_CTRL			        (*((uint32_t volatile *)(TMRC_REGS_BASE + 0x80)))
#define TMRC_AVS_COUNT0			      (*((uint32_t volatile *)(TMRC_REGS_BASE + 0x84)))
#define TMRC_AVS_COUNT1			      (*((uint32_t volatile *)(TMRC_REGS_BASE + 0x88)))
#define TMRC_AVS_DIVISOR		      (*((uint32_t volatile *)(TMRC_REGS_BASE + 0x8C)))

#define REGS_APB1_BASE	          0x07000000

#define UART0_REGS_BASE        ( REGS_APB1_BASE + 0x00000 )    //uart0 controller0
#define UART1_REGS_BASE        ( REGS_APB1_BASE + 0x00400 )    //uart1 controller0
#define UART2_REGS_BASE        ( REGS_APB1_BASE + 0x00800 )    //uart2 controller0
#define UART3_REGS_BASE        ( REGS_APB1_BASE + 0x00C00 )    //uart3 controller0
#define UART4_REGS_BASE        ( REGS_APB1_BASE + 0x01000 )    //uart4 controller0
#define UART5_REGS_BASE        ( REGS_APB1_BASE + 0x01400 )    //uart5 controller0

#define UART_REGS_BASE    			    (UART0_REGS_BASE)
#define UART_OFFSET                 0x400

#define UART_REG_o_RBR              0x00
#define UART_REG_o_THR              0x00
#define UART_REG_o_DLL              0x00
#define UART_REG_o_DLH              0x04
#define UART_REG_o_IER              0x04
#define UART_REG_o_IIR              0x08
#define UART_REG_o_FCR              0x08
#define UART_REG_o_LCR              0x0C
#define UART_REG_o_MCR              0x10
#define UART_REG_o_LSR              0x14
#define UART_REG_o_MSR              0x18
#define UART_REG_o_SCH              0x1C
#define UART_REG_o_USR              0x7C
#define UART_REG_o_TFL              0x80
#define UART_REG_o_RFL              0x84
#define UART_REG_o_HALT             0xA4

#define UART_REG_RBR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_RBR  )))
#define UART_REG_THR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_THR  )))
#define UART_REG_DLL(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_DLL  )))
#define UART_REG_DLH(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_DLH  )))
#define UART_REG_IER(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_IER  )))
#define UART_REG_IIR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_IIR  )))
#define UART_REG_FCR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_FCR  )))
#define UART_REG_LCR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_LCR  )))
#define UART_REG_MCR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_MCR  )))
#define UART_REG_LSR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_LSR  )))
#define UART_REG_MSR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_MSR  )))
#define UART_REG_SCH(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_SCH  )))
#define UART_REG_USR(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_USR  )))
#define UART_REG_TFL(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_TFL  )))
#define UART_REG_RFL(port)          (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_RFL  )))
#define UART_REG_HALT(port)         (*((uint32_t volatile *)( UART_REGS_BASE + port * UART_OFFSET + UART_REG_o_HALT )))

#define SERIAL_CHAR_READY(port)     ( UART_REG_LSR(port) & ( 1 << 0 ) )
#define SERIAL_READ_CHAR(port)      UART_REG_RBR(port)
#define SERIAL_READ_STATUS(port)    ( UART_REG_LSR(port) & 0xFF )
#define SERIAL_WRITE_STATUS(port)	UART_REG_LSR(port)
#define SERIAL_WRITE_READY(port)	( UART_REG_LSR(port) & ( 1 << 6 ) )
#define SERIAL_WRITE_CHAR(port, c)	( ( UART_REG_THR(port) ) = ( c ) )

void main(void)
{
  uint32_t port, pin, val;
  uint32_t uart_port;
  uint32_t volatile *reg;
  uint32_t apb_freq;
  uint32_t uart_baud, uart_clk, lcr;
  uint32_t i;

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

  /* A80, Cubieboard4, Red LED On (PH06) */
  port = 'H';
  pin = 12;

  /* PIO Configure */
  port -= 'A';
  val = *(PIO_REG_CFG(PIO_BASE, port, pin));
  val &= ~(0x07 << PIO_REG_CFG_POS(pin));
  val |= (PIO_REG_CFG_VAL_IO<< PIO_REG_CFG_POS(pin));
  *(PIO_REG_CFG(PIO_BASE, port, pin)) = val;

  /* PIO level */
  val = *(PIO_REG_LVL(PIO_BASE, port, pin));
  val &= ~(0x0F << PIO_REG_LVL_POS(pin));
  val |= (PIO_REG_LVL_VAL_1 << PIO_REG_LVL_POS(pin));
  *(PIO_REG_LVL(PIO_BASE, port, pin)) = val;

  /* PIO Pull */
  val = *(PIO_REG_PUL(PIO_BASE, port, pin));
  val &= ~(0x0F << PIO_REG_PUL_POS(pin));
  val |= (PIO_REG_PUL_VAL_HIGH << PIO_REG_PUL_POS(pin));
  *(PIO_REG_PUL(PIO_BASE, port, pin)) = val;

  /****************************************************************************
  * Magic ...
  ****************************************************************************/

  #if 0
  /* Timer init */
  CCMU_REG_AVS |= (1U << 31);
  TMRC_AVS_CTRL = 3;
  TMRC_AVS_DIVISOR = 0xC2EE0;
	TMRC_AVS_COUNT0 = 0;
	TMRC_AVS_COUNT1 = 0;
  #endif

  /* PLL init, just for UART to work? */
  //CCM_CPU_SOURCECTRL &= ~(0x01);
  CCM_PLL4_PERP0_CTRL = 0x80002800;
  //CCM_PLL12_PERP1_CTRL = 0x80003200;

  /* UART config */
  uart_port = 0;

  //reset
  reg = (uint32_t volatile *)(0x06000400 + 0x1B4);
  *reg &= ~(1 << (16 + uart_port));
  for( i = 0; i < 100; i++ );
  *reg |=  (1 << (16 + uart_port));

  //gate
  reg = (uint32_t volatile *)(0x06000400 + 0x194);
  *reg &= ~(1 << (16 + uart_port));
  for( i = 0; i < 100; i++ );
  *reg |=  (1 << (16 + uart_port));

  // magic!
  *((uint32_t volatile *)0x01c202D8) |= (1 << (16 + uart_port));

  // Set Baudrate
  apb_freq = 24 * 1000 * 1000;
  uart_baud = 115200;
  uart_clk = ( apb_freq + 8*uart_baud ) / (16*uart_baud);
  lcr = UART_REG_LCR(uart_port);
  UART_REG_HALT(uart_port) = 1;
  UART_REG_LCR(uart_port) = lcr | 0x80;
  UART_REG_DLH(uart_port) = uart_clk>>8;
  UART_REG_DLL(uart_port) = uart_clk&0xff;
  UART_REG_LCR(uart_port) = lcr & (~0x80);
  UART_REG_HALT(uart_port) = 0;

  // Set Lin Control Register
  UART_REG_LCR(uart_port) = ((0 /*PARITY*/ &0x03)<<3) | ((0/*STOP*/&0x01)<<2) | (3/*DLEN = 8*/&0x03);

  // Disable FIFOs
  UART_REG_FCR(uart_port) = 0x06;

  // Namaste!
  {
    char const * p_namaste = "\r\nNamaste !\r\n";

    while (*p_namaste)
    {
      while (!SERIAL_WRITE_READY(uart_port));
      SERIAL_WRITE_CHAR(uart_port, *p_namaste++);
    }
  }

  /****************************************************************************
  * ... magic.
  ****************************************************************************/

  #if 1
  /* A80, Cubieboard4, Green LED On (PH17) */
  port = 'H';
  pin = 17;

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
  #endif

  while(1);
}
