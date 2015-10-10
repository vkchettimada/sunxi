/*
Copyright (c) 2015, Vinayak Kariappa Chettimada
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Minimal head searched by BROM

  .section .boot_head
boot_head:
  b reset
  .ascii "eGON.BT0"
  .word 0x5F0A6C39            // Initial stamp, updated by PC tool
  .word 0                     // length, updated by PC tool
  .word 0                     // pad to 32 bytes
  .word 0                     //
  .word 0                     //
*/

  .section .start
start:
  mrs r0, cpsr
  bic r0, r0, #0x1F           // mask mode bits and uses little-endian
  orr r0, r0, #0xD3           // disable FIQ, IRQ and use SVC mode
  msr cpsr, r0

  mrc p15, 0, r0, c1, c0, 0
  bic r0, r0, #0x00000007     // disable MMU, align, dcache
  mcr p15, 0, r0, c1, c0, 0

  ldr sp, =0x19000

  ldr r0, =main
  bx r0

  .section .text

  .end
