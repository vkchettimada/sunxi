#! /bin/sh

arm-none-eabi-as -c -o start.o start.s
arm-none-eabi-gcc -Os -mcpu=cortex-a7 -c -o boot.o boot.c
arm-none-eabi-ld --script boot.lds -Map boot.map -o boot.elf start.o boot.o
arm-none-eabi-size boot.elf
