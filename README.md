# FPGAComputer
This is a 16-bit computer implemented in the DE0-NANO FPGA.

The computer has 16-bit CPU, 64KB, UART (115200 bps), and VGA (640x480, text-based frame buffer, 80x60 characters).

The 16-bit CPU has 8 general-purpose registers (r0 – r7), pc (program counter), sp (stack pointer), ir (instruction register), mbr (memory buffer register), h (higher word when multiplying, or remainder when dividing).

The address bus is 16 bits wide, addressing 65536 addresses. Data bus is also 16 bits wide, but all the addresses are 8-bit aligned. This gives 65536 bytes, or 64KB.

Video output is VGA, 640x480. Text mode hase 80x60 characters, each character being 8x8  pixels in dimensions. Video framebuffer in text mode has 4800 16-bit words (80x60 characters). The lower byte has the ASCII character, while the upper byte has the attributes (3 bits for the background color, 3 bits for the foreground color, inverted, and the last two bits unused).

It has two interrupts: IRQ0 and IRQ1. IRQ0 is connected to the KEY2 of the DE0-NANO, while IDQ1 is connected to the UART. Whenever a byte comes to the UART, it generates an IRQ1. Interrupt causes CPU to push flags to the stack, then to push PC to the stack and then to jump to the location designated for the CPU:
* for the IRQ0, it is 0x0004, and
* for the IRQ1, it is 0x0008.

It is up to the programmer to put the code in those locations. Usually, it is a JUMP instruction. To return from the interrupt routine, it is necessary to put the IRET instruction. It pops the return address, and then pops the flags register, and then goes back into the interrupted program.
KEY1 of the DE0-NANO is used as the reset key. When pressed, it forces CPU to go to the 0x0000 address. Usually there is a JUMP instruction to go to the main program.

# VGA text mode
Text mode is 80x60 characters, occupying 4800 words. Lower byte is the ASCII code of a character, while the upper byte is the attributes.

The foreground color is inverted so zero values (default) would mean white color. That way, you don't need to set the foreground color to white, and by default (0, 0, 0), it is white. The default background color is black (0, 0, 0). This means that if the upper (Attribute) byte is zero (0x00), the background color is black, and the foreground color is white.

VGA female connector is connected via resistors to the GPIO-0 expansion header of the DE0-NANO board:

* GPIO_R (pin 2, GPIO_00, PIN_A3) -> 68Ohm -> VGA_R,
* GPIO_G (pin 4, GPIO_01, PIN_C3) -> 68Ohm -> VGA_G,
* GPIO_B (pin 6, GPIO_03, PIN_D3) -> 68Ohm -> VGA_B,
* GPIO_HS (pin 8, GPIO_05, PIN_B4) -> 470Ohm -> VGA_HORIZONTAL_SYNC,
* GPIO_VS (pin 10, GPIO_07, PIN_B5) -> 470Ohm -> VGA_VERTICAL_SYNC.

# UART interface

UART interface provides TTL serial communication on 115200kbps. It uses one start bit, one stop bit, and eight data bits, no parity, no handshake.

UART is connected to the GPIO-0 expansion header of the DE0-NANO board:

* TX (pin 32, GPIO_025, PIN_D9) should be connected to the RX pin of the PC,
* RX (pin 34, GPIO_027, PIN_E10) should be connected to the TX pin of the PC.

UART is used within the CPU via IN, and OUT instructions. RX also triggers the IRQ1, which means that whenever a byte is received via UART, the IRQ1 will be triggered, forcing CPU to jump to the 0x0008 address. There you should place the JUMP instruction to your UART interrupt routine.

Inside the UART interrupt routine, you can get the received byte by using the IN instruction:

in r1, [64]; r1 holds now received byte from the UART 

To send a byte, first you need to check if the UART TX is free. You can do it by using the in instruction:

loop:
      in r5, [65]   ; tx busy in r5
      cmp r5, 0    
      jz not_busy   ; if not busy, send back the received character
      j loop
not_busy:
      out [66], r1  ; send the character to the UART

Addresses used by the UART are in the following list:
* 64 -> Received byte from the RX part of the UART (use the IN instruction).
* 65 -> 0 if the TX part of the UART is free to send a byte, 1 if TX part is busy.
* 66 -> Byte to be sent must be placed here using the OUT instruction.
