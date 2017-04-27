# dm_firmware

This is a firmware for ATmega chip on dm_control board.

It is intended to be compiled with Atmel Studio 7, but you can also compile it with standard avr-gcc (with some command line options).
You can first burn a bootloader (such as Arduino or Optiboot) onto the ATmega so that firmware upgrade can be done via USB.

## Fuse Settings

This firmware does not handle WDT. WDT must be turned off.

As the firmware controls multiplexing of nixie tubes by 8.192kHz clock
from the RTC, it isn't very strict about clocking the cpu itself.
However, if you use a bootloader, you should set it to use the ceramic
resonator on dm_control board, so that UART communication has less errors.

If you use Arduino bootloader, the following fuse settings is good:
- Extended bits: 0xFD
- High bits: 0xDA
- Low bits: 0xFF

## Functions

The firmware has very limited set of functions.

When you power up the meter, it does a lamp check (cycling from 0 to 9 and dp),
and then displays a divergence value.

When you press SW2 or SW3, the display changes to next preset divergence value.

When you press SW5, it shuffles the display for about 10 seconds and then
goes back to the divergence value.

When you press SW4, it displays current voltage of the battery. When you press SW4 again while shoing the voltage, the meter goes into a demo mode.
In demo mode, divergence value changes every about 20 seconds.



## License

Files in this directory are licensed under BSD license.
