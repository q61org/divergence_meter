# dm_control Board

This board is the motherboard of the divergence meter.

The board has the following functionalities on board:
- Lithium-ion battery charger
- 3.7V to 180V DC/DC boost converter
- 3.7V to 3.3V DC/DC buck converter
- Microcontroller (ATmega328P) that controls nixie tubes
- USB to UART converter (FT231X)
- Accurate RTC (DS3234)


* About Power Supplies

You must be careful about power supplies on this board,
as they can be hazardous to you in some ways.

** Lithium-ion Battery Charger

This board has an MCP73831 battery charger chip. You can connect
a lithium-ion battery to connector P2 to make the divergence meter
free from power cords. 
However, you must be careful about what battery you use.

The MCP73831 chip controls only charging from the USB connector; 
it automatically stops charging when the battery is full, 
but it does not stop discharging when the battery is empty.

*There is no overdischarge protection on this board.* 
Therefore you *MUST* use a battery with a built-in
overdischarge protection. *DO NOT USE RAW CELLS!*

Connector P2 is a 2-pin PH connector. This is a standard connector
for commonly available pouched LiPo cells. These batteries
usually have protection circuits, but double-check it before 
you actually use one.


** DC/DC Converters

This board has two DC/DC converters on board, a boost converter
that generates 180V for igniting nixie tubes, and a buck converter
that generates 3.3V for microcontroller and other digital chips.

The one that can harm you is, obviously, the boost converter.
Although it can only output some milliamps continuously, it has
2uF of smoothing capacitors (C14 and C15). 
When the converter is powered up without any load, it can charge 
the capacitors to their maximum capacity. This charge will remain 
in the capacitors after the converter is powered down, 
and when you touch them, all the charge can go through you.
This can be very, very bad.

When handling this board, always make sure that the power is off,
and that capacitors are discharged.


* Disclaimer (again)

There is no warranty, expressed or implied, associated with files in this project. Especially there is a high voltage shock hazard which can kill you. Use at your own risk. 

* License

Schematics and PCB files are licensed under Creative Commons BY-SA 4.0.
