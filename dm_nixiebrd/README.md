# dm_nixiebrd Board

This is a serial-to-parallel converter board with nixie drivers.
It is designed for NL-902 or NL-842 nixie tubes.

This board needs a high voltage power to ignite nixie tubes. 
Be careful when handling high voltages. It can kill you.

* How to Connect With dm_control Board

The left half of this board is designed to match with the dm_control board.
You can solder pin headers to P1-P3, and they mate with P4-P6 of dm_control board.

## Mounting Nixie Tubes

This board is designed to use discrete pin sockets to mount nixie tubes.
The sockets should be a "Socket Pin" type which accept 1mm pins for 
socket side and which have 0.5mm pin for pin side.

If you search eBay with keywords "1mm nixie pin" you will get good examples.
And in fact I used pins from eBay for my first divergence meter.

Of course it is also good to modify the board so that it fits any sockets
or direct soldering you would want to use.


## Changing Nixie Tubes

As well as mounting method you can change nixie tubes that this board mounts.
If nixie tubes are rated for nominal currents out of 3mA-4mA range, 
you should change values of R2 and R12 resistors.


## Disclaimer (again)

There is no warranty, expressed or implied, associated with files in this project. Especially there is a high voltage shock hazard which can kill you. Use at your own risk. 


## License

Schematics and PCB files are licensed under Creative Commons BY-SA 4.0.
