# Divergence Meter Project (by q61.org)

This is yet another divergence meter project.

This project tries to make a divergence meter that look just like the one in the Steins;Gate anime series, by implementing the following features:
- Using tip-less, flat-top nixie tubes (NL902) that look similar to the tubes in the anime.
- Compact dimensions: so that you can hold it on your hand like Suzuha does, and so that you can use the meter for cosplaying.
- A built-in, rechargeable battery; so that Suzuha can take it out of her bag while it is displaying a divergence value.

The result, a completed divergence meter can be seen below.

![divergence meter with dr pepper](http://q61.org/dm/dm_3748.jpg)
![divergence meter with an anime screen capture](http://q61.org/dm/dm_3757.jpg)


## Files in This Repository

This repository contains schematics, PCB files and a firmware of the divergence meter. Chassis design is left as an exercise to makers (forks and pull requests are welcome!).

The divergence meter is composed of 3 PCBs, and there are 3 PCB filesets in this repository:
- dm_control: A board that controls everything; the motherboard.
- dm_nixiebrd: A serial-to-parallel converter board with nixie drivers. This is the board where nixie tubes are mounted on, and stacks on top of dm_control board.
- dm_topboard: A perfboard-like board that goes on top of the divergence meter. This board does not have any electrical functions, but adds very much to the aesthetics.

dm_control board has a microcontroller, and its firmware is in the following subdirectory:
- dm_firmware: Firmware for dm_control board.


## License

Schematics and PCB, and firmware are licensed under different licenses:
- Schematics and PCB files are licensed under [Creative Commons BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
- Firmware is licensed under the BSD license. 

Please see README.md and LICENSE files in respective directories.


## See Also

This project was inspired by following projects:
- [Divergence Meter Project](http://www.mindspring.com/~tomtitor/)
- [世界线变动率探测仪 | Divergence Meter](http://www.nixieclock.org/?cat=219)


## Disclaimer

There is no warranty, expressed or implied, associated with files in this project. Especially there is a high voltage shock hazard which can kill you. Use at your own risk. 


## Author

Kouichi Kuroi (q61.org)
- Twitter: https://twitter.com/q61501331
