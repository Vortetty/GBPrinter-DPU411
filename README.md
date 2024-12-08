# DPU411-Based gameboy printer replacement

Uses a clone of the Gameboy Printer emulator to act as a printer, while managing a thermal printer to print at a larger size.

This version is made for the DPU411 Type 2 printer, schematics are in the DRIVER folder and require kicad, the pcb is not guaranteed to work as i have not been able to try it.

The code here shold be adaptable to virtually every thermal printer you find, maybe even some non-thermal if you write enough code. All you need to change is the head control and init code to match your printer, the decoding can be the same in 99% of cases.

## How to set up

1. Build the printer as shown in the schematic, will require pulling all the boards out and replacing them with alot of wiring.
2. Flash the micro with the code found in [this repo](https://github.com/Vortetty/arduino-gameboy-printer-parser-for-DPU411TYPE2-printer)
3. Flash the pico with the code found here
4. Connect to a GB/GBA and enjoy!

## Example

[![Video on my yt channel](http://img.youtube.com/vi/J4ya0VCBBrQ/0.jpg)](https://www.youtube.com/watch?v=J4ya0VCBBrQ "Gameboy Printer... but better(?)")
