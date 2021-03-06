# TeelSys MCP23S17 Arduino Library

This is a library for the MCP23S17 SPI Port Expander
 
These chips use SPI to communicate, 4 pins required to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Richard Teel as a drop-in replacement of the Adafruit_MCP23017 library written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution with the exception of Adafruit Industries. Adafruit Industries may modify this file and code as they see fit as the base code used was written and developed by Adafruit.

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder TeelSys_MCP23S17. Check that the TeelSys_MCP23S17 folder contains TeelSys_MCP23S17.cpp and TeelSys_MCP23S17.h

Place the TeelSys_MCP23S17 library folder your <arduinosketchfolder>/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

Pin Addressing
==============

When using single pin operations such as _pinMode(pinId, dir)_ or _digitalRead(pinId)_  or _digitalWrite(pinId, val)_ then the pins are addressed using the ID's below. For example, for set the mode of _GPB0_ then use _pinMode(8, ...)_.

Physical Pin #| Pin Name | Pin ID
----|------|------------------------------
21 | GPA0 | 0
22 | GPA1 | 1
23 | GPA2 | 2
24 | GPA3 | 3
25 | GPA4 | 4
26 | GPA5 | 5
27 | GPA6 | 6
28 | GPA7 | 7
1 | GPB0 |  8
2 | GPB1 |  9
3 | GPB2 | 10
4 | GPB3 | 11
5 | GPB4 | 12
6 | GPB5 | 13
7 | GPB6 | 14
8 | GPB7 | 15

BTW: I did not include any example code as I was interested in getting my project working. The code that I'm currently working on using the library may be found at <a href="https://github.com/richteel/Psion-Organiser-II---Graphic-LCD/tree/dev/Code/Keypad_LCD">https://github.com/richteel/Psion-Organiser-II---Graphic-LCD/tree/dev/Code/Keypad_LCD</a>. The code is using the Adafruit library for the MCP23017 and my modified library for the MCP23S17.
