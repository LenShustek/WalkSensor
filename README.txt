
           Battery-powered walk path sensor
 for clinical trials of therapies for movement disorders

  This is a small battery-powered wireless sensor that, when used
  in a pair on opposite sides of a walking path, detects someone
  walking through an IR beam between the sensors. It then sends a
  Bluetooth alert with detailed timing to a mobile phone or tablet.

  The custom-built sensor hardware consist of:
  - an AVR microprocessor
  - a Bluetooth LE radio module
  - 940nm IR LED transmitter powered by a constant-current driver
  - 940nm 38 Khz tuned IR receiver
  - red/green battery status light
  - blue and yellow operational status lights
  - power on/off pushbutton
  - program-controlled power supply circuit
  - enclosure with battery holder

Version 2 in 2017 used the Adafruit Bluefruit Feather 32U4 with an 
integrated Bluetooth module. The board used mostly through-hole 
components, and required tricky alignment of the LEDs into holes in the 
case. It was powered by 4 AA batteries in a rather clunky-looking square 
enclosure. 

Version 3 in 2020 uses a "raw" ATMega328P processor and a Microchip 
RN4870 Bluetooth module. All components are now surface mounted, 
including the LEDs which sit below light pipes in the cover. It is 
powered by a 9V battery in a sleeker case that looks more like a 
remote control. 
