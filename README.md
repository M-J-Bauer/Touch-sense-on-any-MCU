# A touch-sense technique that works on any MCU 

Some micro-controller devices have on-chip hardware to support reading of capacitive touch-pads. 
These devices provide “automatic” measurement of capacitance between an I/O pin and earth (GND). 
However, a change in capacitance on an input pin can be detected quite easily without any special on-chip wizardry.
This method requires a resistor and diode for each touch input.

The processing overhead to service the touch-pads is very low. 
The interval between calls to the touch-pad “service routine” may be anywhere in the range 100 microseconds up to 5 milliseconds,
or more.  Execution time of the service routine itself is under 30 microseconds for each touch input. 

The micro-controller must have I/O pins which can be configured as either digital (GPIO) 
or analog (ADC inputs). One such pin is required for each touch input.

The touch-sense technique has been tested successfully on a “low-end” 8-bit AVR micro-controller
(ATmega328P) as found in many popular development boards including the Arduino Uno R3 and Nano boards.

The Test & Demo program presented here was designed to run on a Microchip AVR 'X-mini' board which is compatible
with the Arduino Uno and Nano. The program was developed under Microchip Studio IDE for AVR and SAM Devices.

The Touch-Sense algorithm is easily adapted to other micro-controllers.

A full description is provided in the PDF document: "Touch sense for any MCU".
