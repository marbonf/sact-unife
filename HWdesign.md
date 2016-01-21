# SACT board hardware #

![http://sact-unife.googlecode.com/svn/wiki/images/sact.jpg](http://sact-unife.googlecode.com/svn/wiki/images/sact.jpg)

The SACT board is based on a Microchip dsPIC30F6015 16-bit DSC (Digital Signal Controller), which is equipped with
  * two UART modules (for asynchronous serial communication)
  * a CAN controller
  * a Motor Control PWM module
  * a 10-bit A/D converter (ADC)
  * a Quadrature Encoder Interface (QEI)

In order to control two small DC servo motor, the board has two integrated H-Bridge modules (National LMD18200T) providing up to 6A
each. The LMD18200T includes also a current feedback circuit that needs only a standard external resistor for interfacing with
an A/D converter.

Two quadrature encoders can be interfaced to the dsPIC, one by means of the embedded QEI and another by means of an external
encoder interface circuit (an LS7183) that transforms quadrature pulses into up and down count pulses, which can be counted by
two standard timer modules of the dsPIC.

To conclude, one UART module of the dsPIC is connected to an RS-232 level shifter and the other one to a Lantronix
Serial-to-Ethernet device server (XPort on board rev.1 or WiF MatchPort on board rev.2).

The overall block diagram of the SACT board is the following:

![http://sact-unife.googlecode.com/svn/wiki/images/sact-blocks.jpg](http://sact-unife.googlecode.com/svn/wiki/images/sact-blocks.jpg)


