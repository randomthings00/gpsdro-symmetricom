#    Symmetricom GPS Disciplining Code in Python3 and MicroPython

## Description

 This program was written to do an initial discipline on a Symmetricom Rubidium Oscillator based on the internal 1PPS Delta register.   After the disciplining is done, the code will adjust the frequency through an averaging of the PPS Delta returned when the external 1PPS is used.

The 1PPS signal itself can come from any number of sources including a GPS module, a disciplined source driving a PICDIV, or a signal generator.  The more stable the source is the more reliable the discipline is.

### Cavets
There is natural variability to the PPS signal source from the GPS module, however, using a part of the taming code from Lady Heather it is able to discipline the Symmetricom and keep it in-sync with the GPS module without a phase lock.  Some of the inaccuracies also come from the internal Pico clock, the processing code and the time slices.

## Testing
I’ve tested this against a Datum StarLoc2, using a surveyed completed uBlox LEA-8T, and Trimble 66266-45.  I also tested against an Allystar TAU1202, and ATGM336H in 2D stationary mode.

All the GPS modules were set to use all constellations they were capable of receiving. My favourite is the Trimble, with the LEA-8T coming in a close second place, and the TAU1202 in third and the ATGM336H last, but for what they are, all are good options.

This was tested with a Symmetricom X99, SA.22c, MicroPython and Python3.

MicroPython Platform: BPI-PicoW-S3, Raspberry Pi Pico, and ESP32-C3 OLED.  
Python3: LuckFox Pico Mini B, and Raspberry Pi Zero.

### Holdover Behaviour
Testing for the lost of the Rubidium Lock was tested, and a full re-discipline is done whenthe lock is regained.  If there is a lost in the PPS, it continues to run, presuming teh word will re-adjust itself.

## Setup
### GPS 
The GPS module must be powered, but no data needs to be sent to the Pico/ESP32-S3/ESP32-C3.  The PPS signal should be wired up to the PPS input and the coaxial cable should be properly grounded, this reduced random signal spikes.

### Symmetricom
I did not cover the breakout for the Symmetricom but there are various breakout boards and schematics out there.  I made my own at the start but ended up using the boards for the SA.22c and X72 designed and created by BG7TBL and sold by various vendors on Ebay.

If you use the BG7TBL board ensure you also get a MAX3232 convert board to covert from the RS232 signal to the TTL, otherwise you can damage your Pico / ESP32.  You can modify the board to do a straight TTL pass-thru as well if you are inclined to do that.

## Symmetricom Commands

The commands used on the Symmetricom Rubidium Oscillator are:

<table><td><tr> 
<pre>
i – Oscillator Model Information Banner
w – Oscillator Health Output
j – 1 PPS Delta Information
k – Override or reset 1 PPS Delta
f – Set Frequency Adjustment
q - Set Control Register
</pre> 
</tr></td></table>

### Special Notes

There are additional commands that maybe used with caution and have not been implemented as part of this python code but can be useful under specific scenarios.

#### Saving configurations to your Symmetricom

You can save the settings of the ACMOS output, control register, and the frequency adjustment so that the settings remain persistent after a power reset.

To save the configuration you can connect to the Symmetricom UART and issue the command:

<table><td><tr> 
<pre>
t5988717 {enter}
</pre> 
</tr></td></table>

#### Factory reset

This should be used with caution, it's not well documented and I discovered it via a partial firmware extraction, not much is known except that the frequency adjustment is set to true zero.

<table><td><tr> 
<pre>
run mode> x5987717 {enter}
admin mode> f
factory mode> y
hard reset confirm prompt> y {enter}
</pre> 
</tr></td></table>
  
You should let the Symmetricom run for 6 hours and then discipline for another 24 to 48 hours then reprogram the frequency adjustment, control register and ACMOS output before saving.
