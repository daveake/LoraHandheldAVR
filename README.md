This is a simple AVR-based LoRa receiver that connects to a standard Epson-compatible 2-line LCD to display basic telemetry data from a high altitude balloon.

Hardware
========

The LoRa module (e.g. RFM98) connects via SPI.  Since  LoRa modules are 3.3V logic it's best to use an Arduino with 3.3V logic, such as the Arduino Mini Pro.

Optionally, a simple GPS receiver can be added, to provide the distance and direction to the HAB.  Any GPS module that provides RS232 NMEA data at 9600 baud, over 3.3V logic level signals, should be fine.

Any 2-line EPson-compatible LCD should be fine.  Only 16 columns are used. 

Connections
===========

	Arduino A0 - Switch (other side GND)
	Arduino Rx - GPS Tx NMEA 9600 baud
	Arduino  2 - RFM DIO5
	Arduino  3 - RFM DIO0
	Arduino  4 - LCD D4
	Arduino  5 - LCD D5
	Arduino  6 - LCD D6
	Arduino  7 - LCD D7
	Arduino  8 - LCD EN
	Arduino  9 - LCD RS
	Arduino 10 - RFM NSS
	Arduino 11 - RFM MOSI
	Arduino 12 - RFM MISO
	Arduino 13 - RFM CLK

Usage
=====

The software has 5 screens:

	GPS Position
	HAB Position
	Direction/Distance to HAB
	Frequency display/edit
	Mode display/edit
	Frequency offset display/edit

GPS Position screen shows the time (mm:ss), your latitude, longitude and altitude, assuming GPS has a lock

HAB Position screen shows HAB Altitude, packet RSSI, Latitude, Longitude.  After 5 secs RSSI is replaced by live RSSI bar; after 10 seconds altitude is replaced by time since packet was received.
 
Direction/Distance to HAB shows clock direction and horizontal distance.

Frequency display/edit screen shows the manually settable frequency.  To adjust, hold button till display changes, then press button briefly to step through the available frequencies (434.000 to 434.650MHz in 25kHz steps)  When done, press and hoild button to save.

Mode display/edit screen shows the LoRa mode which is "Slow" (mode 0 in my tracker software), "Fast" (mode 1) or "TDM" (mode 2).  As above, hold button down to enter/exit edit mode, and press button briefly to change the setting in edit mode.

Frequency offset display/edit screen shows the frequency offset applied when setting the LoRa module frequency.  These modules can be away from the nominal frequency by a few kHz and this screen allows you to apply an offset of up to +/-5kHz.  If the device receives packets whilst on this screen, the frequency offset from the transmission is displayed, making it easy to reduce that to zero.

The software includes AFC (automatic frequency control) to keep the receiver locked to the transmitter.  On startup the module is set to the configured frequency and offset.  Once a packet is received, the frequency error is measured and if it's more than +/- 500Hz then the module is retuned.

To avoid confusion when setting the frequency offset, AFC is disabled and the AFC offset is reset to zero when the frequency offset screen is placed in edit mode.  This allows the manual offset to be set without AFC adjusting the LoRa module's frequency.  


	 
History
=======

28/02/2018 - Fixed AFC, added frequency offset screen, added mode 2 (TDM), tified up serial output, fixed RSSI calculation

22/06/2015 - Freq control, AFC, UKHAS CRC16 check

05/06/2015 - LCD/button user interface, GPS, direction-finding

11/09/2014 - First release