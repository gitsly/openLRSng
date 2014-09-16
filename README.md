
===========================================
	This is a fork of openLRSng code by kah (which in turn is based on thUndeadMod of openLRS).
	The project is geared towards the ArduPilot platform.
	This software enables you to use your openLRS hardware compatible RC receiver and transmitter as a telemetry module. The 3DR telemetry module becomes redundant.
	The telemetry link is pretty much the same as the one found in original openLRSng project except for:
		- Variable (selectable through CLI) downlink telemetry packet size.
		- Injection of mavlink radio modem status packets. Lets ArduPilot adjust packet send rate automatically in order to avoid lost packets due to overload.
		
	All features from openLRSng v3.5 are still supported, except for:
		- FRSKY emulation or smartport (will not work, planning to make this supported by making mavlink a setting).
		
		
	A note to those wanting to use a joystick to control the mav: It could possible be done, but the focus on this project

Useful links:
	http://diydrones.com/forum/topics/telemetry-openlrs-and-apm?id=705844%3ATopic%3A711570&page=3#comments


USING THE SOFTWARE WITH MISSION PLANNER
=======================================

	Connect the telemetry port to the OpenLRS Rx.
		- The wires that need connection between the ardupilot and the Rx is the following:
			APM TX -> OpenLRS RX
			APM RX -> OpenLRS TX
			APM GND -> OpenLRS GND
			Do NOT connect the 5V wire to the OpenLRS Rx since this will damage the 3.3V radio module. So you need a total of three wires.
			If ground wire is already connected to the Rx unit which is probably true in most cases via the RC wiring, the only wires needed are Tx and Rx.

	Compile and upload the code to your Rx and Tx module.
		- Set correct sketchbook location in arduino IDE: Should point to the folder containing the openLRSng.ino file.
		- Follow the directions under section 'Upload'.

	Configure your TX via CLI (done in the same fashion as with original openLRSng):
		- Set telemetry baud rate (Default for ArduPilot is 57600)
		- Set RFM module datarate, recommended to be set to 2 (19200), inorder to be able to have decent update frequency (packet interval) and telemetry data size.
		- Set wanted telemetry packet size:
		  Value of this param will vary depending on your application.
		  Higher values will reduce your update frequency. Also since packets sent from Rx (downlink) will be
		  larger the risk of introducing bit errors within the packets will increase, which can decrease downlink quality and range.
		  However a high value will increase the number of bytes possible to transfer per second, which can be useful for some applications when you 
		  want good response on the artificial horizon etc.
		  There is currently no changes made to the uplink (Tx to Rx) communication, so the quality of the RC control will remain unaffected,
		  except for packet interval (update frequency).
		  Note that when update frequency is decreased it will also affect the link speed negatively. So this parameter needs some tweaking in-order to get good performance.
		  As a guideline you should stay within a value of 6-30 (maximum allowed is 64).
		  
	Configure Rx:
		- Bind to your Rx as usual, and configure wanted parameters.
		- Note about RSSI: the rssi is currently not sent from the Rx to Tx. This is because ardupilot can send the RSSI information within the mavlink protocol.
		  To be able to see your Rx RSSI you must connect the receiver RSSI pin to your configured RSSI analog input
		  on the ardupilot and set this up in mission planner, see: http://plane.ardupilot.com/wiki/arduplane-parameters/#Receiver-RSSI-sensing-pin-(ArduPlane-RSSI_PIN)
	
	Configure Mission Planner		
		- Set correct baud rate for telemetry port on your APM (do this while connected via USB to your ardupilot). Param: SERIAL3_BAUD 
		- Configure your data rates:
		  Under section 'Configure->Planner->Telemetry rates' you can set the rate for each type of telemetry. Note that since the link speed will be adjusted 
		  automatically you will not loose any information sent from the Rx by setting a rate too high but the rates are important inorder to adjust the portion of each 
		  telemetry type going through the telemetry link.
		  The data rate needed for each type will vary depending on your application. So for example if you only need position updates, set all other data rates to 0 and you'll receive 
		  position updates more frequently, since the telemetry link will only be used to send that type of data.
		  
		- Uncheck the checkbox 'Reset apm on USB connect' if you do not want your Tx to reset each time you connect.
		- Press connect.
		- Tip: you can click on the 'link status' text close to the connect button to see current link quality and status.
		
	
	Connect your groundstation computer to the serial port of the Tx. Use this port when connecting in MissionPlanner (also rememeber to use correct baud rate when connecting).


  Notes using with arducopter
   ArduCopter works ok without modifications, altough you'll need to set SR0_ params with care. Setting them all to zero will work,
   but will cause any MinimOsd connected to not recieve updates correctly. If you're using an OSD, you should adjust the values to
   minimize telemetry packet loss yet still have OSD working (don't have a recipe for this at the moment).
  
  Notes for arduplane
   Connection bug fix in ArduPlane software, to avoid having params being resent upon connection. Bug is noticable when receiving 'Already got param' during connection, but will cause no harm. 
    - see 'fix connection with lower bps telemetry, honour slowdown of telemetry': https://github.com/gitsly/ardupilot/commit/cfd2c77dda02ec308da6c0ace5077e38add9a75e


More information at http://openlrsng.org

CONFIGURATOR UTILITY / BINARY FIRMWARE NOTE:
============================================
  This software should be used in source form only by expert users, for normal use 'binary' firmwares can be uploaded and configured by free configuration software available for Windows/Mac/Linux from Chrome web store.

  http://goo.gl/iX7dJx

  Binary firmware images are also available from

  https://github.com/openLRSng/openlrsng.github.com/tree/master/binaries

TRANSMITTER HW:
===============
  - TX_BOARD_TYPE 2 (Arduino Mini/nano 328 16MHz)
    - Flytron openLRS M2 (100mW)/M3(1W) TX
    - OrangeRX UHF TX unit

  - TX_BOARD_TYPE 3 (RX module acting as TX) (Arduino Mini/nano 328 16MHz)
    - Flytron openLRS RX v2
    - OrangeRX UHF RX
    - HawkEye openLRS RX

    - connect PPM input to 5th slot from left (channel 4)
    - button between ground and 4th slot from left (ch3)
    - buzzer via transistor on 3rd slot (ch2) (active high)

  - TX_BOARD_TYPE 4 (Arduino Mini/nano 328 16MHz)
    - openLRSngTX ('kha' or DIY based on ngTX schematics)
    - HawkEye OpenLRSng TX

  - TX_BOARD_TYPE 5 (RX as TX) (Arduino Mini/nano 328 16MHz)
    - DTFUHF/HawkEye 4ch/6ch RX

  - TX_BOARD_TYPE 6 (Arduino Leonardo)
    - DTFUHF/HawkEye deluxe TX

  - TX_BOARD_TYPE 7 (RX as TX)
    - Brotronics PowerTowerRX

  - RX_BOARD_TYPE 8 (RX as TX)
    - openLRSng microRX

RECEIVER HW:
============
  - RX_BOARD_TYPE 2 (TX module as RX) (328P 16MHz)

  - RX_BOARD_TYPE 3 (Arduino Mini/nano 328 16MHz)
    - Flytron openLRS RX
    - OrangeRX UHF RX (NOTE both LEDs are RED!!)
    - HawkEye OpenLRS RX

  - RX_BOARD_TYPE 5 (Arduino Mini/nano 328 16MHz)
    - DTFUHF/HawkEye 4ch/6ch RX

  - RX_BOARD_TYPE 7
    - Brotronics PowerTowerRX

  - RX_BOARD_TYPE 8
    - openLRSng microRX

  Receiver pin functiontions are mapped using the configurator or CLI interface.

SOFTWARE CONFIGURATION:
=======================
  Only hardware related selections are done in the openLRSng.ino.

  Run time configuration is done by connecting to the TX module (which is put into binding mode) with serial terminal. For best restults use real terminal program like Putty, TeraTerm, minicom(Linux) but it is possible to use Arduino Serial Monitor too.
  Sending '<CR>' (enter) will enter the menu which will guide further. It should be noted that doing edits will automatically change 'RF magic' to force rebinding, if you want to use a specific magic set that using the command (and further automatic changes are ceased for the edit session).

  Datarates are: 0==4800bps 1==9600bps 2==19200bps 3==57600bps 4==125kbps

  Refer to http://openlrsng.org

UPLOADING:
==========
Use a 3v3 FTDI (or other USB to TTL serial adapter) and Arduino >= 1.0.

  o set board to "Arduino Pro or Pro Mini (5V, 16MHz) w/ atmega328" (yes it runs really on 3v3 but arduino does not need to know that)

  o define COMPILE_TX and upload to TX module

  o comment out COMPILE_TX and upload to RX

  o for deluxeTX "arduino boardtype" must be set to "Leonardo"


USERS GUIDE
===========

Please refer to http://openlrsng.org (->wiki)
