// **********************************************************
// ************************ openLRSng ***********************
// **********************************************************
// ** by Kari Hautio - kha @ AeroQuad/RCGroups/IRC(Freenode)
// ** other commits by cTn-dev, rlboyd, DTFUHF, pwarren
//
// Developer chat at IRC: #openLRS @ freenode
//
// This code is based on original OpenLRS and thUndeadMod
//
// Donations for development tools and utilities (beer) here
// https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=DSWGKGKPRX5CS
//
// Please note that for basic usage there is no need to use this
// code in source form. Instead use configurator program available
// freely in Google Chrome store.
// http://goo.gl/iX7dJx
//
// **********************************************************
// ************ based on: OpenLRS thUndeadMod ***************
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
//
// **********************************************************
// *************** based on: OpenLRS Code *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************

// **********************************************************
// **************** original OpenLRS DEVELOPERS *************
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne)

//################################
//### HW CONFIGURATION SECTION ###
//################################

// NOTE: All settings are made via the CLI or configurator interface at runtime

// To compile with Arduino select TX/RX and BOARD_TYPE setting as needed below

//####### COMPILATION TARGET #######
// Enable to compile transmitter code, default is RX
//#define COMPILE_TX

//####### TX BOARD TYPE #######
// 0 = Flytron OpenLRS M1 Tx Board (not verified)
// 1 = Flytron OpenLRS M1 Rx Board as TX (not verified)
// 2 = Flytron OpenLRS M2/M3 Tx Board / OrangeRx UHF TX
// 3 = Flytron OpenLRS Rx v2 Board / OrangeRx UHF RX / HawkEye UHF RX workking as TX
// 4 = OpenLRSngTX / HawkEye UHF TX
// 5 = OpenLRSngRX-4ch (DTF UHF) as TX
// 6 = DTF UHF DeluxeTX (Atmega32u4)
#ifdef COMPILE_TX
	#define BOARD_TYPE 3

#else
//####### RX BOARD TYPE #######
// 3 = Flytron OpenLRS Rx v2 / OrangeRx UHF RX / HawkEye UHF RX
// 5 = OpenLRSngRX-4ch (DTF UHF)
	#define BOARD_TYPE 3

#endif

#define REVERSE_PPM_RSSI_SERVO 1


//### Module type selection (only for modified HW)
//#define RFMXX_868
//#define RFMXX_915


//####### MAVLink #######
// 0 = Disable mavlink injection, normal serial bridge
// 1 = Inject mavlink radio status between incoming mavlink packets (3DR radio telemtry style)
// 2 = Inject mavlink radio status on serial with fixed intervals, disregarding other data on serial line (Useful for connecting to MinimOSD when not running an APM, only for showing radio status stuff on OSD).
#define MAVLINK_INJECT 2 
#define MAVLINK_INJECT_INTERVAL 100000

#ifdef COMPILE_TX
#define SERIAL_RX_BUFFERSIZE 128
#define SERIAL_TX_BUFFERSIZE 64
#else // COMPILE_RX
#define SERIAL_RX_BUFFERSIZE 256
#define SERIAL_TX_BUFFERSIZE 64
#endif


//####################
//### CODE SECTION ###
//####################

#include <FastSerial.h>
#include <BetterStream.h>
#include <AP_Common.h>
#include <AP_Math.h>

#include <Arduino.h>
#include <EEPROM.h>

#include "version.h"
#include "hardware.h"
#include "binding.h"
#include "common.h"

#include <mavlink.h>
#include <mavlinkframedetector.h>


#ifdef COMPILE_TX
#include "binary_com.h"
#include "dialog.h"
#include "frskytx.h"
#include "TX.h"
#else // COMPILE_RX
#include "RX.h"
#endif
