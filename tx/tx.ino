/****************************************************
 * OpenLRSng transmitter code
 ****************************************************/
//extern "C" void __cxa_pure_virtual()
//{
//  cli();
//  for (;;);
//}


#include <FastSerial.h>  // this is now preventing program from running properly!
#include <BetterStream.h>
#include <AP_Common.h>
#include <AP_Math.h>

#include <Arduino.h>
#include <EEPROM.h>

#include "config.h"
#include "hardware.h"
#include "binding.h"
#include "common.h"
#include "packet.h"

#if MAVLINK_INJECT == 1
	#include <GCS_MAVLink.h>
	#include "mavlink.h"
#endif
 
//#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//static int packet_drops = 0;
//static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */
 

FastSerialPort0(Serial);


uint8_t remote_RSSI = 0;
uint16_t remote_rxerrors = 0;

uint8_t RF_channel = 0;

uint8_t FSstate = 0; // 1 = waiting timer, 2 = send FS, 3 sent waiting btn release
uint32_t FStime = 0;  // time when button went down...

uint32_t lastSent = 0;

uint32_t lastTelemetry = 0;


volatile uint8_t ppmAge = 0; // age of PPM data


volatile uint16_t startPulse = 0;
volatile uint8_t  ppmCounter = PPM_CHANNELS; // ignore data until first sync pulse




#define TIMER1_FREQUENCY_HZ 50
#define TIMER1_PRESCALER    8
#define TIMER1_PERIOD       (F_CPU/TIMER1_PRESCALER/TIMER1_FREQUENCY_HZ)

#ifdef USE_ICP1 // Use ICP1 in input capture mode
/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER1_CAPT_vect)
{
  uint16_t stopPulse = ICR1;

  // Compensate for timer overflow if needed
  uint16_t pulseWidth = ((startPulse > stopPulse) ? TIMER1_PERIOD : 0) + stopPulse - startPulse;

  if (pulseWidth > 5000) {      // Verify if this is the sync pulse (2.5ms)
    ppmCounter = 0;             // -> restart the channel counter
    ppmAge = 0;                 // brand new PPM data received
  } else if ((pulseWidth > 1400) && (ppmCounter < PPM_CHANNELS)) {       // extra channels will get ignored here
    PPM[ppmCounter] = servoUs2Bits(pulseWidth / 2);   // Store measured pulse length (converted)
    ppmCounter++;                     // Advance to next channel
  } else {
    ppmCounter = PPM_CHANNELS; // glitch ignore rest of data
  }

  startPulse = stopPulse;         // Save time at pulse start
}

void setupPPMinput()
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, top at 20ms)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << ICES1));
  OCR1A = TIMER1_PERIOD;
  TIMSK1 |= (1 << ICIE1);   // Enable timer1 input capture interrupt
}
#else // sample PPM using pinchange interrupt
ISR(PPM_Signal_Interrupt)
{
  uint16_t time_temp;

  if (PPM_Signal_Edge_Check) {   // Only works with rising edge of the signal
    time_temp = TCNT1; // read the timer1 value
    TCNT1 = 0; // reset the timer1 value for next

    if (time_temp > 5000) {   // new frame detection (>2.5ms)
      ppmCounter = 0;             // -> restart the channel counter
      ppmAge = 0;                 // brand new PPM data received
    } else if ((time_temp > 1400) && (ppmCounter < PPM_CHANNELS)) {
      PPM[ppmCounter] = servoUs2Bits(time_temp / 2);   // Store measured pulse length (converted)
      ppmCounter++;                     // Advance to next channel
    } else {
      ppmCounter = PPM_CHANNELS; // glitch ignore rest of data
    }
  }
}

void setupPPMinput(void)
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, top at 20ms)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));
  OCR1A = TIMER1_PERIOD;
  TIMSK1 = 0;
  PPM_Pin_Interrupt_Setup
}
#endif

void handleCLI(char c)
{
  switch (c) {
  case '?':
    bindPrint();
    break;

  case '#':
    digitalWrite(BUZZER, LOW);
    scannerMode();
    break;
  }
}

void bindMode(void)
{
  uint32_t prevsend = millis();
  init_rfm(1);

  while (Serial.available()) {
    Serial.read();    // flush serial
  }

  while (1) {
    if (millis() - prevsend > 200) {
      prevsend = millis();
      Green_LED_ON;
      digitalWrite(BUZZER, HIGH);   // Buzzer on
      tx_packet((uint8_t*)&bind_data, sizeof(bind_data));
      Green_LED_OFF;
      digitalWrite(BUZZER, LOW);   // Buzzer off
    }

    while (Serial.available()) {
      handleCLI(Serial.read());
    }
  }
}

void checkButton(void)
{

  uint32_t time, loop_time;

  if (digitalRead(BTN) == 0) {     // Check the button
    delay(200);   // wait for 200mS when buzzer ON
    digitalWrite(BUZZER, LOW);   // Buzzer off

    time = millis();  //set the current time
    loop_time = time;

    while ((digitalRead(BTN) == 0) && (loop_time < time + 4800)) {
      // wait for button reelase if it is already pressed.
      loop_time = millis();
    }

    // Check the button again, If it is still down reinitialize
    if (0 == digitalRead(BTN)) {
      int8_t bzstate = HIGH;
      uint8_t doRandomize = 1;

      digitalWrite(BUZZER, bzstate);
      loop_time = millis();

      while (0 == digitalRead(BTN)) {     // wait for button to release
        if (loop_time > time + 9800) {
          digitalWrite(BUZZER, HIGH);
          doRandomize = 0;
        } else {
          if ((millis() - loop_time) > 200) {
            loop_time = millis();
            bzstate = !bzstate;
            digitalWrite(BUZZER, bzstate);
          }
        }
      }

      digitalWrite(BUZZER, LOW);
      randomSeed(micros());   // button release time in us should give us enough seed
      bindInitDefaults();
      if (doRandomize) {
        bindRandomize();
      }
      bindWriteEeprom();
      bindPrint();
    }

    // Enter binding mode, automatically after recoding or when pressed for shorter time.
    Serial.println("Entering binding mode\n");
    bindMode();
  }
}

void checkFS(void)
{

  switch (FSstate) {
  case 0:
    if (!digitalRead(BTN)) {
      FSstate = 1;
      FStime = millis();
    }

    break;

  case 1:
    if (!digitalRead(BTN)) {
      if ((millis() - FStime) > 1000) {
        FSstate = 2;
        digitalWrite(BUZZER, HIGH);   // Buzzer on
      }
    } else {
      FSstate = 0;
    }

    break;

  case 2:
    if (digitalRead(BTN)) {
      digitalWrite(BUZZER, LOW);   // Buzzer off
      FSstate = 0;
    }

    break;
  }
}

void setup(void)
{

  //RF module pins
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL

  //LED and other interfaces
  pinMode(Red_LED, OUTPUT);   //RED LED
  pinMode(Green_LED, OUTPUT);   //GREEN LED
  pinMode(BUZZER, OUTPUT);   //Buzzer
  pinMode(BTN, INPUT);   //Buton

  pinMode(PPM_IN, INPUT);   //PPM from TX
  digitalWrite(PPM_IN, HIGH); // enable pullup for TX:s with open collector output

#if defined (RF_OUT_INDICATOR)
  pinMode(RF_OUT_INDICATOR, OUTPUT);
  digitalWrite(RF_OUT_INDICATOR, LOW);
#endif

  Serial.begin(SERIAL_BAUD_RATE, SERIAL_RX_BUFFERSIZE, SERIAL_TX_BUFFERSIZE);
  Serial.set_blocking_writes(true);

  if (bindReadEeprom()) {
    Serial.print("Loaded settings from EEPROM\n");
  } else {
    Serial.print("EEPROM data not valid, reiniting\n");
    bindInitDefaults();
    bindWriteEeprom();
    Serial.print("EEPROM data saved\n");
  }

#ifdef TX_TIMING
  Serial.print("Tx->Rx packet size: ");
  Serial.println(sizeof(TxToRxPacket));
  Serial.print("Rx->Tx packet size: ");
  Serial.println(sizeof(RxToTxPacket));
#endif

	//mavlink_message_t mavlink_msg; // 272 byte in original size, 112 after including GCS_Mavlink header from arduplane.
	//mavlink_status_t mavlink_status; // 12 bytes in size.
	//mavlink_parse_char(MAVLINK_COMM_0, 't', &mavlink_msg, &mavlink_status);
	//char tmp[24];
	//sprintf(tmp, "came here: %d\n", sizeof(mavlink_message_t));
	//Serial.write(tmp);	


  setupPPMinput();

  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);

  init_rfm(0);
  rfmSetChannel(bind_data.hopchannel[RF_channel]);

  sei();

  digitalWrite(BUZZER, HIGH);
  digitalWrite(BTN, HIGH);
  Red_LED_ON ;
  delay(100);

  checkButton();

  Red_LED_OFF;
  digitalWrite(BUZZER, LOW);

  ppmAge = 255;
  rx_reset();

}


void HandleReceivedSerialPacket(RxToTxSerialData* pkt)
{
#if MAVLINK_INJECT == 1
	mavlink_message_t mavlink_msg; // 272 byte in original size, 112 after including GCS_Mavlink header from arduplane.
	mavlink_status_t mavlink_status; // 12 bytes in size.
	uint8_t sequenceNumber = 1;

	for (uint8_t i = 0; i < sizeof(pkt->data); i++)
	{ 
		Serial.write(pkt->data[i]);

		if(mavlink_parse_char(MAVLINK_COMM_0, pkt->data[i], &mavlink_msg, &mavlink_status))
		{
			sequenceNumber = mavlink_msg.seq;
			// Inject radio info every X mavlink packets.
			if ((sequenceNumber % 40) == 0)
			{
				// Inject Mavlink radio modem status package.
				MAVLink_report(remote_RSSI, remote_rxerrors);
			}

			/* // DEBUG

			switch (mavlink_msg.msgid)
			{
			  case MAVLINK_MSG_ID_HEARTBEAT:
				  Serial.write("MAVLINK_MSG_ID_HEARTBEAT");
				  break;
			  case MAVLINK_MSG_ID_SYS_STATUS:
				  Serial.write("MAVLINK_MSG_ID_SYS_STATUS");
				  break;
			  case MAVLINK_MSG_ID_GPS_RAW_INT:
				  Serial.write("MAVLINK_MSG_ID_GPS_RAW_INT");
				  break;
			  case MAVLINK_MSG_ID_VFR_HUD:
				  Serial.write("MAVLINK_MSG_ID_VFR_HUD");
				  break;
			  case MAVLINK_MSG_ID_ATTITUDE:
				  Serial.write("MAVLINK_MSG_ID_ATTITUDE");
				  break;
			  case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
				  Serial.write("MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT");
				  break;
			  case MAVLINK_MSG_ID_MISSION_CURRENT:
				  Serial.write("MAVLINK_MSG_ID_MISSION_CURRENT");
				  break;
			  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
				  Serial.write("MAVLINK_MSG_ID_RC_CHANNELS_RAW");
				  break;
			  case MAVLINK_MSG_ID_WIND:
				  Serial.write("MAVLINK_MSG_ID_WIND");
				  break;
			}

			char tmp[24];
			sprintf(tmp, " (%d)\n", mavlink_msg.seq);
			Serial.write(tmp);	
			*/

		}
	}
#else
	Serial.write(pkt->data, sizeof(pkt->data));
#endif
}


static uint16_t radioIDCounter = 0;
static RxToTxPacket recievePacket;
void HandleReceivedPacket()
{
    lastTelemetry = micros();
    RF_Mode = Receive;
    spiSendAddress(0x7f);   // Send the package read command
	uint8_t *buf = (uint8_t *)&recievePacket;
	for (uint8_t i = 0; i < sizeof(recievePacket); i++)
	{
		buf[i] = spiReadData();
	}

	switch (recievePacket.type)
	{
	case Pkt_SerialData:
		HandleReceivedSerialPacket(&recievePacket.serial);
		break;
	case Pkt_Status:
		remote_RSSI = recievePacket.status.rssi;
		remote_rxerrors = recievePacket.status.rxerrors;
		break;
	}
}

void loop(void)
{

  if (spiReadRegister(0x0C) == 0) {     // detect the locked module and reboot
    Serial.println("module locked?");
    Red_LED_ON;
    init_rfm(0);
    rx_reset();
    Red_LED_OFF;
  }

  if (RF_Mode == Received) {
    HandleReceivedPacket();
  }

  uint32_t time = micros();

  if ((time - lastSent) >= modem_params[bind_data.modem_params].interval) {
    lastSent = time;

    if (1) {

      ppmAge++;

      if (lastTelemetry) {
        if ((time - lastTelemetry) > modem_params[bind_data.modem_params].interval) {
          // telemetry lost
          digitalWrite(BUZZER, HIGH);   // Buzzer on
          lastTelemetry=0;
        } else {
          // telemetry link re-established
          digitalWrite(BUZZER, LOW);   // Buzzer off
        }
      }

      // Construct packet to be sent
  	  TxToRxPacket packet;
	  packet.header = 0;
	  
	  if (FSstate == 2) {
        packet.header |= Header_FailSafe; // save failsafe
        Red_LED_ON
      } else {
        Red_LED_OFF
      }

	  cli(); // disable interrupts when copying servo positions, to avoid race on 2 byte variable
      //packet.SetPPMValues(PPM)// TODO: implement method in packet class for populating below fields.
      packet.ppmLow0to3[0] = (PPM[0] & 0xff);
      packet.ppmLow0to3[1] = (PPM[1] & 0xff);
      packet.ppmLow0to3[2] = (PPM[2] & 0xff);
      packet.ppmLow0to3[3] = (PPM[3] & 0xff);
      packet.ppmHigh0to3 = ((PPM[0] >> 8) & 3) | (((PPM[1] >> 8) & 3) << 2) | (((PPM[2] >> 8) & 3) << 4) | (((PPM[3] >> 8) & 3) << 6);
      packet.ppmLow4to7[0] = (PPM[4] & 0xff);
      packet.ppmLow4to7[1] = (PPM[5] & 0xff);
      packet.ppmLow4to7[2] = (PPM[6] & 0xff);
      packet.ppmLow4to7[3] = (PPM[7] & 0xff);
      packet.ppmHigh4to7 = ((PPM[4] >> 8) & 3) | (((PPM[5] >> 8) & 3) << 2) | (((PPM[6] >> 8) & 3) << 4) | (((PPM[7] >> 8) & 3) << 6);
      sei();

	  // Fill telemetry portion of packet
	  packet.header |= Header_SerialData;
	  packet.SetDataLength(getSerialData(packet.data, sizeof(packet.data)));

      //Green LED will be on during transmission
      Green_LED_ON ;

      // Send the data over RF
      rfmSetChannel(bind_data.hopchannel[RF_channel]);
      tx_packet((uint8_t*)&packet, sizeof(packet));

      //Hop to the next frequency
      RF_channel++;

      if (RF_channel >= bind_data.hopcount) {
        RF_channel = 0;
      }

      // do not switch channel as we may receive telemetry on the old channel
      if (modem_params[bind_data.modem_params].flags & 0x01) {
        RF_Mode = Receive;
        rx_reset();
      }

    } else {
      if (ppmAge == 8) {
        Red_LED_ON
      }

      ppmAge = 9;
      // PPM data outdated - do not send packets
    }

  }

  //Green LED will be OFF
  Green_LED_OFF;

  checkFS();
}

