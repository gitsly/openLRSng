#ifndef _packet_h_included_
#define _packet_h_included_


//////////////////////////

// Packet type
// 
enum RxToTxPacketType
{
	Pkt_SerialData,
	Pkt_RemoteLostPacketCount,
	Pkt_RemoteNoise, 
	Pkt_RemoteTxBuffer,
	Pkt_Debug,
};


struct RxToTxSerialData
{
	enum Constants
	{
		MaxRxToTx_DataLength = 11
	};

	// Idea: for lower baudrates, it makes no sense wasting an entire byte for size & misc.
	// Use the byte in header to be able to flag when serial is contained within the packet
	// also content of misc could be flagged in this byte!

	// Content (of 'miscDataByte') could be varied through flags stuffed into dataLength byte.
	// 5 bits can specify datalength from 0-32, the 3 remaining bits can be used to distinguish
	// what miscDataByte means. And meaning of data can vary between each packet sent from Rx.
	// See 'MiscDataFlags' enum for a proposet dataset.
	uint8_t packetType; 
	uint8_t data[MaxRxToTx_DataLength];
};

union RxToTxPacket
{
	uint8_t type;
	RxToTxSerialData serial;
};

//////////////////////////////////////////////////////////////////////////////////////////

struct TxToRxPacket
{
public:
	enum Constants
	{
		MaxTxToRx_DataLength = 1
	};

	// 0xF5 = 0b11110101 => save failsafe
	// 0x5E = 0b01011110 => servo positions
	uint8_t packetFlags;  // This header byte could contain lot more info, use it more wisely!

	// TODO: these fields below should be private, and set properly by accessor methods.
	// ppm ranges from 0-1023 (10 bits per ppm channel).
	uint8_t ppmLow0to3[4];
	uint8_t ppmHigh0to3; // 2 msb per ppm channel 0-4 

	uint8_t ppmLow4to7[4];
	uint8_t ppmHigh4to7; // 2 msb per ppm channel 0-4 

	// TODO: accessor methods for ppm channels into structure bytes
	uint8_t dataLength; // Length in bytes of accessory data stream (transparent serial link).
	uint8_t data[MaxTxToRx_DataLength];
};



#endif