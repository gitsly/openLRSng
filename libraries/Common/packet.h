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


// [Flags] 
// Lowest 4 bits of header
enum TxToRxHeader
{
	Header_FailSafe = 1, // if bit not set, packet contains normal servo positions. 
	Header_SerialData = 2, // data portions contains serial
};

struct TxToRxPacket
{
public:
	enum Constants
	{
		MaxTxToRx_DataLength = 2
	};

	bool IsHeaderBitSet(uint8_t bitMask)
	{
		return (header & bitMask) != 0;
	}


	uint8_t GetDataLength()
	{
		return header >> 4;
	}

	void SetDataLength(uint8_t length)
	{
		header |= length << 4;
	}

	uint8_t header;  // This header byte could contain lot more info, use it more wisely!

	// TODO: accessor methods for ppm channels into structure bytes
	// TODO: these fields below should be private, and set properly by accessor methods.
	// ppm ranges from 0-1023 (10 bits per ppm channel).
	uint8_t ppmLow0to3[4];
	uint8_t ppmHigh0to3; // 2 msb per ppm channel 0-4 

	uint8_t ppmLow4to7[4];
	uint8_t ppmHigh4to7; // 2 msb per ppm channel 0-4 

	uint8_t data[MaxTxToRx_DataLength];
};



#endif