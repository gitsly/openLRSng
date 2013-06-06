#ifndef _packet_h_included_
#define _packet_h_included_


//////////////////////////

// Packet type
// 
enum RxToTxPacketType
{
	Pkt_SerialData,
	Pkt_Status,
	Pkt_Debug,
};

struct RxToTxHeader
{
	uint8_t type;
	uint8_t sequenceNumber; // DEBUG
};

struct RxToTxStatus
{
	RxToTxHeader header;

	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t rssi;
	uint8_t txbuf;
	uint8_t noise;
};

struct RxToTxSerialData
{
	RxToTxHeader header;
	uint8_t data[9];
};


union RxToTxPacket
{
	RxToTxHeader header;
	RxToTxSerialData serial;
	RxToTxStatus status;
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