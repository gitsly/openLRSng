//Field name			Index (Bytes)	Purpose
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Start-of-frame		0				Denotes the start of frame transmission (v1.0: 0xFE)
// Payload length		1				Length of the following payload
// Packet sequence		2				Each component counts up his send sequence. Allows to detect packet loss
// System ID			3				Identification of the SENDING system. Allows to differentiate different systems on the same network.
// Component ID			4				Identification of the SENDING component. Allows to differentiate different components of the same system, e.g. the IMU and the autopilot.
// Message ID			5				Identification of the message - the id defines what the payload �means� and how it should be correctly decoded.
// Payload				6 to (n+6)		The data into the message, depends on the message id.
// CRC					(n+7) to (n+8)	Check-sum of the entire packet, excluding the packet start sign (LSB to MSB)
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8
#define MAVLINK_PACKET_START 0xFE

// Message CRC seeds are taken from arduplane 3.0.3 (ardupilot\libraries\GCS_MAVLink\include\mavlink\v1.0\ardupilotmega\ardupilotmega.h)
//#define MAVLINK_MESSAGE_CRCS {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 30, 240, 183, 130, 130, 118, 148, 21, 0, 243, 124, 0, 0, 0, 20, 0, 152, 143, 0, 0, 127, 106, 0, 0, 0, 0, 0, 0, 0, 231, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 211, 108, 32, 185, 235, 93, 124, 124, 119, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 0, 0, 0, 29, 223, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 177, 241, 15, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138, 234, 240, 47, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 0}
//static uint8_t seedCrc[] = MAVLINK_MESSAGE_CRCS;
	
class MavlinkFrameDetector
{
	public:

	MavlinkFrameDetector()
	{
		Reset();
	}

	// Returns true if a mavlink frame has been detected.
	bool Parse(uint8_t ch)
	{
		switch (m_state)
		{
			case MavParse_Idle:
			if (ch == MAVLINK_PACKET_START)
			{
				Reset();
				m_state = MavParse_PayloadLength;
			}
			break;
			case MavParse_PayloadLength:
			//AccumulateCRC(ch);
			m_payloadLength = ch;
			m_state = MavParse_PacketSequence;
			break;
			case MavParse_PacketSequence:
			//AccumulateCRC(ch);
			m_packetSequence = ch;
			m_state = MavParse_SystemID;
			break;
			case MavParse_SystemID:
			//AccumulateCRC(ch);
			m_systemID = ch;
			m_state = MavParse_ComponentID;
			break;
			case MavParse_ComponentID:
			//AccumulateCRC(ch);
			m_componentID = ch;
			m_state = MavParse_MessageID;
			break;
			case MavParse_MessageID:
			//AccumulateCRC(ch);
			m_messageID = ch;
			m_state = MavParse_Payload;
			break;
			case MavParse_Payload:
			//AccumulateCRC(ch);
			if (++m_payloadByteParsedCount >= m_payloadLength)
			{
				m_state = MavParse_CRC1;
				//AccumulateCRC(seedCrc[m_messageID]); // MAVLink 1.0 has an extra CRC seed
			}
			break;
			case MavParse_CRC1:
			m_CRC = ch;
			m_state = MavParse_CRC2;
			break;
			case MavParse_CRC2:
			m_CRC |= (uint16_t)ch << 8;
			m_state = MavParse_Idle;
			//return m_CRC == m_accumulatedCRC;
			return true;
		}
		return false;
	}
	
	void Reset()
	{
		m_payloadLength = 0;
		m_packetSequence = 0;
		m_systemID = 0;
		m_componentID = 0;
		m_messageID = 0;
		m_CRC = 0;

		// Setup helpers
		m_payloadByteParsedCount = 0; // clear helper
		m_state = MavParse_Idle;
		//		m_accumulatedCRC = X25_INIT_CRC;
	}

	bool IsIdle() // Returns true if currently no mavlink packet is being parsed.
	{
		return m_state == MavParse_Idle;
	}	

//	private:
	
	//inline void AccumulateCRC(uint8_t data)
	//{
	//	uint8_t tmp;
	//	tmp = data ^ (uint8_t)(m_accumulatedCRC & 0xff);
	//	tmp ^= (tmp << 4);
	//	m_accumulatedCRC = (m_accumulatedCRC >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
	//}

	// Mavlink frame data (except for payload).
	uint8_t m_payloadLength;
	uint8_t m_packetSequence;
	uint8_t m_systemID;
	uint8_t m_componentID;
	uint8_t m_messageID;
	uint16_t m_CRC;

	enum MavlinkParseState
	{
		MavParse_Idle,
		MavParse_PayloadLength,
		MavParse_PacketSequence,
		MavParse_SystemID,
		MavParse_ComponentID,
		MavParse_MessageID,
		MavParse_Payload,
		MavParse_CRC1,
		MavParse_CRC2
	};

	// Parse helpers
	//	uint16_t m_accumulatedCRC;
	uint8_t m_payloadByteParsedCount;
	MavlinkParseState m_state;
};

