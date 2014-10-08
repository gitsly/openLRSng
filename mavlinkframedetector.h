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

#define MAVLINK_PACKET_START 0xFE

	
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
				m_payloadLength = ch;
				m_state = MavParse_PacketSequence;
				break;
			case MavParse_Payload:
				if (++m_payloadByteParsedCount >= m_payloadLength)
				{
					++m_state;
				}
				break;
			case MavParse_PacketSequence:
			case MavParse_SystemID:
			case MavParse_ComponentID:
			case MavParse_MessageID:
			case MavParse_CRC1:
				++m_state;
				break;
			case MavParse_CRC2:
				m_state = MavParse_Idle;
				return true;
		}
		return false;
	}
	
	void Reset()
	{
		m_payloadLength = 0;
		m_payloadByteParsedCount = 0; // clear helper
		m_state = MavParse_Idle;
	}

	bool IsIdle()
	{
		return m_state == MavParse_Idle;
	}

private:
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

	uint8_t m_payloadLength;
	uint8_t m_payloadByteParsedCount;
	uint8_t m_state;
};

