// disable the stock Arduino serial driver
#ifdef HardwareSerial_h
# error Arduino serial driver is already defined.
#endif
#ifndef __AVR_ATmega32U4__
#define HardwareSerial_h
#endif

#include <Stream.h>

// on the 32u4 the first USART is USART1
#ifndef __AVR_ATmega32U4__
extern class SerialPort serialPort;
//extern class SerialPort Serial1;
//extern class SerialPort Serial2;
//extern class SerialPort Serial3;
#endif


#if   defined(UDR3)
# define FS_MAX_PORTS   4
#elif defined(UDR2)
# define FS_MAX_PORTS   3
#elif defined(UDR1)
# define FS_MAX_PORTS   2
#else
# define FS_MAX_PORTS   1
#endif


#ifndef min
#define min(a,b) (a < b ? a : b)
#endif


/// Transmit/receive buffer descriptor.
///
/// Public so the interrupt handlers can see it
class RingBuffer {
public:
	volatile uint8_t head, tail;	///< head and tail pointers
	volatile uint16_t overflow;		///< Incremented every time the buffer can't fit a character.
	uint8_t mask;					///< buffer size mask for pointer wrap
	uint8_t *bytes;					///< pointer to allocated buffer

	void setBuffer(uint8_t* bufPtr, uint8_t size)
	{
		const uint8_t max_buffer_size = 0xff;
		uint16_t	msk;
		uint8_t		shift;

		// init buffer state
		head = tail = 0;

		// Compute the power of 2 greater or equal to the requested buffer size
		// and then a mask to simplify wrapping operations.  Using __builtin_clz
		// would seem to make sense, but it uses a 256(!) byte table.
		// Note that we ignore requests for more than BUFFER_MAX space.
		for (shift = 1; (1U << shift) < min(max_buffer_size, size); shift++);
		
		bytes = bufPtr;
		msk = (1 << shift) - 1;
		mask = msk;
	}
	
	void freeBuffer()
	{
		head = tail = 0;
		mask = 0;
	}

};

// Used by the per-port interrupt vectors
RingBuffer __FastSerial__rxBuffer[FS_MAX_PORTS];
RingBuffer __FastSerial__txBuffer[FS_MAX_PORTS];

/// Generic Rx/Tx vectors for a serial port - needs to know magic numbers
///
#define FastSerialHandler(_PORT, _RXVECTOR, _TXVECTOR, _UDR, _UCSRB, _TXBITS) \
ISR(_RXVECTOR, ISR_BLOCK)                                               \
{                                                                       \
	uint8_t c;                                                      \
	uint8_t i;                                                      \
	/* read the byte as quickly as possible */                      \
	c = _UDR;                                                       \
	/* work out where the head will go next */                      \
	i = (__FastSerial__rxBuffer[_PORT].head + 1) & __FastSerial__rxBuffer[_PORT].mask; \
	/* decide whether we have space for another byte */             \
	if (i != __FastSerial__rxBuffer[_PORT].tail) {                  \
		/* we do, move the head */                              \
		__FastSerial__rxBuffer[_PORT].bytes[__FastSerial__rxBuffer[_PORT].head] = c; \
		__FastSerial__rxBuffer[_PORT].head = i;                 \
	}                                                               \
	else															\
	{																\
		__FastSerial__rxBuffer[_PORT].overflow++;				\
	}																\
}                                                                       \
ISR(_TXVECTOR, ISR_BLOCK)                                               \
{                                                                       \
	/* if there is another character to send */                     \
	if (__FastSerial__txBuffer[_PORT].tail != __FastSerial__txBuffer[_PORT].head) { \
		_UDR = __FastSerial__txBuffer[_PORT].bytes[__FastSerial__txBuffer[_PORT].tail]; \
		/* increment the tail */                                \
		__FastSerial__txBuffer[_PORT].tail =                    \
		(__FastSerial__txBuffer[_PORT].tail + 1) & __FastSerial__txBuffer[_PORT].mask; \
		} else {                                                        \
		/* there are no more bytes to send, disable the interrupt */ \
		if (__FastSerial__txBuffer[_PORT].head == __FastSerial__txBuffer[_PORT].tail) \
		_UCSRB &= ~_TXBITS;                             \
	}                                                               \
}                                                                       \

//
// Portability; convert various older sets of defines for U(S)ART0 up
// to match the definitions for the 1280 and later devices.
//
#if !defined(USART0_RX_vect)
# if defined(USART_RX_vect)
#  define USART0_RX_vect        USART_RX_vect
#  define USART0_UDRE_vect      USART_UDRE_vect
# elif defined(UART0_RX_vect)
#  define USART0_RX_vect        UART0_RX_vect
#  define USART0_UDRE_vect      UART0_UDRE_vect
# endif
#endif

#if !defined(USART1_RX_vect)
# if defined(UART1_RX_vect)
#  define USART1_RX_vect        UART1_RX_vect
#  define USART1_UDRE_vect      UART1_UDRE_vect
# endif
#endif

#if !defined(UDR0)
# if defined(UDR)
#  define UDR0                  UDR
#  define UBRR0H                UBRRH
#  define UBRR0L                UBRRL
#  define UCSR0A                UCSRA
#  define UCSR0B                UCSRB
#  define U2X0                  U2X
#  define RXEN0                 RXEN
#  define TXEN0                 TXEN
#  define RXCIE0                RXCIE
#  define UDRIE0                UDRIE
# endif
#endif

///
/// Macro initializing a SerialPort port instance.
///
#define DefineSerialPort(_name, _num)          \
SerialPort _name(_num,						 \
&UBRR##_num##H,                                \
&UBRR##_num##L,                                \
&UCSR##_num##A,                                \
&UCSR##_num##B,                                \
U2X##_num,                                     \
(_BV(RXEN##_num) | _BV(TXEN##_num) | _BV(RXCIE##_num)), \
(_BV(UDRIE##_num)));                           \
FastSerialHandler(_num,                         \
USART##_num##_RX_vect,                        \
USART##_num##_UDRE_vect,                      \
UDR##_num,                                    \
UCSR##_num##B,                                \
_BV(UDRIE##_num))

class SerialPort : public Stream
{
public:
	SerialPort(const uint8_t portNumber, volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra, volatile uint8_t *ucsrb, const uint8_t u2x, const uint8_t portEnableBits, const uint8_t portTxBits)
	{
		_open = false;
		_ubrrh = ubrrh;
		_ubrrl = ubrrl;
		_ucsra = ucsra;
		_ucsrb = ucsrb;
		_u2x = u2x;
		_portEnableBits = portEnableBits;
		_portTxBits = portTxBits;
		_rxBuffer = &__FastSerial__rxBuffer[portNumber];
		_txBuffer = &__FastSerial__txBuffer[portNumber];
		_serialInitialized |= (1 << portNumber);
	}

	void begin(long baud, uint8_t* rxPtr, uint8_t rxSize, uint8_t* txPtr, uint8_t txSize)
	{
		uint16_t ubrr;
		bool use_u2x = true;

		// if we are currently open...
		if (_open) {
			// close the port in its current configuration, clears _open
			end();
		}

		// allocate buffers
		_rxBuffer->setBuffer(rxPtr, rxSize);
		_txBuffer->setBuffer(txPtr, txSize);

		// reset buffer pointers
		_txBuffer->head = _txBuffer->tail = 0;
		_rxBuffer->head = _rxBuffer->tail = 0;

		// mark the port as open
		_open = true;

		// If the user has supplied a new baud rate, compute the new UBRR value.
		if (baud > 0) {
			#if F_CPU == 16000000UL
			// hardcoded exception for compatibility with the bootloader shipped
			// with the Duemilanove and previous boards and the firmware on the 8U2
			// on the Uno and Mega 2560.
			if (baud == 57600)
			{
				use_u2x = false;
			}
			#endif

			if (use_u2x) {
				*_ucsra = 1 << _u2x;
				ubrr = (F_CPU / 4 / baud - 1) / 2;
				} else {
				*_ucsra = 0;
				ubrr = (F_CPU / 8 / baud - 1) / 2;
			}

			*_ubrrh = ubrr >> 8;
			*_ubrrl = ubrr;
		}

		*_ucsrb |= _portEnableBits;
	}

	void end()
	{
		*_ucsrb &= ~(_portEnableBits | _portTxBits);
		_rxBuffer->freeBuffer();
		_txBuffer->freeBuffer();
		_open = false;
	}

	virtual size_t write(uint8_t c)
	{
		uint16_t i;

		if (!_open) // drop bytes if not open
			return -1;

		// wait for room in the tx buffer
		i = (_txBuffer->head + 1) & _txBuffer->mask;
		while (i == _txBuffer->tail);

		// add byte to the buffer
		_txBuffer->bytes[_txBuffer->head] = c;
		_txBuffer->head = i;

		// enable the data-ready interrupt, as it may be off if the buffer is empty
		*_ucsrb |= _portTxBits;
		return 1;
	}
	
	uint16_t rxOverflowCounter()
	{
		if (!_open)
			return 0;
		return _rxBuffer->overflow;
	}

	int available()
	{
		if (!_open)
			return (-1);
		return ((_rxBuffer->head - _rxBuffer->tail) & _rxBuffer->mask);
	}

	uint8_t txspace()
	{
		return ((_txBuffer->mask+1) - ((_txBuffer->head - _txBuffer->tail) & _txBuffer->mask));
	}

	int read()
	{
		uint8_t c;
		// if the head and tail are equal, the buffer is empty
		if (!_open || (_rxBuffer->head == _rxBuffer->tail))
			return (-1);

		// pull character from tail
		c = _rxBuffer->bytes[_rxBuffer->tail];
		_rxBuffer->tail = (_rxBuffer->tail + 1) & _rxBuffer->mask;
		return c;
	}

	int peek()
	{
		// if the head and tail are equal, the buffer is empty
		if (!_open || (_rxBuffer->head == _rxBuffer->tail))
			return (-1);
		// pull character from tail
		return (_rxBuffer->bytes[_rxBuffer->tail]);
	}

	void flush()
	{
		// don't reverse this or there may be problems if the RX interrupt
		// occurs after reading the value of _rxBuffer->head but before writing
		// the value to _rxBuffer->tail; the previous value of head
		// may be written to tail, making it appear as if the buffer
		// don't reverse this or there may be problems if the RX interrupt
		// occurs after reading the value of head but before writing
		// the value to tail; the previous value of rx_buffer_head
		// may be written to tail, making it appear as if the buffer
		// were full, not empty.
		_rxBuffer->head = _rxBuffer->tail;

		// don't reverse this or there may be problems if the TX interrupt
		// occurs after reading the value of _txBuffer->tail but before writing
		// the value to _txBuffer->head.
		_txBuffer->tail = _txBuffer->head;
	}
	
private:
	// register accessors
	volatile uint8_t* _ubrrh;
	volatile uint8_t* _ubrrl;
	volatile uint8_t* _ucsra;
	volatile uint8_t* _ucsrb;

	// register magic numbers
	uint8_t	_u2x;
	uint8_t	_portEnableBits;	///const < rx, tx and rx interrupt enables
	uint8_t	_portTxBits;			///const < tx data and completion interrupt enables

	// ring buffers
public: // TMP TMP: should be private
	RingBuffer* _rxBuffer;
	RingBuffer* _txBuffer;
private:
	bool 			_open;

	// whether writes to the port should block waiting
	// for enough space to appear
	bool			_nonblocking_writes;

	/// Bit mask for initialized ports
	static uint8_t _serialInitialized;
};

/// Bit mask for initialized ports
uint8_t SerialPort::_serialInitialized = 0;

// Predeclarations

/*
uint8_t available(SerialPort* s);
uint8_t txspace(SerialPort* s);
uint8_t read(SerialPort* s);
void write(uint8_t);

uint8_t peek(SerialPort* s);
void flush(SerialPort* s);
uint16_t rxOverflowCounter(SerialPort* s);
*/
