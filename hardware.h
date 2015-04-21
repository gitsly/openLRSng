#if (COMPILE_TX == 1)
#define TelemetrySerial Serial

#define USE_ICP1 // use ICP1 for PPM input for less jitter

#define PPM_IN 8 // ICP1

#define TX_AIN0 A4 // SDA
#define TX_AIN1 A5 // SCL

#define BUZZER_ACT 6
#define BUZZER_PAS 3
#define BTN 7

void buzzerInit()
{
  pinMode(BUZZER_ACT, OUTPUT);
  digitalWrite(BUZZER_ACT, LOW);
  TCCR2A = (1<<WGM21); // mode=CTC
#if (F_CPU == 16000000)
  TCCR2B = (1<<CS22) | (1<<CS20); // prescaler = 128
#elif (F_CPU == 8000000)
  TCCR2B = (1<<CS22); // prescaler = 64
#else
#errror F_CPU Invalid
#endif
  pinMode(BUZZER_PAS, OUTPUT);
  digitalWrite(BUZZER_PAS, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER_ACT,HIGH);
    uint32_t ocr = 125000L / freq;
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR2A = ocr;
    TCCR2A |= (1<<COM2B0); // enable output on buzzer2
  } else {
    digitalWrite(BUZZER_ACT,LOW);
    TCCR2A &= ~(1<<COM2B0); // disable output buzzer2
  }
}

#define buzzerOff(foo) buzzerOn(0)
#else // RX
#define PPM_OUT 9 // OCP1A
#define RSSI_OUT 3 // PD3 OC2B

#define OUTPUTS 13 // outputs available

const pinMask_t OUTPUT_MASKS[OUTPUTS] = {
  {0x00,0x00,0x08},{0x00,0x00,0x20},{0x00,0x00,0x40}, // RSSI, CH1, CH2
  {0x00,0x00,0x80},{0x01,0x00,0x00},{0x02,0x00,0x00}, // CH2, CH3, CH4
  {0x04,0x00,0x00},{0x08,0x00,0x00},{0x10,0x00,0x00}, // CH5, CH6, CH7
  {0x00,0x10,0x00},{0x00,0x20,0x00},{0x00,0x00,0x01}, // SDA, SCL, RXD
  {0x00,0x00,0x02},                                   // TXD
};

const uint8_t OUTPUT_PIN[OUTPUTS] = { 3, 5, 6, 7, 8, 9, 10, 11, 12 , A4, A5, 0, 1};

#define PPM_OUTPUT  5
#define RSSI_OUTPUT 0
#define LLIND_OUTPUT 8
#define ANALOG0_OUTPUT 9
#define ANALOG1_OUTPUT 10
#define SDA_OUTPUT 9
#define SCL_OUTPUT 10
#define RXD_OUTPUT 11
#define TXD_OUTPUT 12

struct rxSpecialPinMap rxSpecialPins[] = {
  {  0, PINMAP_RSSI},
  {  0, PINMAP_LBEEP},
  {  5, PINMAP_PPM},
  {  8, PINMAP_LLIND},
  {  9, PINMAP_SDA},
  {  9, PINMAP_ANALOG}, // AIN0
  { 10, PINMAP_SCL},
  { 10, PINMAP_ANALOG}, // AIN1
  { 11, PINMAP_RXD},
  { 12, PINMAP_TXD},
  { 12, PINMAP_SPKTRM},
  { 12, PINMAP_SBUS},
  { 12, PINMAP_SUMD},
};

#endif

#define Red_LED    A3
#define Green_LED  13

#if (COMPILE_TX != 1)
#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);
#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);
#else
#define Red_LED2   9
#define Green_LED2 10
#define Red_LED_ON  { PORTC |= _BV(3); PORTB |= _BV(1); }
#define Red_LED_OFF { PORTC &= ~_BV(3); PORTB &= ~_BV(1); }
#define Green_LED_ON  PORTB |= (_BV(5) | _BV(2));
#define Green_LED_OFF PORTB &= ~(_BV(5) | _BV(2));
#endif

//## RFM22B Pinouts for Public Edition (Rx v2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2

void setupSPI()
{
  pinMode(SDO_pin, INPUT);   //SDO
  pinMode(SDI_pin, OUTPUT);   //SDI
  pinMode(SCLK_pin, OUTPUT);   //SCLK
  pinMode(IRQ_pin, INPUT);   //IRQ
  pinMode(nSel_pin, OUTPUT);   //nSEL
}

#define IRQ_interrupt 0
void setupRfmInterrupt()
{
  attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

#endif
