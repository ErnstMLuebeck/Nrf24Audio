/*  NRF24 Audio Stream
 *  
 *  Author: E. M. Luebeck
 *  Date: 2020-04-04
 *  Hardware: Teensy 4.0, NRF24L01, Audioshield
 *  
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#define AUDIO_BLOCK_SAMPLES  64
#define NUM_SAMPLES_PER_BLOCK 64
#define NUM_SAMPLES_PER_PACKET 16

#include <Audio.h> // #define AUDIO_BLOCK_SAMPLES  64
#include <TimerOne.h>

#include "AudioNrf24Tx.h"
#include "PinMonitor.h"

//#define DEBUG_SKIP_PACKET_TX

/* Button1 */
#define BUTTON1_PIN 0

#define DEBUG_PIN 17

// NRF24L01 PIN MAPPING

#define CSN_PIN 9 // new Teensy 4.0
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13
#define CE_PIN 14 // new Teensy 4.0
#define IRQ_PIN 16 // new Teensy 4.0

// AUDIO SHIELD SGTL5000, Rev D (Teensy 4.0)
#define I2S_TX_PIN 7 // I2S TX
#define I2S_RX_PIN 8 // I2S RX
#define SD_CS_PIN 10 // SD card SPI chip select
#define SD_MOSI_PIN 11 // SD card SPI MOSI
#define SD_MISO_PIN 12 // SD card SPI MISO
#define SD_SCK_PIN 13 // SD card SPI clock
#define SDA_PIN 18 // I2C SDA
#define SCL_PIN 19 // I2C SCL
#define I2S_LRCLK_PIN 20 // left/right clock
#define BLCK_PIN 21 // 1.41 MHz
#define MCLK_PIN 23 // 11.29 MHz

#define NRF_CHANNEL_MIN 106   // lower WIFI traffic at higher channels
#define NRF_CHANNEL_MAX 117 
#define NRF_NUM_RETRIES 0
#define NRF_DLY_RETRY 1 // delay*250us

#define MODE_CHNLHPNG_OFF 0
#define MODE_CHNLHPNG_LINEAR 1
#define MODE_CHNLHPNG_PRAND 2

int16_t InputBuffer[128];
int NrPacketsSent = 0;

struct packet
{
      int16_t Buffer[16];

};
packet data;

RF24 radio(CE_PIN, CSN_PIN);

AudioNrf24Tx audioTx;

// GUItool: begin automatically generated code
AudioControlSGTL5000     sgtl5000_1;     //xy=221.0057029724121,264.0056781768799
AudioInputI2S            i2s2;           //xy=221.00569534301758,412.00566005706787
AudioSynthWaveform       waveform1;      //xy=222.0056915283203,346.99999260902405
AudioMixer4              mixer1;         //xy=389.00569915771484,382.0056343078613
AudioOutputI2S           i2s1;           //xy=542.0055999755859,380.005578994751
AudioConnection          patchCord1(i2s2, 0, mixer1, 1);
AudioConnection          patchCord2(i2s2, 1, mixer1, 2);
AudioConnection          patchCord3(waveform1, 0, mixer1, 0);
AudioConnection          patchCord4(mixer1, 0, i2s1, 0);
//AudioConnection          patchCord5(mixer1, 0, i2s1, 1);
// GUItool: end automatically generated code

AudioConnection          patchCord5(mixer1, 0, audioTx, 0);

float t;
unsigned long TiNow = 0;
unsigned long TiSent = 0;
int IdxStart = 0;

boolean tx_ok = false;
byte addrPipe[][6] = {"1Ad","2Ad","3Ad"};
byte NrRxPipe = 0;
bool FlgNewDataToRead = 0;

int ModeChnlHop = MODE_CHNLHPNG_OFF;
int ChnlNrf = NRF_CHANNEL_MIN;

int StInputSelect = 0;

PinMonitor Button1 = PinMonitor(BUTTON1_PIN, 4, LOW, 1);

void ISR_NRF24();
void ISR_Timer5ms();
uint8_t getNxtChnl(uint8_t LstChnl, int _ModeChnlHop, bool Rst);
uint8_t getPrbs7(bool Rst);

void setup() 
{
    Serial.begin(115200);
    delay(100);
  
    Serial.print("Setup I/O pins..");
    pinMode(CE_PIN, OUTPUT);
    pinMode(CSN_PIN, OUTPUT);
    pinMode(IRQ_PIN, INPUT);
    digitalWrite(IRQ_PIN, LOW);

    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);

    Serial.println("OK");
    
    Serial.print("Setup NRF24L01+..");

    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCK(SCK_PIN);
    // SPI.setClockDivider(SPI_CLOCK_DIV2);  // default: DIV4

    radio.begin(); 
    radio.setAutoAck(false);
    radio.setChannel(ChnlNrf); 
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.setCRCLength(RF24_CRC_8);
    radio.disableCRC();

    attachInterrupt(IRQ_PIN, ISR_NRF24, FALLING); /* falling edge, NRF24L01 */
    radio.maskIRQ(0,1,1); /* tx_ok, tx_fail, rx_ready: Mask interrupTsLght; 0 = unmasked */

    Serial.println("OK");

    /* Init soundcard */
    Serial.print("Setup sound-card..");  
    AudioMemory(10); /* 1 block = 128 samples = 3ms */
    
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);    
    sgtl5000_1.lineOutLevel(13); // 20 = 2.14 Vpp, 13 = 3.16 Vpp

    waveform1.frequency(440);
    waveform1.amplitude(1.0);
    waveform1.begin(WAVEFORM_SINE);
    //analogReference(INTERNAL);

    mixer1.gain(0, 1.0); /* 440 Hz */
    mixer1.gain(1, 0.0); /* LineIn L */
    mixer1.gain(2, 0.0); /* LineIn R */
    mixer1.gain(3, 0.0); /* not connected */

    Serial.println("OK");

    radio.openWritingPipe(addrPipe[1]);

    //radio.openReadingPipe(1, addrPipe[1]);
    //radio.startListening();

    Serial.print("Setup interrupts..");
    Timer1.initialize(5000); /* initialize timer1, and set it to 5ms */
    Timer1.attachInterrupt(ISR_Timer5ms);  /* attaches as a timer overflow interrupt */

    Serial.println("OK");
}

void loop() 
{   
    TiNow = micros();

    /* Select input between Line in and test tone */
    if(Button1.risingEdge())
    {   if(StInputSelect == 0) StInputSelect = 1;
        else StInputSelect = 0;
        
    }
    if(StInputSelect == 0)
    {
        mixer1.gain(0, 1.0); /* 440 Hz */
        mixer1.gain(1, 0.0); /* LineIn L */
        mixer1.gain(2, 0.0); /* LineIn R */
    }
    else if(StInputSelect == 1)
    {
        mixer1.gain(0, 0.0); /* 440 Hz */
        mixer1.gain(1, 1.0); /* LineIn L */
        mixer1.gain(2, 1.0); /* LineIn R */
    }

    /* Check if buffer is filled with new samples */
    if(audioTx.available())
    {
        IdxStart = 0;
        /* Make sure the data is sent right away */
        TiSent = 0;
    }

    /* TX delay shall not be lower than 300 us, carrier starts to drift */
    if((TiNow-TiSent) >= 300)
    {   TiSent = TiNow;

        if(IdxStart < NUM_SAMPLES_PER_BLOCK)
        {
            //digitalWrite(DEBUG_PIN, HIGH);
            for(int i = 0; i < NUM_SAMPLES_PER_PACKET; i++)
            {
                data.Buffer[i] = InputBuffer[i+IdxStart];
            }
            IdxStart += NUM_SAMPLES_PER_PACKET;

            #ifdef DEBUG_SKIP_PACKET_TX

            /* Skip one of 4 packets */
            if(IdxStart != 16) tx_ok = radio.writeFast(&data, sizeof(packet));   
            
            #else

            tx_ok = radio.writeFast(&data, sizeof(packet));

            #endif
            //digitalWrite(DEBUG_PIN, LOW);
        }
        else
        {   //radio.txStandBy();  
            /* Channel Hopping */
            ChnlNrf = getNxtChnl(ChnlNrf, ModeChnlHop, 0);
            radio.setChannel(ChnlNrf);
        }
        
    }
}

void ISR_NRF24()
{   /* read data from NRF in main loop */

    noInterrupts();

    radio.available(&NrRxPipe);
    FlgNewDataToRead = 1;

    interrupts();
}

void ISR_Timer5ms()
{   
    Button1.update(); /* fall time is about 1ms */

}

uint8_t getNxtChnl(uint8_t LstChnl, int _ModeChnlHop, bool Rst)
{
    uint8_t NxtChnl=0;
    
    switch(_ModeChnlHop)
    {   // no hopping
        case 0:
            NxtChnl = NRF_CHANNEL_MIN;
            break;
            
            // linear hopping sequence
        case 1:
            if (LstChnl < NRF_CHANNEL_MAX) NxtChnl = LstChnl+1;
            else NxtChnl = NRF_CHANNEL_MIN;
            break;
            
            // pseudo random hopping
        case 2:
            NxtChnl = NRF_CHANNEL_MIN + (getPrbs7(Rst) & 0b00000111);
            break;
            
        // no hopping
        default:
            NxtChnl = NRF_CHANNEL_MIN;
            break;
    }
    return(NxtChnl);
}

// An example of generating a "PRBS-7" sequence
// https://en.wikipedia.org/wiki/Pseudorandom_binary_sequence
// ..has a repetition period of 127 bits.
uint8_t getPrbs7(bool Rst)
{
    uint8_t start = 0x02;
    static uint8_t a = start;
    
    if(Rst) a = start;
    
    int newbit = (((a >> 6) ^ (a >> 5)) & 1);
    a = ((a << 1) | newbit) & 0x7f;
    
    //Serial.println(a);
    return(a);
}