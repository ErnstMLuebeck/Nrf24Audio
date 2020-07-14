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
#include <FastLED.h>

#define AUDIO_BLOCK_SAMPLES  64
#define NUM_SAMPLES_PER_BLOCK 64
#define NUM_SAMPLES_PER_PACKET 16

#include <Audio.h> 
// Update: /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy4
// #define AUDIO_BLOCK_SAMPLES  64
//#include <TimerOne.h>

#include "AudioNrf24Tx.h"
#include "PinMonitor.h"
#include "SignalMonitor.h"
#include "effect_remove_vocals.h"

//#define DEBUG_SKIP_PACKET_TX

/* Button1 */
// LED strip WS2812B
#define DATA1_PIN 2  
#define NUM_LEDS 2

#define BUTTON1_PIN 10

#define DMX_REDE 15//2
#define DMX_RX_PIN 0 // not used
#define DMX_TX_PIN 1 

// Footswitches
#define FS2_PIN 4
#define FS1_PIN 5

/* Standby Toggle */
#define STANDBY_TOGGLE_PIN 10 // = SD CS, new Teensy 4.0

// NRF24L01 MODULE A PIN MAPPING
#define CSN_PIN 9 // new Teensy 4.0
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13
#define CE_PIN 14 // new Teensy 4.0
#define IRQ_PIN 16 // new Teensy 4.0

// NRF24L01 MODULE B PIN MAPPING
#define CSN_B_PIN 3 // new Teensy 4.0
#define CE_B_PIN 0 // new Teensy 4.0
#define IRQ_B_PIN 17 // new Teensy 4.0

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

#define NRF_CHANNEL_MIN 96   // lower (almost no) WIFI traffic at higher channels
#define NRF_CHANNEL_MAX 117
#define NRF_NUM_RETRIES 0
#define NRF_DLY_RETRY 1 // delay*250us

#define DBNC_MAX 8000 // button debounce counter maximum

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

CRGB leds[NUM_LEDS];

//RF24 radioA(CE_PIN, CSN_PIN);

// create NRF object
RF24 radioA = RF24(CE_PIN, CSN_PIN);
//RF24 radioB = RF24(CE_B_PIN, CSN_B_PIN);

AudioNrf24Tx audioTx;

// GUItool: begin automatically generated code
AudioControlSGTL5000     audioShield;     //xy=221.0057029724121,264.0056781768799
AudioInputI2S            i2s1;           //xy=221.00569534301758,412.00566005706787
AudioSynthWaveform       waveform1;      //xy=222.0056915283203,346.99999260902405
AudioMixer4              mixer1;         //xy=389.00569915771484,382.0056343078613
AudioEffectRemoveVocals  combineSymmetric;
AudioEffectRemoveVocals  combineMono; 

AudioConnection          patchCord1(waveform1, 0, mixer1, 0);

AudioConnection          patchCord2(i2s1, 0, combineMono, 0);
AudioConnection          patchCord3(i2s1, 1, combineMono, 1);
AudioConnection          patchCord4(combineMono, 0, mixer1, 1);

AudioConnection          patchCord5(i2s1, 0, combineSymmetric, 0);
AudioConnection          patchCord6(i2s1, 1, combineSymmetric, 1);
AudioConnection          patchCord7(combineSymmetric, 0, mixer1, 2);

AudioConnection          patchCord8(mixer1, 0, audioTx, 0);
// GUItool: end automatically generated code



float t;
unsigned long TiNow = 0;
unsigned long TiSent = 0;
int IdxStart = 0;

boolean tx_ok = false;
byte addrPipe[][6] = {"1Ad","2Ad","3Ad"};
byte NrRxPipe = 0;
bool FlgNewDataToRead = 0;

bool FlgIsClippingRaw = 0;
bool FlgIsClippingDeb = 0; /* debounced clipping flag */

int ModeChnlHop = MODE_CHNLHPNG_OFF;
int ChnlNrf = NRF_CHANNEL_MIN;

int StInputSelect = 1;

PinMonitor Button1 = PinMonitor(BUTTON1_PIN, DBNC_MAX, LOW, 1);

SignalMonitor StInputSelectMonitor = SignalMonitor(StInputSelect);
SignalMonitor FlgIsClippingMonitor = SignalMonitor(FlgIsClippingRaw);

void ISR_NRF24();
//void ISR_Timer5ms();
uint8_t getNxtChnl(uint8_t LstChnl, int _ModeChnlHop, bool Rst);
uint8_t getPrbs7(bool Rst);

void setup() 
{
    Serial.begin(115200);
    delay(100);
  
    Serial.print("Setup I/O pins..");

    Serial.println("OK");

    /* Init LED strips pins */
    Serial.print("Setup RGB LED..");
    FastLED.addLeds<WS2812, DATA1_PIN, GRB>(leds, NUM_LEDS);

    //FastLED.setMaxPowerInVoltsAndMilliamps(5, 5000); 
    FastLED.setBrightness(255);

    leds[0] = CRGB(0, 255, 255);
    FastLED.show();

    Serial.println("OK");
    
    Serial.print("Setup NRF24L01+..");

    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCK(SCK_PIN);
    // SPI.setClockDivider(SPI_CLOCK_DIV2);  // default: DIV4

    radioA.begin(); 
    radioA.setAutoAck(false);
    radioA.setChannel(ChnlNrf); 
    radioA.setPALevel(RF24_PA_LOW); // RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radioA.setDataRate(RF24_2MBPS);
    radioA.setCRCLength(RF24_CRC_8);
    radioA.disableCRC();

    attachInterrupt(IRQ_PIN, ISR_NRF24, FALLING); /* falling edge, NRF24L01 */
    radioA.maskIRQ(0,1,1); /* tx_ok, tx_fail, rx_ready: Mask interrupTsLght; 0 = unmasked */

    Serial.println("OK");

    /* Init soundcard */
    Serial.print("Setup sound-card..");  
    AudioMemory(10); /* 1 block = 128 samples = 3ms */
    
    audioShield.enable();
    audioShield.volume(0.5);    
    audioShield.lineOutLevel(13); // 20 = 2.14 Vpp, 13 = 3.16 Vpp

    waveform1.frequency(440);
    waveform1.amplitude(1.0);
    waveform1.begin(WAVEFORM_SINE);
    //analogReference(INTERNAL);

    mixer1.gain(0, 0.0); /* 440 Hz */
    mixer1.gain(1, 1.0); /* LineIn L + R */
    mixer1.gain(2, 0.0); /* LineIn L - R */
    mixer1.gain(3, 0.0); /* not connected */

    combineMono.enable(0); /* 0 = L + R, 1 = L - R */ 
    combineSymmetric.enable(1); /* 0 = L + R, 1 = L - R */

    Serial.println("OK");

    radioA.openWritingPipe(addrPipe[1]);

    //radioA.openReadingPipe(1, addrPipe[1]);
    //radioA.startListening();

    /* !!! Timer1 interrupts cause audio distortion !!! */
    // Serial.print("Setup interrupts..");
    // Timer1.initialize(5000); /* initialize timer1, and set it to 5ms */
    // Timer1.attachInterrupt(ISR_Timer5ms);  /* attaches as a timer overflow interrupt */

    // Serial.println("OK");

    leds[0] = CRGB(255, 0, 255);
    FastLED.show();
}

void loop() 
{   
    TiNow = micros();

    Button1.update(); /* fall time is about 1ms */

    /* Select input between Line in and test tone */
    if(Button1.risingEdge())
    {   if(StInputSelect == 0) StInputSelect = 1;
        else StInputSelect = 0;
        
    }

    if(StInputSelectMonitor.detectChange(StInputSelect))
    {
        if(StInputSelect == 0)
        {
            mixer1.gain(0, 0.0); /* 440 Hz */
            mixer1.gain(1, 0.0); /* LineIn L + R */
            mixer1.gain(2, 1.0); /* LineIn L - R */

            leds[0] = CRGB(255, 255, 0);
            FastLED.show();
        }
        else if(StInputSelect == 1)
        {
            mixer1.gain(0, 0.0); /* 440 Hz */
            mixer1.gain(1, 1.0); /* LineIn L + R */
            mixer1.gain(2, 0.0); /* LineIn L - R */

            leds[0] = CRGB(255, 0, 255);
            FastLED.show();
        }
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
            if(IdxStart != 16) tx_ok = radioA.writeFast(&data, sizeof(packet));   
            
            #else

            tx_ok = radioA.writeFast(&data, sizeof(packet));

            #endif
            //digitalWrite(DEBUG_PIN, LOW);
        }
        else
        {   //radioA.txStandBy();  
            /* Channel Hopping */
            ChnlNrf = getNxtChnl(ChnlNrf, ModeChnlHop, 0);
            radioA.setChannel(ChnlNrf);
        }
        
    }

    if(audioTx.isClipping())
    {
        FlgIsClippingRaw = 1;
    }
    else
    {
        FlgIsClippingRaw = 0;
    }

    if(FlgIsClippingMonitor.detectChange(FlgIsClippingRaw))
    {
        if(FlgIsClippingRaw)
        {
            leds[0] = CRGB(0, 255, 255);
            FastLED.show();
        }
        else
        {
            if(StInputSelect == 0)
            {
                leds[0] = CRGB(255, 255, 0);
                FastLED.show();
            }
            else if(StInputSelect == 1)
            {
                leds[0] = CRGB(255, 0, 255);
                FastLED.show();
            }
        }

    }

            
    
}

void ISR_NRF24()
{   /* read data from NRF in main loop */

    noInterrupts();

    radioA.available(&NrRxPipe);
    FlgNewDataToRead = 1;

    interrupts();
}

// void ISR_Timer5ms()
// {   
//     Button1.update(); /* fall time is about 1ms */

// }

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
