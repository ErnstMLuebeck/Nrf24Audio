/*  NRF24 Audio Stream
 *  
 *  Author: E. M. Luebeck
 *  Date: 2020-04-04
 *  Hardware: Teensy 4.0, NRF24L01, Audioshield
 *  
 */

#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <FastLED.h>

#define AUDIO_BLOCK_SAMPLES  64
#define NUM_SAMPLES_PER_BLOCK 64
#define NUM_SAMPLES_PER_PACKET 16
#define NUM_PACKETS_PER_BLOCK 4

#include <TimerOne.h>
#include <Audio.h> 
// AudioStream.h, Update: #define AUDIO_BLOCK_SAMPLES  64

#include "AudioNrf24Rx.h"
#include "PinMonitor.h"
#include "LowPassFilter.h"
#include "SignalMonitor.h"

//#define DEBUG_RATATOB

/* Mute Button */
#define MUTE_BUTTON_PIN 10

/* Button1 */
// LED strip WS2812B
#define DATA1_PIN 2  
#define NUM_LEDS 2

/* Volume poti, analog in */
#define VOL_POT_PIN 22

// 12bit DAC PIN
#define DAC_PIN A21

// RF LED
//#define RF_LED_PIN 0 // removed in HW V2

// Debug PIN
//#define DEBUG_PIN 16
//#define DEBUG2_PIN 17

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

#define DBNC_MAX 8000

#define MODE_CHNLHPNG_OFF 0
#define MODE_CHNLHPNG_LINEAR 1
#define MODE_CHNLHPNG_PRAND 2

#define MAX_SOUND_LEVEL 0.5
#define TS_VOLPOT_MS 50 /* sample time, [ms] */

#define NUM_PINGPONG 2

AudioNrf24Rx audioRx;

// GUItool: begin automatically generated code
AudioControlSGTL5000     audioShield;
AudioSynthWaveform       waveform1;      //xy=221.0056915283203,360.99999237060547
AudioMixer4              mixer1;         //xy=389.00569915771484,382.0056343078613
AudioOutputI2S           i2s1;           //xy=551.0056495666504,405.00566482543945
AudioConnection          patchCord1(waveform1, 0, mixer1, 0);
AudioConnection          patchCord2(audioRx, 0, mixer1, 1);
AudioConnection          patchCord5(mixer1, 0, i2s1, 0);
AudioConnection          patchCord6(mixer1, 0, i2s1, 1);
// GUItool: end automatically generated code

// create NRF object
RF24 radioA = RF24(CE_PIN, CSN_PIN);
RF24 radioB = RF24(CE_B_PIN, CSN_B_PIN);

byte NrRxPipe = 0;
byte addrPipe[][6] = {"1Ad","2Ad","3Ad"};
volatile bool FlgNewDataToReadA = 0;
volatile bool FlgNewDataToReadB = 0;

bool FlgNewDataToProcA = 0;
bool FlgNewDataToProcB = 0;

bool FlgTransmitterDetected = 0;

struct packet
{
      int16_t Buffer[16];

};
packet dataA, dataB;

CRGB leds[NUM_LEDS];

volatile unsigned long TiRx = 0;
unsigned long TiNow = 0;

unsigned long TiRxPacket_k = 0;
unsigned long TiRxPacket_kn1 = 0;
unsigned long TdRxPacket = 0;

unsigned long TiRxBuffer_k = 0;
unsigned long TiRxBuffer_kn1 = 0;
unsigned long TdRxBuffer = 0;

unsigned long TdDebugPrint = 0;

unsigned long TiUpdateBuffer_k = 0;
unsigned long TiUpdateBuffer_kn1 = 0;
unsigned long TdUpdateBuffer = 0;

unsigned long TiVolPotUpdate = 0;

unsigned long TiPacketToBuffer = 0;

float FacSampleRates = 1.0f;
float FacSampleRatesSum = 1.0f;
float FacSampleRatesMean = 1.0f;

unsigned long TiUpdate = 0;
int IdxBuffer = 0;

unsigned long NumRxBuffersTot = 1;
unsigned long NumPacketsTot = 0;
unsigned long NumPacketsLost = 0;

unsigned long CntrPacketsA = 1;
unsigned long CntrPacketsB = 1;
float RatAToB = 1;

int IdxPingPongWrite = 0;
int IdxPingPongRead = 2;

bool FlgPingPongViolation = 0;
bool NumAutoPacketIncr = 0;
bool FlgAutoFreqHop = 0;

bool StMuteOutput = 1;
bool StRfPresent = 0;

SignalMonitor StMuteMonitor = SignalMonitor(StMuteOutput);
SignalMonitor StRfPresentMonitor = SignalMonitor(StRfPresent);

int16_t OutputBuffer[NUM_PINGPONG][NUM_SAMPLES_PER_BLOCK];
int16_t FadeOut[NUM_SAMPLES_PER_BLOCK];
int IdxStart = 0;
int IdxPacket = 0;
int VldPacket[2][4] = {0};

int ModeChnlHop = MODE_CHNLHPNG_OFF;
int ChnlNrf = NRF_CHANNEL_MIN;

int ValVolPotRaw = 0;
float ValVolPot = 0.0f;
float ValVolPot_kn1 = 0.0f;
float ValVolPotDiff = 0.05f;
float LevelOutput = 0.0f;

PinMonitor MuteButton = PinMonitor(MUTE_BUTTON_PIN, DBNC_MAX, LOW, 1);
LowPassFilter FilterVolPot = LowPassFilter((float)TS_VOLPOT_MS/1000.0f, 0.08, 0.0);

void ISR_NRF24_A();
uint8_t getNxtChnl(uint8_t LstChnl, int _ModeChnlHop, bool Rst);
uint8_t getPrbs7(bool Rst);

void setup() 
{
    Serial.begin(115200);
    delay(100);

    /* Init LED strips pins */
    Serial.print("Setup RGB LED..");
    FastLED.addLeds<WS2812, DATA1_PIN, GRB>(leds, NUM_LEDS);
    //FastLED.addLeds<1, WS2812, DATA1_PIN, GRB>(leds, NUM_LEDS);

    //FastLED.setMaxPowerInVoltsAndMilliamps(5, 5000); 
    FastLED.setBrightness(255);

    leds[0] = CRGB(255-100, 255-0, 255-0);
    FastLED.show();

    Serial.println("OK");

    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCK(SCK_PIN);
    SPI.setClockDivider(SPI_CLOCK_DIV4); // does not change anything

    pinMode(VOL_POT_PIN, INPUT);

    /* Init A NRF24L01+ RF module */
    Serial.print("Setup A NRF24L01+..");

    radioA.begin(); 
    radioA.setAutoAck(false);
    radioA.setChannel(NRF_CHANNEL_MIN); 
    radioA.setPALevel(RF24_PA_MAX);
    radioA.setDataRate(RF24_2MBPS); 
    radioA.setRetries(NRF_DLY_RETRY, NRF_NUM_RETRIES);
    radioA.setCRCLength(RF24_CRC_8);
    radioA.disableCRC();

    Serial.println("OK");
    //radioA.printDetails();

    /* Init B NRF24L01+ RF module */
    Serial.print("Setup B NRF24L01+..");

    radioB.begin(); 
    radioB.setAutoAck(false);
    radioB.setChannel(NRF_CHANNEL_MIN); 
    radioB.setPALevel(RF24_PA_MAX);
    radioB.setDataRate(RF24_2MBPS); 
    radioB.setRetries(NRF_DLY_RETRY, NRF_NUM_RETRIES);
    radioB.setCRCLength(RF24_CRC_8);
    radioB.disableCRC();

    Serial.println("OK");
    //radioB.printDetails();

    /* Init soundcard */
    Serial.print("Setup sound-card..");  
    AudioMemory(10); /* 1 block = 128 samples = 3ms */

    waveform1.frequency(440);
    waveform1.amplitude(1.0);
    waveform1.begin(WAVEFORM_TRIANGLE);
    //analogReference(EXTERNAL);

    mixer1.gain(0, 0.0); /* triangle 440 Hz */
    mixer1.gain(1, 0.0); /* NRF24 RX L */
    mixer1.gain(2, 0.0); /* not connected */
    mixer1.gain(3, 0.0); /* not connected */

    /* headphone level (unused) */
    float level_dB = 0;
    float level = pow(10,level_dB/20);
    if(level >= 1.0) level = 1.0;
    if(level <= 0.0) level = 0.0;

    audioShield.enable();
    audioShield.volume(level);    
    audioShield.lineOutLevel(13); // 20 = 2.14 Vpp, 13 = 3.16 Vpp

    Serial.println("OK");

    for(int i = 0; i < NUM_SAMPLES_PER_BLOCK; i++)
    {
        OutputBuffer[0][i] = 0;
        OutputBuffer[1][i] = 0;
        FadeOut[i] = 1.0 - (float)i * (1/(float)(NUM_SAMPLES_PER_BLOCK+1));
    }

    /* Activate NRF24 A */
    radioA.maskIRQ(1,1,0); /* tx_ok, tx_fail, rx_ready: Mask interrupTsLght; 0 = unmasked */
    attachInterrupt(IRQ_PIN, ISR_NRF24_A, FALLING); /* falling edge, NRF24L01 */
    radioA.openReadingPipe(1, addrPipe[1]);
    radioA.startListening();

    /* Activate NRF24 B */
    radioB.maskIRQ(1,1,0); /* tx_ok, tx_fail, rx_ready: Mask interrupTsLght; 0 = unmasked */
    attachInterrupt(IRQ_B_PIN, ISR_NRF24_B, FALLING); /* falling edge, NRF24L01 */
    radioB.openReadingPipe(1, addrPipe[1]);
    radioB.startListening();

    /* !!! Timer1 interrupts cause audio distortion !!! */
    // Serial.print("Setup interrupts..");
    // Timer1.initialize(5000); /* initialize timer1, and set it to 5ms */
    // Timer1.attachInterrupt(ISR_Timer5ms);  /* attaches as a timer overflow interrupt */

    // Serial.println("OK");
}


void loop() 
{   TiNow = micros();

    MuteButton.update();

    /* Volume poti handling */
    if((TiNow-TiVolPotUpdate) >= (TS_VOLPOT_MS * 1000))
    {   TiVolPotUpdate = TiNow;

        ValVolPotRaw = analogRead(VOL_POT_PIN); /* 0..1024 */

        ValVolPot = FilterVolPot.calculate((float)ValVolPotRaw / 1024.0);

        if(ValVolPot >= (ValVolPot_kn1 + ValVolPotDiff)||
        ValVolPot <= (ValVolPot_kn1 - ValVolPotDiff))
        {
            ValVolPot_kn1 = ValVolPot;

            float level_dB = -65.0 + 65.0 * ValVolPot;
            LevelOutput = pow(10, level_dB/20);
            if(LevelOutput >= 1.0) LevelOutput = 1.0;
            if(LevelOutput <= 0.0) LevelOutput = 0.0;

            if(StMuteOutput == 0)
            {
                mixer1.gain(1, LevelOutput*MAX_SOUND_LEVEL); /* NRF24 RX L */
            }
        }
    }

    /* Toggle output mute with push button */
    if(MuteButton.risingEdge())
    {
        if(StMuteOutput == 1) StMuteOutput = 0;
        else StMuteOutput = 1;
    }

    if(StMuteMonitor.detectChange(StMuteOutput) || StRfPresentMonitor.detectChange(StRfPresent))
    {
        if(StMuteOutput == 1)
        {
            mixer1.gain(1, 0.0); /* NRF24 RX L */

            if(StRfPresent == 1)
            {   /* Mute but RF available */
                leds[0] = CRGB(255-0, 255-0, 255-100);
            }
            else
            {   /* Mute because no RF available */
                leds[0] = CRGB(255-100, 255-0, 255-0);
            }
            
            FastLED.show();
        }
        if(StMuteOutput == 0)
        {
            mixer1.gain(1, LevelOutput*MAX_SOUND_LEVEL); /* NRF24 RX L */
            leds[0] = CRGB(255-0, 255-100, 255-0);
            FastLED.show();
        }
    }

    if(FlgNewDataToReadA)
    {   FlgNewDataToReadA = 0;

        /*----------------------------------------------------*/
        radioA.read(&dataA, sizeof(packet));
        /*----------------------------------------------------*/
        FlgNewDataToProcA = 1;
        
    }   

    if(FlgNewDataToReadB)
    {   FlgNewDataToReadB = 0;

        /*----------------------------------------------------*/
        radioB.read(&dataB, sizeof(packet));
        /*----------------------------------------------------*/
        FlgNewDataToProcB = 1;
    }    

    if(FlgNewDataToProcA || FlgNewDataToProcB)
    {   
        NumAutoPacketIncr = 0;

        /* Packet has already been written */
        if((TiNow-TiRxPacket_k) < 200)
        {
            FlgNewDataToProcA = 0;
            FlgNewDataToProcB = 0;
        }
        else
        {
            /* Time delay between packets */
            TiRxPacket_kn1 = TiRxPacket_k;
            TiRxPacket_k = TiNow;
            TdRxPacket = TiRxPacket_k - TiRxPacket_kn1;

            NumPacketsTot++;
        }

        /* Write packet into buffer, module A has priority */
        if(FlgNewDataToProcA)
        {   //digitalWrite(DEBUG_PIN, HIGH);
            
            for(int i = 0; i < NUM_SAMPLES_PER_PACKET; i++)
            {              
                /* use ping pong buffering to reduce effect of not synchronized clocks */
                OutputBuffer[IdxPingPongWrite][i+IdxStart] = dataA.Buffer[i];
            }
            IdxStart += NUM_SAMPLES_PER_PACKET;
            FlgNewDataToProcA = 0;
            CntrPacketsA++;
        }
        else if(FlgNewDataToProcB)
        {   //digitalWrite(DEBUG_PIN, LOW);
            
            for(int i = 0; i < NUM_SAMPLES_PER_PACKET; i++)
            {              
                /* use ping pong buffering to reduce effect of not synchronized clocks */
                OutputBuffer[IdxPingPongWrite][i+IdxStart] = dataB.Buffer[i];
            }
            IdxStart += NUM_SAMPLES_PER_PACKET;
            FlgNewDataToProcB = 0;
            CntrPacketsB++;
        }

        RatAToB = (float)CntrPacketsA / (float)CntrPacketsB;

        /* Buffer is filled with packets and ready to play */
        if(IdxStart >= NUM_SAMPLES_PER_BLOCK) 
        {
            /* Buffer ready */
            IdxStart = 0;
            NumRxBuffersTot++;

            /* Todo: Buffer post processing, interpolation of lost packets */

            /* Ping-pong buffer read/write indices */
            if(IdxPingPongWrite < NUM_PINGPONG-1) IdxPingPongWrite++;
            else IdxPingPongWrite = 0;
            if(IdxPingPongRead < NUM_PINGPONG-1) IdxPingPongRead++;
            else IdxPingPongRead = 0;

            /* Channel Hopping */
            if(ModeChnlHop != MODE_CHNLHPNG_OFF)
            {
                ChnlNrf = getNxtChnl(ChnlNrf, ModeChnlHop, 0);
                radioA.setChannel(ChnlNrf);
                radioB.setChannel(ChnlNrf);
            }

            /* Measure sample rate of transmitter and receiver */
            if(NumRxBuffersTot >= (44100/128*60))
            {
                NumRxBuffersTot = 1;
                FacSampleRatesSum = 0;
            }

            /* Time delay between complete buffers */
            TiRxBuffer_kn1 = TiRxBuffer_k;
            TiRxBuffer_k = micros();
            TdRxBuffer = TiRxBuffer_k - TiRxBuffer_kn1;

            /* Sample rate factor mean value calculation */
            FacSampleRates = (float)TdUpdateBuffer/(float)TdRxBuffer;
            FacSampleRatesSum += FacSampleRates;
            FacSampleRatesMean = FacSampleRatesSum / NumRxBuffersTot;

            //Serial.println(AudioMemoryUsageMax()); 
        }

        
    }

    /* Auto-increase start index if one packet is lost */
    unsigned long TdAutoPacketIncr = 0;
    if(NumAutoPacketIncr == 0) TdAutoPacketIncr = 350;
    else TdAutoPacketIncr = 300;

    TiNow = micros();
    if((TiNow-TiRxPacket_k) >= TdAutoPacketIncr)
    {   
        if(IdxStart == 0)
        {   /* End of buffer or packet 1 lost */
            if((TiNow-TiRxPacket_k) > 600);
                //digitalWrite(DEBUG_PIN, HIGH);
        } 
        else
        {   /* Lost packet */
            NumPacketsLost++;
            //digitalWrite(DEBUG_PIN, HIGH);

            if(IdxStart < (NUM_SAMPLES_PER_BLOCK - NUM_SAMPLES_PER_PACKET))
            {
                IdxStart += NUM_SAMPLES_PER_PACKET;
                NumAutoPacketIncr++;
            }

            /* Time delay between packets */
            TiRxPacket_kn1 = TiRxPacket_k;
            TiRxPacket_k = TiNow;
            TdRxPacket = TiRxPacket_k - TiRxPacket_kn1;

            /* Calculate packet error rate PER */
            //Serial.println(NumPacketsLost);
        }
        
    }

    /* Mute output if connection is lost */
    if((micros()-TiRxBuffer_k) >= 200000)
    {
        StMuteOutput = 1;
        StRfPresent = 0;
        
        if(ModeChnlHop != MODE_CHNLHPNG_OFF)
        {
            ChnlNrf = getNxtChnl(ChnlNrf, ModeChnlHop, 1);
            radioA.setChannel(ChnlNrf);
        }
    }   
    else
    {
        StRfPresent = 1;
    }

    #ifdef DEBUG_RATATOB

    if((micros()-TdDebugPrint) >= 100000)
    {
        TdDebugPrint = micros();
        Serial.println(RatAToB,6);
    }

    #endif
     

}

void ISR_NRF24_A()
{   /* read dataA from NRF in main loop */
    //radioA.available(&NrRxPipe);
    FlgNewDataToReadA = 1;
}

void ISR_NRF24_B()
{   /* read dataA from NRF in main loop */
    //radioB.available(&NrRxPipe);
    FlgNewDataToReadB = 1;
}

void ISR_Timer5ms()
{   
    MuteButton.update(); /* fall time is about 1ms */

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