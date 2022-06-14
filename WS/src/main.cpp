/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network. It's pre-configured for the Adafruit
 * Feather M0 LoRa.
 * /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
 * Please ensure they are connected.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>

#include <DFRobot_B_LUX_V30B.h>
#include <Adafruit_AM2315.h>
#include <ArduinoLowPower.h>

// Clock code

#include <ThreeWire.h>  
#include <RtcDS1302.h>

ThreeWire myWire(9,A5,A4); // IO, SCLK, CE
RtcDS1302<ThreeWire> rtc(myWire);

// Weather Station Code
byte SENSORTYPE = 0x02;

Adafruit_AM2315 am2315;
DFRobot_B_LUX_V30B sen0390(13, 12, 11);//The sensor chip is set to 13 pins, SCL and SDA adopt default configuration

volatile bool awake = false;
const int REED_PIN = 10;
int count = 0;

const int sleepTime = 30000;
int timeBeforeSleep;
volatile bool interrupted = false;

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
// #ifdef COMPILE_REGRESSION_TEST
// # define FILLMEIN 0
// #else
// # warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
// # define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
// #endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
// 6081F923ACC69CCD
static const u1_t PROGMEM APPEUI[8]= { 0xCD, 0x9C, 0xC6, 0xAC, 0x23, 0xF9, 0x81, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
// 6081F9310B07B016
static const u1_t PROGMEM DEVEUI[8]= { 0x16, 0xB0, 0x07, 0x0B, 0x31, 0xF9, 0x81, 0x60 };
// static const u1_t PROGMEM DEVEUI[8]= { 0xA3, 0x8D, 0x25, 0x91, 0x09, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
// 3D1B5699C40DFEBE7A94AE1A882943DB
static const u1_t PROGMEM APPKEY[16] = { 0x3D, 0x1B, 0x56, 0x99, 0xC4, 0x0D, 0xFE, 0xBE, 0x7A, 0x94, 0xAE, 0x1A, 0x88, 0x29, 0x43, 0xDB };
// static const u1_t PROGMEM APPKEY[16] = { 0xFF, 0xEC, 0x76, 0x2B, 0x77, 0xAB, 0xFA, 0x19, 0xE3, 0x40, 0x3D, 0x42, 0x69, 0xF2, 0xB8, 0xCC };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void do_send(osjob_t* j);
void do_send_data(osjob_t* j);

// static byte mydata[] = { 0x01, 0x01, 0x77, 0x64, 0x07, 0xD0, 0x01};
static byte mydata[] = {SENSORTYPE, 0x01, 0x77, 0x64, 0x07, 0xD0, 0x00, 0x10, 0x01};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void putToSleep(){
        timeBeforeSleep = rtc.GetDateTime().Epoch64Time();
            sleepAgain:
            int now = rtc.GetDateTime().Epoch64Time();
            LowPower.deepSleep(sleepTime - ((now - timeBeforeSleep)* 1000));
            if(interrupted == true){
                interrupted = false;
                count++;
                goto sleepAgain;
            }
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }

            putToSleep();
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send_data);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void precipitationInterupt() {
  interrupted = true;
}

void do_send_data(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

        //  Read and print temp and hum values
  float temperature = 20, humidity = 20;

//   if (! am2315.readTemperatureAndHumidity(&temperature, &humidity)) {
//     Serial.println("Failed to read data from AM2315");
//     // delay(1000);
//     return;
//   }
  int tempValue = (temperature + 273.15) * 100;
  int humidValue = humidity;
  mydata[1] = tempValue >> 8 & 0xFF;
  mydata[2] = tempValue & 0xFF;
  mydata[3] = humidValue;
  // mydata[4] = humidValue >> 8;
  Serial.print("Temp *C: "); Serial.println(temperature);
  Serial.print("Hum %: "); Serial.println(humidity);

  // Read and print light value
  Serial.print("value: ");
  // Serial.print(sen0390.lightStrengthLux());
  int lightValue = sen0390.lightStrengthLux();
  mydata[4] = lightValue >> 8 & 0xFF;
  mydata[5] = lightValue & 0xFF;
  Serial.println(" (lux).");

  delay(2000);

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}

void initialize_sensors(){
Serial.println("Initializing temp/hum sensor...");
    if (! am2315.begin()) {
     Serial.println("Sensor not found, check wiring & pullups!");
     while (1);
    }

  delay(2000);

  Serial.println("Initializing light sensor...");
  sen0390.begin();
    // begin() does a test read, so need to wait 2secs before first read
  delay(4000);
}

void initialize_clock(){
    Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();

    if (!rtc.IsDateTimeValid()) 
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("rtc lost confidence in the DateTime!");
        rtc.SetDateTime(compiled);
    }

    if (rtc.GetIsWriteProtected())
    {
        Serial.println("rtc was write protected, enabling writing now");
        rtc.SetIsWriteProtected(false);
    }

    if (!rtc.GetIsRunning())
    {
        Serial.println("rtc was not actively running, starting now");
        rtc.SetIsRunning(true);
    }

    RtcDateTime now = rtc.GetDateTime();
    Serial.println("now: ");
    printDateTime(now);
    Serial.println("compiled: ");
    printDateTime(compiled);
    // rtc.SetDateTime(compiled);
    if (now < compiled) 
    {
        Serial.println("rtc is older than compile time!  (Updating DateTime)");
        rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("rtc is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("rtc is the same as compile time! (not expected but all is fine)");
    }
}



void setup() {
    delay(5000);
    while (! Serial)
        ;
    Serial.begin(9600);
    Serial.println(F("Starting"));

    // initialize_sensors();

    pinMode(REED_PIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(REED_PIN, precipitationInterupt, FALLING);  

    initialize_clock();

    // LowPower.deepSleep(30000);    

    // Serial.println("");

    // Serial.println("NOW - BEFORE");
    // Serial.println(now - before);

    // Serial.println("SLEEPTIME - NOW - BEFORE");

    // Serial.println( sleepTime - ((now - before)* 1000));

    // Serial.println("Before: ");
    // Serial.println(before);
    // Serial.println("Now: ");
    // Serial.println(now);


    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    #if defined(ARDUINO_DISCO_L072CZ_LRWAN1)
    SPI.setMOSI(RADIO_MOSI_PORT);
    SPI.setMISO(RADIO_MISO_PORT);
    SPI.setSCLK(RADIO_SCLK_PORT);
    SPI.setSSEL(RADIO_NSS_PORT);
    #endif

    // Pin mapping
    //  We use the built-in mapping -- see src/hal/getpinmap_thisboard.cpp
    //
    // If your board isn't supported, declare an lmic_pinmap object as static
    // or global, and set pPimMap to that pointer.
    //
    const lmic_pinmap *pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();

    // don't die mysteriously; die noisily.
    if (pPinMap == nullptr) {
        pinMode(LED_BUILTIN, OUTPUT);
        for (;;) {
            // flash lights, sleep.
            for (int i = 0; i < 5; ++i) {
                digitalWrite(LED_BUILTIN, 1);
                delay(100);
                digitalWrite(LED_BUILTIN, 0);
                delay(900);
            }
            Serial.println(F("board not known to library; add pinmap or update getconfig_thisboard.cpp"));
        }
    }

    os_init_ex(pPinMap);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // allow much more clock error than the X/1000 default. See:
    // https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
    // https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
    // the X/1000 means an error rate of 0.1%; the above issue discusses using values up to 10%.
    // so, values from 10 (10% error, the most lax) to 1000 (0.1% error, the most strict) can be used.
    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 10);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);

#if CFG_LMIC_EU_like
    // This makes joins faster in the US because we don't wander all over the
    // spectrum.
    // LMIC_selectSubBand(1);
#endif

    // Start job (sending automatically starts OTAA too)
    do_send_data(&sendjob);
}

void loop() {
    os_runloop_once();
}