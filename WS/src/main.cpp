#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>

// #include <DFRobot_B_LUX_V30B.h>
#include <Adafruit_AM2315.h>
#include <ArduinoLowPower.h>

#include <ThreeWire.h>  
#include <RtcDS1302.h>

// uncomment for DEBUG mode
#define DEBUG 

ThreeWire myWire(A3,A5,A4); // IO, SCLK, CE
RtcDS1302<ThreeWire> rtc(myWire);

// Weather Station Code
byte SENSORTYPE = 0x02;

Adafruit_AM2315 am2315; // Connect to default SCL SDA
// DFRobot_B_LUX_V30B sen0390(13, 12, 11); // CE, SCL, SDA // Light sensor disabled

volatile bool awake = false;
const int RAIN_SENSOR_PIN = 10;
int count = 0;

const int sleepTime = 3600000; // In milliseconds
int timeBeforeSleep;
volatile bool interrupted = false;

static const u1_t PROGMEM APPEUI[8]= { 0xCD, 0x9C, 0xC6, 0xAC, 0x23, 0xF9, 0x81, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]= { 0x16, 0xB0, 0x07, 0x0B, 0x31, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x3D, 0x1B, 0x56, 0x99, 0xC4, 0x0D, 0xFE, 0xBE, 0x7A, 0x94, 0xAE, 0x1A, 0x88, 0x29, 0x43, 0xDB };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void do_send_data(osjob_t* j);

// { 0xST, 0xTM, 0xTM, 0xHM, 0xBR, 0xBR, 0xPR, 0xPR, 0xEB }
// ST = Sensor Type
// TM = temperature
// HM = Humidity
// BR = Brightness
// PR = Precipitation
// EB = End Byte - Required for Techtenna connection
static byte mydata[] = {SENSORTYPE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
static osjob_t sendjob;

const unsigned TX_INTERVAL = 1; // In seconds

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
            digitalWrite(LED_BUILTIN, LOW);
            LowPower.deepSleep(sleepTime - ((now - timeBeforeSleep)* 1000));
            digitalWrite(LED_BUILTIN, HIGH);
            if(interrupted == true){
                interrupted = false;
                count++;
                goto sleepAgain;
            }
}

void onEvent (ev_t ev) {
    #ifdef DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            #ifdef DEBUG
            Serial.println(F("EV_SCAN_TIMEOUT"));
            #endif
            break;
        case EV_BEACON_FOUND:
            #ifdef DEBUG
            Serial.println(F("EV_BEACON_FOUND"));
            #endif
            break;
        case EV_BEACON_MISSED:
            #ifdef DEBUG
            Serial.println(F("EV_BEACON_MISSED"));
            #endif
            break;
        case EV_BEACON_TRACKED:
            #ifdef DEBUG
            Serial.println(F("EV_BEACON_TRACKED"));
            #endif
            break;
        case EV_JOINING:
            #ifdef DEBUG
            Serial.println(F("EV_JOINING"));
            #endif
            break;
        case EV_JOINED:
            #ifdef DEBUG
            Serial.println(F("EV_JOINED"));
            #endif
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              #ifdef DEBUG
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
              #endif
            }
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            #ifdef DEBUG
            Serial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
            #ifdef DEBUG
            Serial.println(F("EV_REJOIN_FAILED"));
            #endif
            break;
        case EV_TXCOMPLETE:
            #ifdef DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            #endif
            // Reset rain sensor counter
            count = 0;
            putToSleep();
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send_data);
            break;
        case EV_LOST_TSYNC:
            #ifdef DEBUG
            Serial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
            #ifdef DEBUG
            Serial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            #ifdef DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            #ifdef DEBUG
            Serial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
            #ifdef DEBUG
            Serial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
        case EV_TXSTART:
            #ifdef DEBUG
            Serial.println(F("EV_TXSTART"));
            #endif
            break;
        case EV_TXCANCELED:
            #ifdef DEBUG
            Serial.println(F("EV_TXCANCELED"));
            #endif
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            #ifdef DEBUG
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            #endif
            break;
        default:
            #ifdef DEBUG
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            #endif
            break;
    }
}

void precipitationInterupt() {
  interrupted = true;
}

void do_send_data(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {   
        #ifdef DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        //  Read and print temp and hum values
        float temperature = 20, humidity = 20;

        if (! am2315.readTemperatureAndHumidity(&temperature, &humidity)) {
            #ifdef DEBUG
            Serial.println("Failed to read data from AM2315");
            #endif
            return;
        }

        int tempValue = (temperature + 273.15) * 100;
        int humidValue = humidity;
        mydata[1] = tempValue >> 8 & 0xFF;
        mydata[2] = tempValue & 0xFF;
        mydata[3] = humidValue;
        
        #ifdef DEBUG
        Serial.print("Temp *C: "); Serial.println(temperature);
        Serial.print("Hum %: "); Serial.println(humidity);
        #endif

        // Read light value
        // int lightValue = sen0390.lightStrengthLux(); // Light sensor disabled

        // #ifdef DEBUG
        // Serial.print("value: ");
        // Serial.print(lightValue);
        // Serial.print(" (lux) ");
        // #endif
        
        // mydata[4] = lightValue >> 8 & 0xFF;
        // mydata[5] = lightValue & 0xFF;
        
        #ifdef DEBUG
        Serial.print("Precipitation count: ");
        Serial.println(count);
        #endif

        mydata[6] = count >> 8 & 0xFF;
        mydata[7] = count & 0xFF;

        delay(2000);

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        
        #ifdef DEBUG
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void initialize_sensors(){
    #ifdef DEBUG
    Serial.println("Initializing temp/hum sensor...");
    #endif

    if (! am2315.begin()) {
        #ifdef DEBUG
        Serial.println("Sensor not found, check wiring & pullups!");
        #endif
    }

    delay(3000);

    // Light sensor disabled
    // #ifdef DEBUG
    // Serial.println("Initializing light sensor...");
    // #endif

    // sen0390.begin();
    // begin() does a test read, so need to wait 4secs before first read
    // delay(4000);
}

void initialize_clock(){
    #ifdef DEBUG
    Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);
    #endif

    rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!rtc.IsDateTimeValid()) 
    {
        #ifdef DEBUG
        Serial.println("rtc lost confidence in the DateTime!");
        #endif

        rtc.SetDateTime(compiled);
    }

    if (rtc.GetIsWriteProtected())
    {
        #ifdef DEBUG
        Serial.println("rtc was write protected, enabling writing now");
        #endif

        rtc.SetIsWriteProtected(false);
    }

    if (!rtc.GetIsRunning())
    {
        #ifdef DEBUG
        Serial.println("rtc was not actively running, starting now");
        #endif

        rtc.SetIsRunning(true);
    }

    RtcDateTime now = rtc.GetDateTime();
    
    if (now < compiled) 
    {
        #ifdef DEBUG
        Serial.println("rtc is older than compile time!  (Updating DateTime)");
        #endif

        rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        #ifdef DEBUG
        Serial.println("rtc is newer than compile time. (this is expected)");
        #endif
    }
    else if (now == compiled) 
    {
        #ifdef DEBUG
        Serial.println("rtc is the same as compile time! (not expected but all is fine)");
        #endif
    }
}

void setup() {
    #ifdef DEBUG
    delay(5000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    #endif

    initialize_sensors();

    pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(RAIN_SENSOR_PIN, precipitationInterupt, FALLING);  

    initialize_clock();

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
            #ifdef DEBUG
            Serial.println(F("board not known to library; add pinmap or update getconfig_thisboard.cpp"));
            #endif
        }
    }

    os_init_ex(pPinMap);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 10);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job (sending automatically starts OTAA too)
    do_send_data(&sendjob);
}

void loop() {
    os_runloop_once();
}