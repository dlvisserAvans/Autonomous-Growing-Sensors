#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <arduino_lmic_hal_boards.h>
#include <ArduinoLowPower.h>
#include <I2CSoilMoistureSensor.h>
#include <Wire.h>

// Include libraries for DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// uncomment for DEBUG mode
#define DEBUG

// Temperature Sensor is connected to port 11
#define ONE_WIRE_BUS 11

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress gsTemperatureSensor;

static byte sensorType = 0x01;

void printTemperature(DeviceAddress);

I2CSoilMoistureSensor soilSensor;
const int soilMinValue = 260;
const int soilMaxValue = 600;

int sleepTime = 3600000; // In milliseconds 

static const u1_t PROGMEM APPEUI[8]= { 0xCD, 0x9C, 0xC6, 0xAC, 0x23, 0xF9, 0x81, 0x60 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]= { 0xE2, 0xFF, 0xBB, 0xD3, 0x9D, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x2F, 0x59, 0x75, 0x94, 0x6A, 0x5F, 0x0A, 0x50, 0x4F, 0x6A, 0x6A, 0xA2, 0x00, 0xB8, 0x20, 0x46 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void do_send(osjob_t* j);

// { 0xST, 0xTM, 0xTM, 0xSM, 0xSV, 0xSV, 0xEB }
// ST = Sensor Type
// TM = temperature
// SM = Soil Moisture
// SV = Suction Voltage
// EB = End Byte - Required for Techtenna connection
static byte mydata[] = { sensorType, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
static osjob_t sendjob;

const unsigned TX_INTERVAL = 1; // In seconds

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void putToSleep(){
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.deepSleep(sleepTime);
    digitalWrite(LED_BUILTIN, HIGH);
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

            putToSleep();

            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        #ifdef DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        while (soilSensor.isBusy()) delay(50); // available since FW 2.3
        // Read soil moisture
        int capacitance = soilSensor.getCapacitance();
        int soilMoisture = constrain((float)(capacitance - soilMinValue) / (float)(soilMaxValue - soilMinValue) * 100.0, 0.0, 100.0);

        mydata[3] = soilMoisture;

        // Read temperature
        sensors.requestTemperatures(); // Send the command to get temperatures
        delay(1000);
        int tempKelvin = (sensors.getTempC(gsTemperatureSensor) + (float)273.155555555) * 100;

        mydata[1] = tempKelvin >> 8 & 0xff;
        mydata[2] = tempKelvin & 0xff;

        #ifdef DEBUG
        Serial.print("Soil Moisture Capacitance: ");
        Serial.print(capacitance); //read capacitance register
        Serial.print(", Soil Moisture Percentage: ");
        Serial.print(soilMoisture);
        Serial.println("%");
        Serial.print("Temp K: ");
        Serial.println(tempKelvin);
        #endif

        soilSensor.sleep(); // available since FW 2.3

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);

        #ifdef DEBUG
        Serial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    #ifdef DEBUG
    delay(5000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    #endif

    Wire.begin();

    #ifdef DEBUG
        Serial.println("Initializing soil moisture sensor...");
    #endif

    // Initialize soil sensor
    soilSensor.begin(); // reset sensor
    delay(2000); // give some time to boot up

    #ifdef DEBUG
    Serial.print("I2C Soil Moisture Sensor Address: ");
    Serial.println(soilSensor.getAddress(),HEX);
    Serial.print("Sensor Firmware version: ");
    Serial.println(soilSensor.getVersion(),HEX);
    Serial.println();

    Serial.println("Initializing temperature sensor...");
    #endif

    // Initialize temperature sensor
    sensors.begin();

    #ifdef DEBUG
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    if (!sensors.getAddress(gsTemperatureSensor, 0)) Serial.println("Unable to find address for Device 0");
    #endif

    sensors.setResolution(gsTemperatureSensor, 9);

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

    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}