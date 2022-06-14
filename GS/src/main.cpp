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
#include <ArduinoLowPower.h>

#include <I2CSoilMoistureSensor.h>

#include <Wire.h>

// Include libraries for DS18B20 temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to port 2
#define ONE_WIRE_BUS 11

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress gsTemperatureSensor;

static byte sensorType = 0x01;

void printTemperature(DeviceAddress);

I2CSoilMoistureSensor soilSensor;
const int soilMinValue = 260;
const int soilMaxValue = 600;

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

// { 0xST, 0xTM, 0xTM, 0xSM, 0xSV, 0xSV, 0xEB }
// ST = Sensor Type
// TM = temperature
// SM = Soil Moisture
// SV = Suction Voltage
// EB = End Byte - Required for Techtenna connection
static byte mydata[] = { sensorType, 0x01, 0x77, 0x64, 0x00, 0x00, 0x01};
// static byte mydata[] = {0x02, 0x01, 0x77, 0x64, 0x07, 0xD0, 0xFF, 0xFF, 0x01};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5;


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void printTemperature(DeviceAddress deviceAddress)
{
  int tempC = sensors.getTempC(deviceAddress);
  int tempK = tempC + 273;
  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp K: ");
  Serial.println(tempK);
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
            digitalWrite(13, LOW);
            LowPower.deepSleep(600000);
            digitalWrite(13, HIGH);
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        while (soilSensor.isBusy()) delay(50); // available since FW 2.3
        Serial.print("Soil Moisture Capacitance: ");
        int capacitance = soilSensor.getCapacitance();
        Serial.print(capacitance); //read capacitance register
        Serial.print(", Soil Moisture Percentage: ");
        int soilMoisture = constrain((float)(capacitance - soilMinValue) / (float)(soilMaxValue - soilMinValue) * 100.0, 0.0, 100.0);
        mydata[3] = soilMoisture;
        Serial.print(soilMoisture);
        Serial.print("%");
        Serial.print(", Temperature: ");
        Serial.println(soilSensor.getTemperature()/(float)10); //temperature register
        soilSensor.sleep(); // available since FW 2.3

        sensors.requestTemperatures(); // Send the command to get temperatures
        int tempKelvin = (sensors.getTempC(gsTemperatureSensor) + (float)273.155555555) * 100;
        mydata[1] = tempKelvin >> 8 & 0xff;
        mydata[2] = tempKelvin & 0xff;

        // printTemperature(gsTemperatureSensor); // Use a simple function to print out the data
    // delay(100);

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    // while (! Serial)
    //     ;
    Serial.begin(9600);


    Serial.print("millis(): ");
    Serial.println(millis());
    Serial.print("__TIME__: ");
    Serial.println(__TIME__);
    digitalWrite(13, LOW);
    LowPower.deepSleep(5000);
    digitalWrite(13, HIGH);
    Serial.print("millis(): ");
    Serial.println(millis());
    Serial.print("__TIME__: ");
    Serial.println(__TIME__);
    
    Serial.println(F("Starting"));

    Wire.begin();

    Serial.println("Initializing soil moisture sensor...");
    soilSensor.begin(); // reset sensor
    delay(2000); // give some time to boot up
    Serial.print("I2C Soil Moisture Sensor Address: ");
    Serial.println(soilSensor.getAddress(),HEX);
    Serial.print("Sensor Firmware version: ");
    Serial.println(soilSensor.getVersion(),HEX);
    Serial.println();

    Serial.println("Initializing temperature sensor...");
    sensors.begin();

    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    if (!sensors.getAddress(gsTemperatureSensor, 0)) Serial.println("Unable to find address for Device 0");

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
    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);

#if CFG_LMIC_EU_like
    // This makes joins faster in the US because we don't wander all over the
    // spectrum.
    // LMIC_selectSubBand(1);
#endif

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}