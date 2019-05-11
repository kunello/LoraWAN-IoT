/*******************************************************************************

* Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

*

* Permission is hereby granted, free of charge, to anyone

* obtaining a copy of this document and accompanying files,

* to do whatever they want with them without any restriction,

* including, but not limited to, copying, modification and redistribution.

* NO WARRANTY OF ANY KIND IS PROVIDED.

*

* This example sends a valid LoRaWAN packet with payload "Hello,
* world!", using frequency and encryption settings matching those of
* the The Things Network.

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

* Do not forget to define the radio type correctly in config.h.

*

*******************************************************************************/



#include <lmic.h>

#include <hal/hal.h>

#include <SPI.h>



#define VBATPIN 9




// Backwards - LSB

static const u1_t PROGMEM APPEUI[8]= { 0x93, 0xAD, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}



// Backwards - LSB

static const u1_t PROGMEM DEVEUI[8]= { 0xAE, 0x19, 0x8B, 0x79, 0xF6, 0xDB, 0xB9, 0x00 };

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}



// Forwards - MSB

static const u1_t PROGMEM APPKEY[16] = { 0x49, 0x87, 0xAD, 0xCA, 0x93, 0x4E, 0xC4, 0x9F, 0x32, 0x22, 0xA4, 0x8F, 0x18, 0xEE, 0x67, 0x53 };

void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}



static osjob_t sendjob;



// Schedule TX every this many seconds (might become longer due to duty

// cycle limitations).

const unsigned TX_INTERVAL = 600;  //time between readings in secs



// Pin mapping for TTGO Tbeam

const lmic_pinmap lmic_pins = {

    .nss = 18,

    .rxtx = LMIC_UNUSED_PIN,

    .rst = LMIC_UNUSED_PIN,

    .dio = {27, 19, 26},

};







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



            // Disable link check validation (automatically enabled

            // during join, but not supported by TTN at this time).

            LMIC_setLinkCheckMode(0);

            break;

        case EV_RFU1:

            Serial.println(F("EV_RFU1"));

            break;

        case EV_JOIN_FAILED:

            Serial.println(F("EV_JOIN_FAILED"));

            break;

        case EV_REJOIN_FAILED:

            Serial.println(F("EV_REJOIN_FAILED"));

            break;

            break;

        case EV_TXCOMPLETE:

            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

//            if (LMIC.txrxFlags & TXRX_ACK)

//              Serial.println(F("Received ack"));

              //Serial.println(F("Clearing variables"));

            if (LMIC.dataLen) {

              Serial.println(F("Received "));

              Serial.println(LMIC.dataLen);

              Serial.println(F(" bytes of payload"));

            }

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

        default:

            Serial.println(F("Unknown event"));

            break;

    }

}



void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running

    if (LMIC.opmode & OP_TXRXPEND) {

        Serial.println(F("OP_TXRXPEND, not sending"));

    } else {





  // Prepare upstream data transmission at the next possible time.


  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back

  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage

  measuredvbat /= 1024; // convert to voltage

  int pres = analogRead(A0) * 1000;

  int vbat = measuredvbat * 100;

  int waterpressure = pres * 100 ;



  byte payload[4];

  payload[1] = highByte(vbat);

  payload[2] = lowByte(vbat);

  payload[3] = highByte(pres);

  payload[4] = lowByte(pres);



  LMIC_setTxData2(1, payload, sizeof(payload),0);

    // Next TX is scheduled after TX_COMPLETE event.

}

}

void setup() {



    delay(2500);   // Give time to the ATMega32u4 port to wake up and be recognized by the OS.

    Serial.begin(9600);

    delay(2000);

    Serial.println(F("Starting"));



    // LMIC init

    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.

    LMIC_reset();



     // Let LMIC compensate for +/- 1% clock error

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);



    // Start job (sending automatically starts OTAA too)

    do_send(&sendjob);

}



void loop() {
  os_runloop_once();

}
