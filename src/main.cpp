/*----------------------------------------------------------------------

Quilaco HAUS - Home automation for quilaco
Implemented with Atom / VSC IDE

pubsubclient : https://github.com/Imroy/pubsubclient
ntpclient : https://github.com/arduino-libraries/NTPClient


--- MQTT OTA ----------------------------------

~/Documents/PlatformIO/Projects/q-esp32 haus/.pioenvs/esp01_1m

sftp> put firmware.bin          // 292384 ?
sftp {user}@{host}:{remote_dir} <<< $'put {local_file_path}'
mosquitto_pub -h 192.168.10.124 -t '/pucon/gast/ota' -r -f firmware.bin

-------------------------------------------------------------------------

Ricardo Timmermann

28 Apr 17   : Uploding Quilaco
8 Jun 17    : Checking code
21 Oct 17   : New HW blowed up
25 Mar 18   : Removing IoT Manager, using local mosquitto / OpenHAB
18 May 18   : New HW for ESP32 and full code refactoring

TEST:

test Alarm
test Current

TODO:

move libs to local


HardwareSerial Serial1(2)
GPIO16 U2RXD
GPIO17 U2TXD

-------------------------------------------------------------------------*/

#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>           // https://github.com/Imroy/pubsubclient

#include <Ticker.h>                 // move to lib

#include <WiFiUdp.h>
#include <NTPClient.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "SSD1306AsciiWire.h"

const char *ssid =  "Quilaco";      // cannot be longer than 32 characters!
const char *pass =  "";             // WiFi password

WiFiClient wifi;

#define OFF 0
#define ON 1

HardwareSerial Serial1(2);          // UART2 GPIO16 U2RXD, GPIO17 U2TXD

// == MQTT server ======================================

String mqttServerName = "192.168.10.124";
int    mqttport = 1883;
String mqttuser =  "";              // from CloudMQTT account data
String mqttpass =  "";              // from CloudMQTT account data

PubSubClient mqttClient( wifi, mqttServerName, mqttport );
void callback( const MQTT::Publish& sub );

void autoLicht( void );             // light precense emulation
String LogStatus( String );
void log2DB( String );

#define MAXBUF 80
char sJson[MAXBUF];

// == NTP time ====================================================

#define NTP_OFFSET   -4 * 60 * 60       // In seconds for Chile time
#define NTP_INTERVAL 60 * 1000          // In miliseconds
#define NTP_ADDRESS  "cl.pool.ntp.org"  // "europe.pool.ntp.org"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// == OLED ===============================================

#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

#define OX_VOLT     0
#define OX_STATE    2
#define OX_SIMUL    4
#define OX_TEMP     5
#define OX_SUB      6
#define OX_PUB      7

void oledSet( int line )
{
    if( line==0 ) oled.set2X();
    else oled.set1X();

    oled.setCursor(0,line);
    oled.clearToEOL();
}

// == devices ============================================

String sDeviceID =      "HAUS";
String sFeed =          "/pucon/haus/";

#define S_OTA           "ota"                   // OTA update
#define S_STATE         "/state"

#define T_BATTERY       "battery/sensor"
#define T_STATE         "state/sensor"          // multiple state in json format
#define T_DHT           "DHT-room/sensor"    	

#define T_IRKUECHE      "IR-kueche/sensor"
#define T_IRBACK        "IR-back/sensor"
#define T_ALARM         "alarm/sensor"
#define T_GALPON        "galpon/sensor"

#define T_LDKUECHE      "kueche"
#define T_LDBED         "licht"
#define T_LDLIVING      "living"
#define T_LDROOM        "room"

#define T_IO12V         "IO12V"

#define T_LSIMULATOR    "LightSimulator"

// == HW GPIO defines =====================================================

#define IO_IO12V        2
#define IO_LDBED        32
#define IO_LDKUECHE     33
#define IO_LDLIVING     25
#define IO_LDROOM       12

#define IO_ALARM        35
#define IO_SWGALPON     26
#define IO_IRKUECHE     27
#define IO_IRBACK       14

#define ADC_BATTERY     36      // ADC0
#define ADC_LUMI        39      // ADC3
#define ADC_AMPS        34      // ADC6

#define SCL             22
#define SDA             21

#define IO_DHT          4

// == DHT 11  =================================

#define DHTTYPE         DHT11         // DHT 21 (AM2301)
#define DHT_PIN         IO_DHT

DHT_Unified dht(DHT_PIN, DHTTYPE);

/*--------------------------------------------------------------------------
;
;   watchDog settings, uses Ticker.h
;
;---------------------------------------------------------------------------*/

void ISRwatchdog();

#define TM_ISR 5
static int watchdogCount = 0;

void ISRwatchdog()
{
    ++watchdogCount;
    if( watchdogCount == 10 ) {
        Serial.printf("\n\n== Watchdog timeout ==\n\n");
        ESP.restart();
    }
}

Ticker timer1( ISRwatchdog, 1000, MILLIS );

/*--------------------------------------------------------------------------------
;
;   light precense emulation, internet independ function, uses Ticker.h
;
;--------------------------------------------------------------------------------*/

#define TM_LSIMUL  (10*60)        // 10*60

static unsigned long tm_MinuteCnt = 0;
static bool simulatorMode = 0;

// == array to tell the relay ON / OFF in lapses of 10 minutes.

uint8_t lightState[] = {

                0,2,3,1,15,7,       // 0    20:00 PM
                5,7,3,15,15,3,      //      21:00
                6,6,6,2,4,2,        // 12   22:00
                4,2,6,6,0,0,        //      23:00
                0,0,0,0,0,0,        // 24   24:00
                0,0,0,4,0,0,        //      1:00 AM
                0,0,0,0,0,0,        // 36   2:00
                0,0,2,0,0,0,        //      3:00
                0,0,0,0,0,0,        // 48   4:00
                0,0,0,0,0,0,        //      5:00
                0,0,0,6,1,0 };      // 60   6:00 AM

void LightSimulator( void )
{
    int lumi;
    lumi = analogRead( ADC_LUMI );

    // -- day mode --------------
    if( lumi>40 ) {
        tm_MinuteCnt=0;
        digitalWrite( IO_LDLIVING, LOW );
        digitalWrite( IO_LDKUECHE, LOW );
        digitalWrite( IO_LDBED, LOW );
        digitalWrite( IO_LDROOM, LOW );               
        return;
        }

    // == night mode ==============
    if( simulatorMode == ON ) {
        digitalWrite( IO_LDLIVING, lightState[tm_MinuteCnt] & 1 ? HIGH : LOW );
        digitalWrite( IO_LDKUECHE, lightState[tm_MinuteCnt] & 2 ? HIGH : LOW );
        digitalWrite( IO_LDROOM, lightState[tm_MinuteCnt] & 4 ? HIGH : LOW );
        digitalWrite( IO_LDBED, lightState[tm_MinuteCnt] & 8 ? HIGH : LOW );
    }

    if( tm_MinuteCnt < sizeof(lightState)-1 )
        ++tm_MinuteCnt;
}

Ticker timer2( LightSimulator, 1000, MILLIS );

/*--------------------------------------------------------------------------
;
;   Blink on-board LED for status
;
;---------------------------------------------------------------------------*/

void blinkLed()
{
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(100);
//    digitalWrite(LED_BUILTIN, HIGH);
}

/*--------------------------------------------------------------------------------
;
;   time stamp string for logs
;
;--------------------------------------------------------------------------------*/

String LogStatus( String log )
{

    timeClient.update();

    String formattedTime = timeClient.getFormattedDate();
    formattedTime += " ";
    formattedTime += timeClient.getFormattedTime();
    formattedTime += " : ";

//    Serial.println( formattedTime + log );

    return formattedTime + log;

}

/*--------------------------------------------------------------------------------
;
;   Log to mySql DB
;
;--------------------------------------------------------------------------------*/

WiFiClient client;

void log2DB( String sLog )
{

    String sMsg = "devID=" + sDeviceID + "&message=";
    sMsg += sLog;

//    Serial.println( sMsg);

    if( client.connect( "192.168.10.124", 8000 ) ) {

//        Serial.println("--> Connected to server");

        client.println("POST /api/records HTTP/1.1");
        client.println("Host: 192.168.10.124:8000");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.println("Cache-Control: no-cache");

        client.println("Content-Length: " + String(sMsg.length()));

        client.println();
        client.println( sMsg );
        client.stop();
    }

    else
        Serial.println("--> connection failed");
}

/*--------------------------------------------------------------------------------
;
;   publish message
;
;--------------------------------------------------------------------------------*/

void pubMessage( String sTopic, String sPayload )
{
    Serial.println(" p>> " + sFeed + sTopic + "  :  " + sPayload );
    mqttClient.publish( sFeed + sTopic, sPayload );
}

/*--------------------------------------------------------------------------------
;
;   reconnect to MQTT broker
;
;--------------------------------------------------------------------------------*/

void reconnect_Broker( void )
{

    Serial.print( "Connecting to MQTT server ... " );
    bool success;

    String clientId = sDeviceID + "-"+ WiFi.macAddress();

    if( mqttuser.length() > 0 )
        success = mqttClient.connect( MQTT::Connect( clientId ).set_auth(mqttuser, mqttpass) );
    else success = mqttClient.connect( clientId );

    // -- subscribe for what ever ... ----------------------------

    if (success) {
        Serial.println( "OKI" );

        mqttClient.set_callback( callback );

        mqttClient.subscribe( sFeed + "+/control" );

//        mqttClient.subscribe( sFeed + S_OTA );

        log2DB( "MQTT connected" );
        }

    else {
        Serial.println( "MQTT connection FAIL" );
        delay(1000);
        }
}

/*--------------------------------------------------------------------------------
;
;   publish current gpio-out state, this for openHAB feedback
;
;--------------------------------------------------------------------------------*/

#define TM_PUBSTATE (1000*30)            // 30 sec min : State of publish
static unsigned long lastStatusTime;

void publishPayloadState( void )
{
    if ( (millis() - lastStatusTime) > TM_PUBSTATE ) {

        // Serial.printf( "Hupe = %d, Relay=%d, Patio=%d\n", digitalRead(IO_HUPE), digitalRead(IO_RELAY), digitalRead(IO_PATIOLICHT) );

        pubMessage( T_LDLIVING S_STATE, digitalRead(IO_LDLIVING) ? "ON" : "OFF" );
        pubMessage( T_LDKUECHE S_STATE, digitalRead(IO_LDKUECHE) ? "ON" : "OFF" );
        pubMessage( T_LDBED S_STATE, digitalRead(IO_LDBED) ? "ON" : "OFF" );
        pubMessage( T_LDROOM S_STATE, digitalRead(IO_LDROOM) ? "ON" : "OFF" );
        pubMessage( T_IO12V S_STATE, digitalRead(IO_IO12V) ? "ON" : "OFF" );       
        pubMessage( T_LSIMULATOR S_STATE, simulatorMode ? "ON" : "OFF" );

        lastStatusTime = millis();
        }
}

/*--------------------------------------------------------------------------------
;
;   ESP8266 Setup
;
;--------------------------------------------------------------------------------*/

void setup()
{

   // == oled init ============

    Wire.begin();
    oled.begin( &Adafruit128x64, 0x3C );        //ssd1306_init(0, 22, 21)) - 0x3C

    oled.setFont(Adafruit5x7);

    oledSet( OX_VOLT );
    oled.print("Q-HAUS");
    oledSet( OX_STATE );
    oled.print( "ver: " __DATE__ );

    // == UART =====================

    Serial.begin(115200);                       // for debugging
    while (!Serial);                            // wait for serial port to connect
    
    Serial1.begin(1200,SERIAL_8N1,16,17);
    while (!Serial1);

    // == wifi =======================

    Serial.print("Connecting WiFi : ");
    Serial.print(ssid);
    Serial.print("...");

    WiFi.begin(ssid, pass);

    int cnt=0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if( cnt++>100) ESP.restart();
    }

    Serial.println( "Success" );
    Serial.print( "IP address: " );
    Serial.println( WiFi.localIP() );
    Serial.println("");

    WiFi.mode( WIFI_STA );
   
    log2DB( ">>> Q-ESP32 Haus Started <<<");
    log2DB( "Flash ver : " __DATE__  );

    // == defaults ==================================

    simulatorMode = ON;
    dht.begin();

    analogReadResolution(12);

    // -- setup gpio's -----------------------

    // out
    pinMode(IO_IO12V, OUTPUT);

    pinMode(IO_LDBED, OUTPUT);
    pinMode(IO_LDKUECHE, OUTPUT);
    pinMode(IO_LDLIVING, OUTPUT);
    pinMode(IO_LDROOM, OUTPUT);

    digitalWrite(IO_IO12V,LOW);
    digitalWrite(IO_LDBED,LOW);
    digitalWrite(IO_LDKUECHE,LOW);
    digitalWrite(IO_LDBED,LOW);
    digitalWrite(IO_LDKUECHE,LOW);

    // in

    pinMode(IO_SWGALPON, INPUT_PULLUP);
    pinMode(IO_IRKUECHE, INPUT_PULLUP);
    pinMode(IO_IRBACK, INPUT_PULLUP);
    pinMode(IO_ALARM, INPUT_PULLUP);    
    
    // time interrupt services

    timer1.start();         // ISRwatchdog
    timer2.start();         // hupeAlarm
    
}

/*--------------------------------------------------------------------------------
;
;   main() loop
;
;--------------------------------------------------------------------------------*/

#define TM_LOOP (1000*5*1)                  // 1  min : loop
#define TM_IR (1000*60*15)                  // 15 min : IR

static unsigned long tm_loop;
static unsigned long tm_IRback= 0;
static unsigned long tm_IRkueche= 0;

int prevGalponState = 0;
int prevAlarmState = 0;

float get_VoltageAvg( void );

void loop()
{
    watchdogCount = 0;      // reset watchdog to signal keepalife

    // == reconnect WiFi ===========================================

    int cnt=0;
    while (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
    
        Serial.printf( "reconnecting WiFi %d\n", cnt );

        oledSet( OX_STATE );
        oled.printf( "reconnect WiFi %d", cnt );

        delay(500);
        if( cnt++>100) ESP.restart();
    }

    // == connect to MQTT broker if not connected =====================================

    if ( !mqttClient.connected() ) {
        reconnect_Broker();
        return;
        }

    timer1.update();         // ISRwatchdog
    timer2.update();         // LightSimulator

    // == IR Kueche =========================================================

    int IRstate;

    if( millis() > tm_IRkueche ) {

        IRstate = digitalRead( IO_IRKUECHE );

        if( IRstate==1 ) {
            tm_IRkueche = millis() + TM_IR;
            pubMessage( T_IRKUECHE, LogStatus("IR Kueche") );
            log2DB( "IR Kueche" );
            }
        }

    // == IR Back =========================================================

    if( millis() > tm_IRback ) {

        IRstate = digitalRead( IO_IRBACK );

        if( IRstate==1 ) {
            tm_IRback = millis() + TM_IR;
            pubMessage( T_IRBACK, LogStatus("IR Backdoor") );
            log2DB( "IR Backdoor" );
            }
        }

    mqttClient.loop();
    yield();

    publishPayloadState();                  // for feedback in openHUP

    // == continue only each TM_LOOPs =========================================
    //
    //      Battery, Lumi, DTH, Alarm, Galpon SW
    //

    if ( millis() - tm_loop < TM_LOOP) return;
    tm_loop = millis();

   // == serial test ==========
/*
    if ( Serial1.available()>0 ) {
        Serial.write( Serial1.read() );
    }
*/
    // == system state =======================================================

    int lumi;
    lumi = analogRead( ADC_LUMI );         // lumi
    
    int32_t rssi = WiFi.RSSI();

    snprintf( sJson, MAXBUF, "{ \"Lumi\":%d, \"Scnt\":%d, \"dBm\":%d }", lumi, tm_MinuteCnt, rssi );
    pubMessage( T_STATE, sJson );

    //Serial.println( sJson );

    oledSet( OX_SIMUL );
    oled.printf( "L=%d, C=%d, S=%d\n", lumi, tm_MinuteCnt, lightState[tm_MinuteCnt] );

    // == publish ADC Voltage ===================================================

    /*
    vout = vin * R2/(R1+R2) >> 0.0896 (9.63/107.43) ; 11.1558 

        R1 = 97.8
        R2 = 9.63
        
        #define ANARES (3.3/2048)       

        #define BAT_VMIN 23.8
        #define BAT_VMAX 26.0               // 25.4

        perc = 100*(volt-BAT_VMIN)/(BAT_VMAX-BAT_VMIN);

        if( perc<0.) perc = 0.;
        if( perc>100.) perc = 100;

        ESP ADC is not stable and not liner. So I have to average a rad set
        and then linearize.
    */

    float volt = get_VoltageAvg();

    // == battery volt ===============================================

    volt = volt * (24.55/1.19);         // measured factor from vin / vout

    // calculate % of load

    #define BAT_VMIN 23.8
    #define BAT_VMAX 26.0

    if( volt<BAT_VMIN ) {
        sprintf( sJson, "Low Voltage = %.1f V", volt );
        log2DB( sJson );
        }

    int perc = 100*(volt-BAT_VMIN)/(BAT_VMAX-BAT_VMIN);

    if( perc<0.) perc = 0;
    if( perc>100.) perc = 100;

    sprintf( sJson, "{ \"Voltage\":%.1f, \"Power\":%d }", volt, perc );
    pubMessage( T_BATTERY, sJson );

    oledSet( OX_VOLT );
    oled.printf( "%.1fV %d%%", volt, perc ) ;

    // == publish galpon status =========================================

    int GalponState = digitalRead( IO_SWGALPON );

    if( GalponState != prevGalponState) {

        prevGalponState = GalponState;

        if( GalponState==0 ) {
            pubMessage( T_GALPON, LogStatus( "CLOSE" ) );
            log2DB( "Galpon door close" );
            }

        else {
            pubMessage( T_GALPON, LogStatus( "OPEN" ) );
            log2DB( "Galpon door open" );
            }
        }

    // == publish alarm state =========================================

    int alarmState = digitalRead( IO_ALARM );

    if( alarmState != prevAlarmState) {

        prevAlarmState = alarmState;

        if( alarmState==0 ) {
            pubMessage( T_ALARM, LogStatus( "ON" ) );
            log2DB( "Alarm ON" );
            }

        else {
            pubMessage( T_ALARM, LogStatus( "OFF" ) );
            log2DB( "Alarm OFF" );
            }
        }

    // == DHT Room =========================================================

    sensors_event_t event;

    dht.temperature().getEvent(&event);
    int temperature = event.temperature;

    dht.humidity().getEvent(&event);
    int humidity = event.relative_humidity;

    snprintf( sJson, MAXBUF, "{ \"Temp\":%d, \"Hum\":%d }", temperature, humidity );
    pubMessage( T_DHT, sJson );

    oledSet( OX_TEMP );
    oled.printf( "Temp: %d, Hum :%d", temperature, humidity );
}

/*--------------------------------------------------------------------------------
;
;   subscription: get messages from OpenHAB
;
;   /pucon/HAUS/LDBED/control 0/1
;
;--------------------------------------------------------------------------------*/

void callback( const MQTT::Publish& sub )
{

//    blinkLed();

    // -- subscribe from widgets ---------------

    Serial.print( "s<< ");
    Serial.print( sub.topic() );
    Serial.print(" <= ");
    Serial.println( sub.payload_string() ) ;

    oledSet( OX_SUB);
    oled.printf( "s:%s:%s", sub.topic(), sub.payload_string() );

    // -- Light Simulator mode -----------------------------

    if( strstr( sub.topic().c_str(), T_LSIMULATOR )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LSIMULATOR S_STATE, "OFF" );
            simulatorMode = OFF;
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_LSIMULATOR S_STATE, "ON" );
            simulatorMode = ON;
            }
        }

   // -- IO 12V ----------------------------------

    if( strstr( sub.topic().c_str(), T_IO12V )>0 ) {

        Serial.println( sub.topic().c_str());

        if (sub.payload_string() == "0") {
            pubMessage( T_IO12V S_STATE, "OFF" );
            digitalWrite(IO_IO12V,LOW);
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_IO12V S_STATE, "ON" );
            digitalWrite(IO_IO12V,HIGH);
            }
        }


    // -- Bed Licht ----------------------------------

    if( strstr( sub.topic().c_str(), T_LDBED )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LDBED S_STATE, "OFF" );
            digitalWrite(IO_LDBED,LOW);
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_LDBED S_STATE, "ON" );
            digitalWrite(IO_LDBED,HIGH);
            }
        }

    // -- Kueche Licht ---------------------

    if( strstr( sub.topic().c_str(), T_LDKUECHE )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LDKUECHE S_STATE, "OFF" );
            digitalWrite(IO_LDKUECHE,LOW);
            }
        else if (sub.payload_string() == "1") {
            pubMessage( T_LDKUECHE S_STATE, "ON" );
            digitalWrite(IO_LDKUECHE,HIGH);
            }
        }

  // -- Living ----------------------------------

    if( strstr( sub.topic().c_str(), T_LDLIVING )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LDLIVING S_STATE, "OFF" );
            digitalWrite(IO_LDLIVING,LOW);
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_LDLIVING S_STATE, "ON" );
            digitalWrite(IO_LDLIVING,HIGH);
            }
        }

  // -- Room ----------------------------------

    if( strstr( sub.topic().c_str(), T_LDROOM )>0 ) {

        if (sub.payload_string() == "0") {
            pubMessage( T_LDROOM S_STATE, "OFF" );
            digitalWrite(IO_LDROOM,LOW);
            }

        else if (sub.payload_string() == "1") {
            pubMessage( T_LDROOM S_STATE, "ON" );
            digitalWrite(IO_LDROOM,HIGH);
            }
        }

}


/*--------------------------------------------------------------------------------
;
;   get averaje voltage adjusted using analogRead
;
;--------------------------------------------------------------------------------*/

float get_VoltageAvg( void )
{
    #define ANARES (3.3/4096)
    
    #define MAX_SAMPLING 8

    int adc = 0;                       
    for( int n=0; n<MAX_SAMPLING; n++ ) 
        adc += analogRead( ADC_BATTERY ); 
    adc /= MAX_SAMPLING;

    float av = adc * ANARES;
    float volt;

         if( av < 0.2 ) volt = av;
    else if( av < 0.7 ) volt = av + .13;
    else if( av < 1.6 ) volt = av + .14;
    else if( av < 2.2 ) volt = av + .15;
    else if( av < 2.6 ) volt = av + .16;
    else if( av < 2.7 ) volt = av + .15;
    else if( av < 2.8 ) volt = av + .12;
    else if( av < 2.9 ) volt = av + .09;
    else if( av < 3.0 ) volt = av + .05;
    else volt = av;

    oledSet( OX_STATE );
    oled.printf( "%d  %.2f: %.2f", adc, av, volt ) ;

    return volt;
}