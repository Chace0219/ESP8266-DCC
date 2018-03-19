/*
 * 
 *  Modified by jinzhouyun.
 *  Date : 2016.11.12
*/

#include <ESP8266WiFi.h>
#include <Servo.h> 

// BEGIN BY INCLUDING THE HEADER FILES FOR EACH MODULE
 
#include "DCCpp_ESP.h"
#include "PacketRegister.h"
#include "CurrentMonitor.h"
#include "Sensor.h"
#include "SerialCommand.h"
#include "Accessories.h"
#include "EEStore.h"
#include "Config.h"
#include "Comm.h"
#include "myFunc.h"

//
#include "WEMOS_Motor.h"
#include <Servo.h> 

volatile RegisterList mainRegs(MAX_MAIN_REGISTERS);    // create list of registers for MAX_MAIN_REGISTER Main Track Packets
volatile RegisterList progRegs(2);                     // create a shorter list of only two registers for Program Track Packets
CurrentMonitor mainMonitor(CURRENT_MONITOR_PIN_MAIN,"<p2>");  // create monitor for current on Main Track

/**/

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//

//#define IP_ADDRESS { 192, 168, 1, 200 }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE PORT TO USE FOR ETHERNET COMMUNICATIONS INTERFACE
//
#define ETHERNET_PORT 2560


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MAC ADDRESS ARRAY FOR ETHERNET COMMUNICATIONS INTERFACE
//
// #define MAC_ADDRESS {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE COUNT OF MAX CLIENTS
//
#define MAX_SRV_CLIENTS 1

/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE SSID OF BASE STATION

const char* ssid = "TP-LINK_64FB80";      

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE PASSWORD OF BASE STATION

const char* password = "chunxing151201";  

/// DEFINE DEFAULT CAB ADDRESS
int DEFAULTCAB = 3;

/////////////////////////////////////////////////////////////////////////////////////
// DEFINE Long Address Enable Flag
unsigned short nLongAddrFlag = false; // It means flag that using nLongAddress.

/* Definitions of Peripheral Port configuration */
// Definition of LED Pins
#define LED0      12
#define LED1      13
#define LED2      15

// Definition of Servo Pins
#define SERVOFRONT        3
#define SERVOREAR         4
#define SERVOREVERSER     5

/* WIFI Socket Server and Client Class instance */
WiFiServer server(ETHERNET_PORT);  // Create and instance of an WIFIServer
WiFiClient Client; // Create instance of an WIFIClient.

/* Configuration values */
unsigned short nDelayCycle = 200; // CV Number 101 (value user entered in sketch as per a CV)
unsigned short F13Momen_Value =  25; // CV Number 102 (value user entered in sketch as per a CV, treat as decimal, 0-255(?), increase speed by this factor, this is emulating momenting)
unsigned short F21Brake_Value =  25; // CV Number 103 (value user entered in sketch as per a CV, treat as decimal, 0-255(?), decrease speed by this factor, this is emulating braking)

/* Enable of Leds Command flag */
unsigned short bEnableLED = false; 

/* 
 *  If F13 Momentum_Value received ie < f 03 222 BYTE2 >, map(F13,0,255,0,200), 
 *  This flag is triggered when F13 Command is received and off when 100% speed.
*/
unsigned short bMomentumFlag = false; 
/* 
 *  If F21 Brake_Value received ie < f 03 223 BYTE2 >, map(F21,0,255,0,200), reduce motor speed using delay(Brake_Value) 
 *  This flag is triggered when F21 Command is received and off when 0% speed and STOP and Break.
*/
unsigned short bBreakFlag = false; 

/* Current Motor Speed and Direction */
uint8_t nMotorSpeed = 0; // Its range is in 0 ~ 126.
uint8_t nMotorDir = 0; // 1: Forward, 0: Backward

//Motor shiled I2C Address: 0x30, You can change it with jumper
//PWM frequency: 1000Hz(1kHz), It can be changed.
Motor M1(0x30, _MOTOR_A, 1000);//Motor A
Motor M2(0x30, _MOTOR_B, 1000);//Motor B

// Servo Motor control instance
Servo Front_CouplerServo;  // (Front Coupler)
Servo Rear_CouplerServo;  // (Rear Coupler) 
Servo ReverserServo;  // (Reverser)

// Current Positions of Servo Motors
unsigned short nFrontPos = 0;
unsigned short nRearPos = 0;
unsigned short nReverPos = 0;


/* My Own Time Blocks instances, */
static TON BreakTON, MomentumTON;
//
static Rtrg BreakTrg, MomentumTrg;

/* if SPEED>0, then brighten for 20s every 120s (firebox) */
#define BRIGHTENTIME 5000 // 20sec
#define DARKENTIME 10000 // 120sec
static TFLASH LEDBlinkObj; // in order to control LED as specified time logic

/*
 * 
*/
void InitVars()
{
    // 
    BreakTON.IN = 0;
    BreakTON.Q = 0;
    BreakTON.PT = nDelayCycle;
    BreakTON.ET = 0;

    BreakTrg.IN = 0;
    BreakTrg.Q = 0;
    BreakTrg.PRE = 0;

    //
    MomentumTON.IN = 0;
    MomentumTON.Q = 0;
    MomentumTON.PT = nDelayCycle;
    MomentumTON.ET = 0;

    MomentumTrg.IN = 0;
    MomentumTrg.Q = 0;
    MomentumTrg.PRE = 0;

    // 
    LEDBlinkObj.IN = 0;
    LEDBlinkObj.Q = 0;
    LEDBlinkObj.PT1 = BRIGHTENTIME;
    LEDBlinkObj.PT2 = DARKENTIME;
    LEDBlinkObj.ET = 0;

}


///////////////////////////////////////////////////////////////////////////////
// MAIN ARDUINO LOOP
///////////////////////////////////////////////////////////////////////////////

void loop(){
    SerialCommand::process();              // check for, and process, and new serial commands
    if(CurrentMonitor::checkTime())
    {      // if sufficient time has elapsed since last update, check current draw on Main and Program Tracks 
        mainMonitor.check();
    }
    Sensor::check();    // check sensors for activate/de-activate
    
    // LED Logic
    if(bEnableLED)
    {
        if(nMotorDir)
        { // Direction 1
            digitalWrite(LED0, HIGH);
            digitalWrite(LED1, LOW);
        }
        else
        { // Direction 0
            digitalWrite(LED0, LOW);
            digitalWrite(LED1, HIGH);
        }

        if(nMotorSpeed == 0)
        {
            digitalWrite(LED0, LOW);
            digitalWrite(LED1, LOW);
        }
    }
    else
    {
        digitalWrite(LED0, LOW);
        digitalWrite(LED1, LOW);
    }

    // Enable LED and if SPEED>0, then brighten for 20s every 120s (firebox)
    LEDBlinkObj.IN = bEnableLED && nMotorSpeed > 0;
    TFLASHFunc(&LEDBlinkObj);
    if(LEDBlinkObj.Q)
        digitalWrite(LED2, LOW);
    else
        digitalWrite(LED2, HIGH);
    
    //digitalWrite(LED_BUILTIN, LEDBlinkObj.Q);
    //digitalWrite(LED2, LEDBlinkObj.Q);

    BreakTON.IN = bBreakFlag && (BreakTON.Q == 0);
    BreakTON.PT = nDelayCycle;
    TONFunc(&BreakTON);
    BreakTrg.IN = BreakTON.Q;
    RTrgFunc(&BreakTrg);
    if(BreakTrg.Q)
    { // per given Delaycycle as CV when Breaking.
        if(nMotorSpeed == 0)
        {
            bBreakFlag = false;
            M1.setmotor(_SHORT_BRAKE);
            M2.setmotor(_SHORT_BRAKE);
            Serial.println("Motor Braked!");
        }  
        if(nMotorSpeed - map(F21Brake_Value, 0, 255, 0, 200) > 0)
            SetThrottleCallback(nMotorDir, nMotorSpeed - map(F21Brake_Value, 0, 255, 0, 200));        
        else
            SetThrottleCallback(nMotorDir, 0);
    }

    MomentumTON.IN = bMomentumFlag && (MomentumTON.Q == 0);
    MomentumTON.PT = nDelayCycle;
    TONFunc(&MomentumTON);
    MomentumTrg.IN = MomentumTON.Q;
    RTrgFunc(&MomentumTrg);
    if(MomentumTrg.Q)
    {  // per given Delaycycle as CV when momenting.
        if(nMotorSpeed + map(F13Momen_Value, 0, 255, 0, 200) < 126)
            SetThrottleCallback(nMotorDir, nMotorSpeed + map(F13Momen_Value, 0, 255, 0, 200));        
        else
        {
            bMomentumFlag = 0;
            SetThrottleCallback(nMotorDir, 126);
            Serial.println("Motor Speed Up 100%!");
        }
    }
    
} // loop

///////////////////////////////////////////////////////////////////////////////
// INITIAL SETUP
///////////////////////////////////////////////////////////////////////////////

void setup(){  

    Serial.begin(19200);            // configure serial interface
    Serial.println("Test Started!");
    Serial.flush();

    // Pin configuration
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);

    pinMode(LED1, OUTPUT);
    digitalWrite(LED1, LOW);

    pinMode(LED2, OUTPUT);
    digitalWrite(LED2, LOW);


    pinMode(LED_BUILTIN, OUTPUT);

    // Init Variables 
    InitVars();

    // initialize and load CAB address, Turnout, Sensor definitions stored in EEPROM
    EEStore::init();                                          

    // WIFI Begin
    WiFi.begin(ssid, password);
    Serial.print("\nConnecting to "); 
    Serial.println(ssid);

    // 
    M1.setmotor(_STOP);
    M2.setmotor(_STOP);
	
    uint8_t idx = 0;
    // If WIFI is not connected, 10 sec delay.
    while (WiFi.status() != WL_CONNECTED && idx++ < 20) delay(500);
    if (idx == 21)
    {
      	Serial.print("Could not connect to");
      	Serial.println(ssid);
      	while (1)
      	{ // When Wifi connection error, Led flickers per 0.5sec
        		digitalWrite(LED_BUILTIN, LOW);
        		delay(250);
        		digitalWrite(LED_BUILTIN, HIGH);
        		delay(250);
      	}
    }
    
    // Server socket start.
    server.begin();
    server.setNoDelay(true);

    // 
    Serial.println("DCC++ Base Station is Ready!");
    Serial.print("Local IP is ");
    Serial.println(WiFi.localIP());

    // 
    Front_CouplerServo.attach(SERVOFRONT);    
    Rear_CouplerServo.attach(SERVOREAR);    
    ReverserServo.attach(SERVOREVERSER);    
    
    SerialCommand::init(&mainRegs, &progRegs, &mainMonitor);   // create structure to read and parse commands from serial line
    digitalWrite(LED_BUILTIN, LOW);

    Serial.print("Current CAB is ");
    Serial.println(EEStore::eeStore->data.nCABAddr);
} // setup


///////////////////////////////////////////////////////////////////////////////

////////
void SetThrottleCallback(uint8_t nDir, short nSpeed)
{
    Serial.print("Setting Motor Direction:");
    if(nDir)
        Serial.print("Forward, ");
    else
        Serial.print("Backward, ");
    Serial.print("Motor Speed:");
    Serial.println(nSpeed);

    if(nSpeed == -1)
    { // Emergency Stop
        
    }

    nMotorSpeed = nSpeed;
    
    // Speed Mapping 
    int pwm = map(nMotorSpeed, 0, 126, 0, 100);
    Serial.print("PWM duty:");
    Serial.println(pwm);
    
    if(nDir)
    {
        M1.setmotor(_CW, pwm);
        M2.setmotor(_CW, 100-pwm);
    }
    else
    {
        M1.setmotor(_CCW, pwm);
        M2.setmotor(_CCW, 100-pwm);
    }

    if(pwm == 0)
    {
        M1.setmotor(_STOP);
        M2.setmotor( _STOP);
    }
}

void writeCVBitMainCallback(int nCV, uint8_t nBit, unsigned short nNewVal)
{
    Serial.println("Write CV Bit Main call back");
    Serial.print("CV Number:");
    Serial.print(nCV);
    Serial.print(", Bit Number:");
    Serial.println(nBit);
    if(nCV == 29 && nBit == 5)
    {
        if(nNewVal)
        {
            EEStore::eeStore->data.bLongAddrEnable = true;
            Serial.println("Switched Long Address Mode!");
            EEStore::eeStore->data.nCABAddr = EEStore::eeStore->data.nLongCAB;
            Serial.print("Current CAB is ");
            Serial.println(EEStore::eeStore->data.nCABAddr);
        }
        else
        {
            EEStore::eeStore->data.bLongAddrEnable = false;
            Serial.println("Switched Short Address Mode!");
            EEStore::eeStore->data.nCABAddr = EEStore::eeStore->data.nShortCAB;
            Serial.print("Current CAB is ");
            Serial.println(EEStore::eeStore->data.nCABAddr);
        }
    }
      
}

void WriteCVMainCallback(int nCV, unsigned short nNewVal)
{
    Serial.println("Write CV Main call back");
    switch(nCV)
    {
        case 1:
        { // CAB Short Address 
            Serial.print("Write CV Number : 1, value: ");
            Serial.println(nNewVal);
            EEStore::eeStore->data.nShortCAB = nNewVal;  
            if(EEStore::eeStore->data.bLongAddrEnable)
            { // when Long Address enabled
                Serial.println("You can not change CAB because it is in long address mode!");
            }
            else
            { 
                Serial.println("Setting Changed!");
                EEStore::eeStore->data.nCABAddr = EEStore::eeStore->data.nShortCAB;
                Serial.print("Current Addr is ");
                Serial.println(EEStore::eeStore->data.nCABAddr);
            }
        }
        break;

        case 17:
        { // High byte of Long CAB
            Serial.print("Write CV Number : 17, value: ");
            Serial.println(nNewVal);
            EEStore::eeStore->data.nLongCAB = nNewVal * 256 + (EEStore::eeStore->data.nLongCAB % 256);  
            Serial.print("Current Long CAB is ");
            Serial.println(EEStore::eeStore->data.nLongCAB);
            if(EEStore::eeStore->data.bLongAddrEnable)
            { // when Long Address enabled
                EEStore::eeStore->data.nCABAddr = EEStore::eeStore->data.nLongCAB;  
                Serial.print("Current Addr is ");
                Serial.println(EEStore::eeStore->data.nCABAddr);
            }
            else
            { 
                Serial.println("You can not change CAB because it is in short address mode!");
            }
        }
        break;

        case 18:
        { // Low byte of Long CAB
            Serial.print("Write CV Number : 18, value: ");
            Serial.println(nNewVal);
            EEStore::eeStore->data.nLongCAB = (EEStore::eeStore->data.nLongCAB / 256) * 256 + nNewVal;  
            Serial.print("Current Long CAB is ");
            Serial.println(EEStore::eeStore->data.nLongCAB);
            if(EEStore::eeStore->data.bLongAddrEnable)
            { // when Long Address enabled
                EEStore::eeStore->data.nCABAddr = EEStore::eeStore->data.nLongCAB;  
                Serial.print("Current Addr is ");
                Serial.println(EEStore::eeStore->data.nLongCAB);
            }
            else
            { 
                Serial.println("You can not change CAB because it is in short address mode!");
            }
        }
        break;

        case 101:
        { // delay(200) (user to modify as per CV)
            nDelayCycle = nNewVal;        
            Serial.print("Write CV Address:101, Delay Cycle value:");
            Serial.println(nDelayCycle);
        }
        break;

        case 102:
        { // +F13 Momentum_Value (value user entered in sketch as per a CV)
            F13Momen_Value = nNewVal;        
            Serial.print("Write CV Address:102, F13 Momentum value:");
            Serial.println(F13Momen_Value);
        }
        break;

        case 103:
        { // -F21 Brake_Value (treat as decimal, 0-255(?), decrease speed by this factor, this is emulating braking)
            F21Brake_Value = nNewVal;        
            Serial.print("Write CV Address:103, F21 Brake value:");
            Serial.println(F21Brake_Value);
        }
        break;


    }
}

void SetFunctionCallback(uint8_t nCnt, uint8_t fByte, uint8_t eByte)
{
    /*
     *    turns on and off engine decoder functions F0-F28 (F0 is sometimes called FL)  
     *    NOTE: setting requests transmitted directly to mobile engine decoder --- current state of engine functions is not stored by this program
     *    
     *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
     *    
     *    To set functions F0-F4 on (=1) or off (=0):
     *      
     *    BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
     *    BYTE2:  omitted
     *   
     *    To set functions F5-F8 on (=1) or off (=0):
     *   
     *    BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
     *    BYTE2:  omitted
     *   
     *    To set functions F9-F12 on (=1) or off (=0):
     *   
     *    BYTE1:  160 + F9*1 +F10*2 + F11*4 + F12*8
     *    BYTE2:  omitted
     *   
     *    To set functions F13-F20 on (=1) or off (=0):
     *   
     *    BYTE1: 222 
     *    BYTE2: F13*1 + F14*2 + F15*4 + F16*8 + F17*16 + F18*32 + F19*64 + F20*128
     *   
     *    To set functions F21-F28 on (=1) of off (=0):
     *   
     *    BYTE1: 223
     *    BYTE2: F21*1 + F22*2 + F23*4 + F24*8 + F25*16 + F26*32 + F27*64 + F28*128
     *   
     *    returns: NONE
     * 
     */
    if(nCnt==2)
    {                      // this is a request for functions FL,F1-F12  
        if ((fByte & 0xE0) == 128)
        { // 128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16

            Serial.println("F0-F5 Command");

            /*
                If F0 (Lights) is on
                Enable LED if DIRECTION = 1 (Forward)
                Enable LED if DIRECTION = 0 (Reverse)
                Enable LED and if SPEED>0, then brighten for 20s every 120s (firebox)            
            */
            // F0 Command process
            if(bitRead(fByte, 4))
            {
                bEnableLED = true;
                Serial.println("Enable LED");    
            }
            else
            {
                bEnableLED = false;    
                Serial.println("Disable LED");    
            }


            /*
                If F2 (Front Coupler) is
                On - Control servo to increase by 45 degrees
                Off– Control servo to return to 0 position
            */
            // F2 Command process
            if(bitRead(fByte, 1))
            { // Front Coupler
                nFrontPos += 45;
                if(nFrontPos > 180)
                    nFrontPos = 0;
                Front_CouplerServo.write(nFrontPos);        
                Serial.print("Front servo Pos:");    
                Serial.println(nFrontPos);    
            }
            else
            {
                nFrontPos = 0;
                Front_CouplerServo.write(nFrontPos);        
                Serial.print("Front servo Pos:");    
                Serial.println(nFrontPos);    
            }

            /*
                If F3 (Rear Coupler) is
                On - Control servo to increase by 45 degrees
                Off – Control servo to return to 0 position
            */
            
            // F3 Command process
            if(bitRead(fByte, 2))
            { // Rear Coupler
                nRearPos += 45;
                if(nRearPos > 180)
                    nRearPos = 0;
                //Rear_CouplerServo.write(nRearPos);        
                Serial.print("Rear servo Pos:");    
                Serial.println(nRearPos);    
            }
            else
            {
                nRearPos = 0;
                //Rear_CouplerServo.write(nRearPos);        
                Serial.print("Rear servo Pos:");    
                Serial.println(nRearPos);    
            }
        }
        else if ((fByte & 0xF0) == 176)
        {
            Serial.println("F6-F9 Command");

            /* If F9 Reverser_Value recieved ie < f 03 BYTE1 >, control servo to increase/decrease by 45 degrees */
            // F9 Command Check
            if(bitRead(fByte, 0))
            { // Reverser_Value recieved ie < f 03 BYTE1 >, control servo to increase/decrease by 45 degrees
                nReverPos += 45;
                if(nReverPos > 180)
                    nReverPos = 0;
                ReverserServo.write(nReverPos);        
                Serial.print("Reverser Pos:");    
                Serial.println(nReverPos);    
            }
            else
            {
                nReverPos -= 45;
                if(nReverPos < 0)
                    nReverPos = 180;
                ReverserServo.write(nReverPos);        
                Serial.print("Reverser Pos:");    
                Serial.println(nReverPos);    
            }
          
        }
    } 
    else 
    {                             // this is a request for functions F13-F28
        if (fByte == 222)
        { // F13 - F20
            Serial.println("F13-F20 Command");
            // F13 command check
            /* If F13 Momentum_Value received ie < f 03 222 BYTE2 >, map(F13,0,255,0,200), */
            if(bitRead(eByte, 0))
            {
                Serial.print("Momentum Started!");    
                bMomentumFlag = true;
            }    
            else
                bMomentumFlag = false;    
        }
        else if(fByte == 223)
        { // F21 - F28
            Serial.println("F21-F28 Command");
            // F21 command check
            /* If F21 Brake_Value received ie < f 03 223 BYTE2 >, map(F21,0,255,0,200), reduce motor speed using delay(Brake_Value) */
            if(bitRead(eByte, 0))
            {
                Serial.print("Breaking Started!");    
                bBreakFlag = true;    
            }
            else
                bBreakFlag = false;    
              
        }
    }      
}

