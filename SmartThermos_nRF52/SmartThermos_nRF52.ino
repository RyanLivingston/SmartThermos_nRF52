///
/// @mainpage	SmartThermos_nRF52
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ryan Livingston
/// @author		Ryan Livingston
/// @date		3/20/19 12:41 PM
/// @version	<#version#>
///
/// @copyright	(c) Ryan Livingston, 2019
/// @copyright	CC = BY SA NC
///
/// @see		ReadMe.txt for references
///


///
/// @file		SmartThermos_nRF52.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ryan Livingston
/// @author		Ryan Livingston
/// @date		3/20/19 12:41 PM
/// @version	<#version#>
///
/// @copyright	(c) Ryan Livingston, 2019
/// @copyright	CC = BY SA NC
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

#include <bluefruit.h>
#include <Bluefruit_FileIO.h>
#include "ThermosService.h"

// Macros
#define BLE_CONFIG_FILENAME     "/config.txt"

#define INIT_SET_TEMP_VALUE     125
#define INIT_OP_STATE_VALUE     0

#define MOSFET_PIN              07
#define TEMP_SENSOR_PIN         A0
#define VBAT_PIN                A7
#define TIMER_VBAT              30000
#define TIMER_TEMP_SENSOR       5000

// Constants
const float ADC_MV_PER_LSB =   3000.0F / 4096.0F;
const float VBAT_DIVIDER =      0.71275837;
const float VBAT_DIVIDER_COMP = 1.403;

// Prototypes
void set_temp_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);
void op_state_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);
void fetchConfigData(uint8_t* set_temp_config, uint8_t* op_state_config);
void writeConfigData(uint8_t set_temp_config, uint8_t op_state_config);
void notifyTimerCallback(TimerHandle_t xTimerID);
uint8_t readVBAT(int * vbat_mv = nullptr);
uint8_t readTempSensor(void);

void println(const __FlashStringHelper * str);
void print(const __FlashStringHelper * str);
void print(uint32_t str);
void println(uint32_t str);

// Global Variables
SoftwareTimer notifyBatteryTimer;
SoftwareTimer notifyTempSensorTimer;
TimerHandle_t batteryTimerHandle;
TimerHandle_t tempSensorTimerHandle;

ThermosService bleThermos;

uint8_t setTemp;
uint8_t opState = 0;
bool opStatus = false;
volatile uint8_t actualTemp;

File bleConfigFile(InternalFS);

void setup() {
    
    // Begin serial at 115200 baudrate
    Serial.begin(115200);
    
    // Delay setup
    delay(7500);
    
    // Set bandwidth and begin BLE stack
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    
    // Set Tx power and name
    Bluefruit.setTxPower(4);
    Bluefruit.setName("Smart Thermos");
    
    // Set connection callbacks
    Bluefruit.setConnectCallback(connect_callback);
    Bluefruit.setDisconnectCallback(disconnect_callback);
    
    // Set thermos service callbacks and being service
    bleThermos.setWriteCallback(OP_STATE, op_state_write_callback);
    bleThermos.setWriteCallback(SET_TEMP, set_temp_write_callback);
    bleThermos.begin();
    
    // Fetch saved config data from FS
    fetchConfigData(&setTemp, &opState);
    
    // Write saved config to BLE characteristics
    bleThermos.write(SET_TEMP, setTemp);
    bleThermos.write(OP_STATE, opState);
    
    // Configure BLE advertising
    startAdv();
    
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    
    // Begin timed callback for writing battery characterisitc
    notifyBatteryTimer.begin(TIMER_VBAT, notifyTimerCallback);
    notifyTempSensorTimer.begin(TIMER_TEMP_SENSOR, notifyTimerCallback);
    
    batteryTimerHandle = notifyBatteryTimer.getHandle();
    tempSensorTimerHandle = notifyTempSensorTimer.getHandle();
    
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW);
    
}

void loop() {
    
    actualTemp = readTempSensor();
    
    if(opState) {
        
        bleThermos.notify(OP_STATUS, opStatus);

        if(!opStatus && actualTemp < (setTemp - 2)) {
            
            opStatus = true;
            digitalWrite(MOSFET_PIN, opStatus);
            ledOn(LED_RED);
        }
        if(opStatus && actualTemp > (setTemp + 2)) {
            
            opStatus = false;
            digitalWrite(MOSFET_PIN, opStatus);
            ledOff(LED_RED);
        }
    }
    else{
        opStatus = false;
        digitalWrite(MOSFET_PIN, opStatus);
        ledOff(LED_RED);
    }
    
    delay(250);
}

void fetchConfigData(uint8_t* set_temp_config, uint8_t* op_state_config) {
    InternalFS.begin();
    
    bleConfigFile.open(BLE_CONFIG_FILENAME, FILE_READ);
    if(bleConfigFile) {
        println(F("Reading Config File..."));
        
        bleConfigFile.read(set_temp_config, sizeof(uint8_t));
        bleConfigFile.read(op_state_config, sizeof(uint8_t));
        
        print(F("Saved Set Temp: "));
        println(*set_temp_config);
        print(F("Saved Operation State: "));
        println(*op_state_config);
        
//      InternalFS.format(true);
    }
    else {
        println(F("Created Config File!"));
        bleConfigFile.open(BLE_CONFIG_FILENAME, FILE_WRITE);
        bleConfigFile.write(INIT_SET_TEMP_VALUE);
        bleConfigFile.write((uint8_t)INIT_OP_STATE_VALUE);
        *set_temp_config = INIT_SET_TEMP_VALUE;
        *op_state_config = INIT_OP_STATE_VALUE;
    }
    
    bleConfigFile.close();
}

void writeConfigData(uint8_t set_temp_config, uint8_t op_state_config) {
    
    bleConfigFile.open(BLE_CONFIG_FILENAME, FILE_WRITE);
    if(bleConfigFile) {
        bleConfigFile.seek(0);
        bleConfigFile.write(set_temp_config);
        bleConfigFile.write(op_state_config);
    }
    bleConfigFile.close();
}

void notifyTimerCallback(TimerHandle_t xTimerID) {
    
    if(xTimerID == batteryTimerHandle) {
        int vbat_mv;
        uint8_t vbatPercent = readVBAT(&vbat_mv);
        
        print(F("Update vBat: "));
        print(vbatPercent);
        print(F("% ("));
        print(vbat_mv);
        println(F("mV)"));
        
        if(Bluefruit.connected())
            bleThermos.notify(BATTERY_LEVEL, vbatPercent);
    }
    else if (xTimerID == tempSensorTimerHandle) {
        print(F("Update temp: "));
        print(actualTemp);
        println(F("F"));
        
        if(Bluefruit.connected())
            bleThermos.notify(ACTUAL_TEMP, actualTemp);
    }
}

void set_temp_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset) {
    setTemp = *data;
    writeConfigData(setTemp, opState);
    
    print(F("Set Temp: "));
    println(setTemp);
}

void op_state_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset) {
    opState = *data;
    writeConfigData(setTemp, opState);
    
    print(F("Operational State: "));
    println(opState);
}

void connect_callback(uint16_t conn_handle) {
    println(F("\nConnected"));
    
    notifyBatteryTimer.start();
    notifyTempSensorTimer.start();
    
    delay(2000);
    bleThermos.notify(BATTERY_LEVEL, readVBAT());
    bleThermos.notify(ACTUAL_TEMP, actualTemp);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;
    
    println(F("\nDisconnected"));
    
    notifyBatteryTimer.stop();
    notifyTempSensorTimer.stop();
}

void startAdv(void)
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleThermos);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

uint8_t readVBAT(int * vbat_mv) {
    int vbat;
    int* vbatPtr = &vbat;
    uint8_t vbatPercent;
    
    if(vbat_mv != nullptr)
        vbatPtr = vbat_mv;
    
    *vbatPtr = analogRead(VBAT_PIN) * VBAT_DIVIDER_COMP * ADC_MV_PER_LSB;
    
    if(*vbatPtr > 4100)
        vbatPercent = 100;
    else if (*vbatPtr > 4000)
        vbatPercent = 90;
    else if (*vbatPtr > 3750)
        vbatPercent = 50;
    else
        vbatPercent = 10;
    
    return vbatPercent;
}

uint8_t readTempSensor(void) {
    float temp_mv;
    float temp_C;
    float temp_F;
    
    temp_mv = analogRead(TEMP_SENSOR_PIN) * ADC_MV_PER_LSB;
    temp_C = (temp_mv - 500) / 10;
    temp_F = (temp_C * (9/5)) + 32;
    
    if(temp_F > 65)
        return 68 + ((temp_F - 68) * 4.5);
    else
        return 65;
}

void print(const __FlashStringHelper * str){
    Serial.print(str);
}
void print(uint32_t str){
    Serial.print(str);
}

void println(const __FlashStringHelper * str){
    Serial.println(str);
}
void println(uint32_t str){
    Serial.println(str);
}
