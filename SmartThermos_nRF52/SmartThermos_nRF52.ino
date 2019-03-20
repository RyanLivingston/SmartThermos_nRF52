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
#include "ThermosService.h"

// Macros
#define INIT_SET_TEMP_VALUE     125
#define INIT_OP_STATE_VALUE     0

// Prototypes
void set_temp_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);
void op_state_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void startAdv(void);

// Global Variables
ThermosService bleThermos;
int setTemp;
bool opState;


void setup() {
    
    Serial.begin(115200);
    
    Bluefruit.autoConnLed(true);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    
    Bluefruit.setTxPower(4);
    Bluefruit.setName("Smart Thermos");
    
    Bluefruit.setConnectCallback(connect_callback);
    Bluefruit.setDisconnectCallback(disconnect_callback);
    
    bleThermos.setWriteCallback(OP_STATE, op_state_write_callback);
    bleThermos.setWriteCallback(SET_TEMP, set_temp_write_callback);
    bleThermos.begin();
    
    startAdv();
    
}

void loop() {
    
    
    
    
}

void set_temp_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset) {
    setTemp = int(*data);
    
    Serial.print("Set Temp: ");
    Serial.println(setTemp);
}

void op_state_write_callback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset) {
    opState = int(*data);
    
    Serial.print("Operational State: ");
    Serial.println(opState);
}

void connect_callback(uint16_t conn_handle) {
    Serial.println("Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;
    
    Serial.println();
    Serial.println("Disconnected");
}

void startAdv(void)
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleThermos);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);
}
