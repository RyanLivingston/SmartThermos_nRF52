
#include "ThermosService.h"

ThermosService::ThermosService(void) : BLEService(BLEUuid(0x1234)), 
  Set_Temp(BLEUuid(0x2001)),
  Actual_Temp(BLEUuid(0x2002)),
  Battery_Level(BLEUuid(0x2003)), 
  Op_State(BLEUuid(0x2004)),
  Op_Status(BLEUuid(0x2005)) 
{
}

err_t ThermosService::begin(void) {
  VERIFY_STATUS( BLEService::begin() );

  Serial.println("Starting Thermos Service");
  
  Set_Temp.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
  Set_Temp.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Set_Temp.setFixedLen(1);
  VERIFY_STATUS( Set_Temp.begin() );

  Op_State.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
  Op_State.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Op_State.setFixedLen(1);
  VERIFY_STATUS( Op_State.begin() );

  Actual_Temp.setProperties(CHR_PROPS_NOTIFY);
  Actual_Temp.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  Actual_Temp.setFixedLen(1);
  VERIFY_STATUS( Actual_Temp.begin() );

  Op_Status.setProperties(CHR_PROPS_NOTIFY);
  Op_Status.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  Op_Status.setFixedLen(1);
  VERIFY_STATUS( Op_Status.begin() );

  Battery_Level.setProperties(CHR_PROPS_NOTIFY);
  Battery_Level.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  Battery_Level.setFixedLen(1);
  VERIFY_STATUS( Battery_Level.begin() );


  return ERROR_NONE;
}

bool ThermosService::write(Characteristics ch, uint8_t data) {
  switch (ch) {
    case SET_TEMP:
      return Set_Temp.write8(data) > 0;
    case OP_STATE:
      return Op_State.write8(data) > 0;
    default:
      return false;
  }
}

bool ThermosService::notify(Characteristics ch, uint8_t data) {
  switch (ch) {
    case ACTUAL_TEMP:
      return Actual_Temp.notify8(data);
    case BATTERY_LEVEL:
      return Battery_Level.notify8(data);
    case OP_STATUS:
      return Op_Status.notify8(data);
    default:
      return false;
  }
}

  void ThermosService::setWriteCallback(Characteristics ch, BLECharacteristic::write_cb_t fp) {
    switch (ch) {
    case SET_TEMP:
      Set_Temp.setWriteCallback(fp);
      break;
    case OP_STATE:
      Op_State.setWriteCallback(fp);
      break;
  }
}
