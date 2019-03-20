#ifndef THERMOSSERVICE_H_
#define THERMOSSERVICE_H_

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

//#define UUID_SVC_THERMOS        0x1234
//#define UUID_CHR_SET_TEMP       0x2001
//#define UUID_CHR_ACTUAL_TEMP    0x2002
//#define UUID_CHR_BATTERY        0x2003
//#define UUID_CHR_OP_STATE       0x2004
//#define UUID_CHR_OP_STATUS      0x2005

enum Characteristics{
  SET_TEMP,
  ACTUAL_TEMP,
  BATTERY_LEVEL,
  OP_STATUS,
  OP_STATE
};

class ThermosService : public BLEService {
protected:
  // Notify Characteristics
  BLECharacteristic Actual_Temp;
  BLECharacteristic Battery_Level;
  BLECharacteristic Op_Status;

  // Read & Write Characteristic
  BLECharacteristic Set_Temp;
  BLECharacteristic Op_State;

public:
  ThermosService(void);

  virtual err_t begin(void);

  bool write(Characteristics ch, uint8_t data);
  bool notify(Characteristics ch, uint8_t data);

  void setWriteCallback(Characteristics ch, BLECharacteristic::write_cb_t fp);

};


#endif
