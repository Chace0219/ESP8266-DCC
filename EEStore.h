/**********************************************************************

EEStore.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#ifndef EEStore_h
#define EEStore_h

#define  EESTORE_ID "DCC++"

struct EEStoreData{
  char id[sizeof(EESTORE_ID)];
  unsigned short bLongAddrEnable;
  int nCABAddr;
  int nTurnouts;
  int nSensors;  
  int nOutputs;
  int nShortCAB;  
  int nLongCAB;  
};

struct EEStore{
  static EEStore *eeStore;
  EEStoreData data;
  static int eeAddress;
  static void init();
  static void reset();
  static int pointer();
  static void advance(int);
  static void store();
  static void clear();
};
  
#endif

