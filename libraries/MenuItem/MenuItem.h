/*
  MenuItem.h - Library for creating menu items to interface with the TINAH board EEPROM menu system.
*/

#ifndef MenuItem_h
#define MenuItem_h

#include "Arduino.h"

class MenuItem
{
  public:
    MenuItem(String item, unsigned int* address );
    String getName();
    int getValue();
    void setValue( int value );

  private:
    String itemName;
    unsigned int* EEPROMAddress;
};

#endif