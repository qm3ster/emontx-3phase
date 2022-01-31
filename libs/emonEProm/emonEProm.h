/*
 * emonEProm.h
 *
 * Library to manage configuration data stored in EEPROM
 *
 * incorporating EEWL, by Fabrizio Pollastri, Torino, Italy <mxgbot@gmail.com>
 * (c) 2017 Fabrizio Pollastri
 * GNU General Public License
 *
 * V 1.0.0  9/7/2021
 */
 

#ifndef emonEProm_h
#define emonEProm_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <EEPROM.h>

//#define EEWL_DEBUG

struct block {
  int16_t  length;
  uint16_t signature;
};


unsigned int eepromSize(void);
void eepromFormat(void);
bool eepromValidate(uint16_t signature);
unsigned int eepromFindSignature(uint16_t signature);
unsigned int eepromRead(uint16_t signature, byte *dest);
unsigned int eepromWrite(uint16_t signature, byte *src, uint16_t length);
unsigned int eepromHide(uint16_t signature);
void eepromPrint(bool emonPi=false);
void printblock(int start, int size, bool emonPi=false); //private to library

void storeEValues(long E1, long E2, long E3, long E4, unsigned long pulses1);
void storeEValues(long E1, long E2, unsigned long pulses1, unsigned long pulses2);

bool recoverEValues(long *E1, long *E2, long *E3, long *E4, unsigned long *pulses); 
bool recoverEValues(long *M1, long *M2, unsigned long *pulses1, unsigned long *pulses2);

void zeroEValues(void);

struct EValues {long m1, m2, m3, m4, m5;};


/*
* Handler for Energy variables
*/

#define WHTHRESHOLD_1 200
#define WHTHRESHOLD_2 200
#define WHTHRESHOLD_3 200
#define WHTHRESHOLD_4 200
#define WHTHRESHOLD_5 200

struct EEWL {

  // control vars
  int blk_num;
  int blk_size;
  int blk_addr;
  int blk_mark = 0xfe;
  int start_addr;
  int end_addr;

  #ifdef EEWL_DEBUG
    unsigned long nwrites = 0;
    byte nsaves = 0;
  #endif
  
  // member functions

  // class constructor
  template <typename T> EEWL(T &data, int blk_num_, int start_addr_) 
  {

    // allocate and init control vars
    blk_size = sizeof(T) + 1;
    blk_num = blk_num_;
    start_addr = start_addr_;
    end_addr = start_addr + blk_num * blk_size;

    // search for a valid current data
    blk_addr = 0;
    for (int addr = start_addr; addr < end_addr; addr += blk_size) 
    {

      // save the first occurrence of a valid data marker
      if (EEPROM[addr] != 0xff) 
      {
        blk_addr = addr;

        // check for other valid data markers
        for (addr += blk_size; addr < end_addr; addr += blk_size) 
        {
          // if found, formatting is needed
          if (EEPROM[addr] != 0xff) 
          {
            fastFormat();
            break;
          }
        }
        break;
      }
    }
  }


  // format essential metadata of circular buffer, buffer is logically cleared.
  void fastFormat(void) 
  {

    // set all data status bytes as free
    for (int addr = start_addr; addr < end_addr; addr += blk_size) 
    {
      EEPROM[addr].update(0xff);

      // mark no valid data available
      blk_addr = 0;
    }
  }


  // read data from EEPROM 
  template <typename T> bool get(T &data) 
  {

    // if no valid data into eeprom, return a ram data null pointer
    if (!blk_addr) return false;

    // else copy data from eeprom to ram
    uint8_t *ptr = (uint8_t *) &data;
    for(int data_addr = blk_addr + 1; data_addr < blk_addr + blk_size; data_addr++)
      *ptr++ = EEPROM[data_addr];

    // return success to mark presence of valid data
    return true;
    
  }
 

  // write data to EEPROM
  template <typename T> void put(T &data) 
  {

    // if data already stored in buffer ...
    if (blk_addr) 
    {
      // save current block mark and set new mark as free
      blk_mark = EEPROM[blk_addr];
      EEPROM.update(blk_addr, 0xff);

      // point to next data block
      blk_addr += blk_size;
      if (blk_addr >= end_addr)
        blk_addr = start_addr;
    }
    else
      blk_addr = start_addr;

    // write data block data mark and data
    blk_mark <<= 1;
    blk_mark |= 1;
    blk_mark &= 0xff;
    if (blk_mark == 0xff)
      blk_mark = 0xfe;
    EEPROM.update(blk_addr, blk_mark);
    EEPROM.put(blk_addr + 1, data);
    #ifdef EEWL_DEBUG
      if (++nsaves > blk_num)
      {
        nwrites++;
        nsaves = 1;
      }
    #endif
  }


#ifdef EEWL_DEBUG

  void dump_control(void) 
  {

    Serial.print(F("blk_size:   "));
    Serial.println(blk_size);
    Serial.print(F("blk_num:    "));
    Serial.println(blk_num);
    Serial.print(F("blk_addr:   "));
    if (blk_addr<0x10)
      Serial.print(F("0"));
    Serial.println(blk_addr,HEX);
    Serial.print(F("blk_mark:   "));
    if (blk_mark<0x10)
      Serial.print(F("0"));
    Serial.println(blk_mark,HEX);
    Serial.print(F("start_addr: "));
    if (start_addr<0x10)
      Serial.print("0");
    Serial.println(start_addr,HEX);
    if (end_addr<0x10)
      Serial.print(F("0"));
    Serial.print(F("end_addr:   "));
    Serial.println(end_addr,HEX);
  }


  void dump_buffer(void) 
  {
    for (int addr = start_addr; addr < end_addr;) 
    {
      Serial.print(addr, HEX);
      Serial.print(F(": "));
      if (EEPROM[addr]<0x10)
        Serial.print(F("0"));
      Serial.print(EEPROM[addr], HEX);
      Serial.print(F("-"));
      int endaddr = addr + blk_size;
      for (++addr; addr < endaddr; addr++) 
      {
        if (EEPROM[addr]<0x10)
          Serial.print(F("0"));
        Serial.print(EEPROM[addr], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }

  void dump_stats(void)
  {
    Serial.print(nwrites);Serial.print(F(" write cycles + "));
    Serial.print(nsaves);Serial.println(F(" blocks saved"));
  }
  
  #endif

};

#endif

