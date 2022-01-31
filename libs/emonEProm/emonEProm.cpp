/*
 * emonEProm.cpp
 *
 * Library to manage configuration data stored in EEPROM
 *
 * V 1.0.0  9/7/2021
 */
 
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <emonEProm.h>


char moniker[3] = {'O', 'E', 'M'};
struct EValues mem = {0, 0, 0, 0, 0};

unsigned int eepromSize(void)
{
/*
 * Get EEPROM size (duplicates EEPROM.length() )
 * Returns total size of EEPROM (E2END is from Arduino header file)
 */
 
  return E2END + 1;
}


void eepromFormat(void)
{
/*
 * Unconditionally erases all EEPROM contents and
 * writes OEM signature.
 * (Although done AFTER the upper ¾ is formatted by the class constructor,
 *  this does exactly the same - so no harm is done.)
 */
 
  for (byte j=0; j<sizeof(moniker); j++)
    EEPROM.update(j, moniker[j]);
  for (unsigned int j=sizeof(moniker); j<=E2END; j++)
    EEPROM.update(j, 0xFF);    
}

  
bool eepromValidate(uint16_t signature)
{
/*
 * Checks for the existence of both the "OEM" signature and the given one
 * Returns TRUE if found - or if given signature = 0.
 */
 
  unsigned int offset = sizeof(moniker);
  block b; 
  
  for (byte j=0; j<sizeof(moniker); j++)
    if (EEPROM.read(j) != moniker[j])
      return false;
  
  if (signature)
  {
    EEPROM.get(offset, b);
    if (b.signature == signature)
      return true;
    else 
    if (b.signature == 0xFFFF && (uint16_t)b.length == 0xFFFF)
      return false;
  }
  else 
    return true;
  return false;
}


unsigned int eepromRead(uint16_t signature, byte *dest)
{
/*
 * Verifies the block's signature and copies the EEPROM data to the struct "dest", which MUST be big enough. 
 * Returns zero if not found, or EEPROM block has been hidden.
 */
 
  unsigned int offset = sizeof(moniker);
  if (eepromValidate(signature))
  {
    block b;
    EEPROM.get(offset, b);
    if (b.length < 0)
      return 0;
    unsigned int end = offset + b.length;
    offset += sizeof(b);
    
    do {
      *dest++ = EEPROM.read(offset++);
    } while (offset < end);
    return b.length-sizeof(b);  
  }
  return 0;
}


unsigned int eepromWrite(uint16_t signature, byte *src, uint16_t length)
{
/*
 * Writes unconditionally to unrecognised EEPROM, else overwrites the block identified by signature,
 * by copying the struct "src" data of length 'length' bytes to the EEPROM. 
 * Returns the number of bytes written (=length + block header), or zero if unable to write. 
 */

  unsigned int offset = sizeof(moniker);
  if (!eepromValidate(signature))                               // EEPROM format unrecognised or wrong signature - use it
    eepromFormat();
  
  if (length > (E2END - (((E2END>>2) * 3) / (sizeof(mem)+1))*(sizeof(mem)+1) - 1))
    return 0;
    
  // Write the block
  block b;
  b.length = length + sizeof(b);
  unsigned int end = offset + b.length;
  b.signature = signature;
  EEPROM.put(offset, b);
  offset += sizeof(b);
  do {
    EEPROM.update(offset++, *src++);
  } while (offset < end && offset < E2END);
  return b.length;
}


unsigned int eepromHide(uint16_t signature)
{
/*
 * Marks the block as hidden. 
 * Returns the number of data bytes in the block (=length), or zero if the signature does not match. 
 */

  block b;
  unsigned int offset = sizeof(moniker);
  if (!eepromValidate(signature))                               // EEPROM format unrecognised or invalid signature
    return 0;

  EEPROM.get(offset, b);
  if (b.length < 0)
    return 0;                                                   // Already hidden!
  b.length = -b.length;
  EEPROM.put(offset, b);
  return -b.length;
}


void eepromPrint(bool emonPi)
{
/*
 * Print to Serial the entire used contents of EEPROM.
 * If the format is unrecognised, the complete contents of EEPROM is printed, else the data block only is printed.
 * 
 * Assumes Serial printing is available via host sketch.
 */
  block b;
  unsigned int offset;
  if (!eepromValidate(0))                                       // EEPROM format not recognised - print raw.
  {
    if (emonPi)
      Serial.print(F("|"));
    Serial.print(F("Size: "));Serial.print(eepromSize());Serial.println(F(" bytes, not formatted to OEM Standard"));
    printblock(0, eepromSize(), emonPi);
    Serial.print(F("\n"));
    if (emonPi)
      Serial.print(F("|"));
  }
  else                                                          // OEM-formatted - print the block.
  {
    offset = sizeof(moniker);

    EEPROM.get(offset, b);
    if (b.signature == 0xFFFF)
    {
      if (emonPi)
        Serial.print(F("|"));
      Serial.println(F("No data block found - EEPROM appears to be formatted to OEM Standard but unused."));
      return;
    }
    if (emonPi)
      Serial.print(F("|"));
    Serial.print(F(" Signature: "));Serial.print(b.signature);
    int size = abs(b.length) - sizeof(block);
    Serial.print(F(", Data size: "));Serial.print(size);Serial.print(F(" bytes"));
    if (b.length < 0)
        Serial.print(F(" (hidden)."));
    Serial.println();  
    printblock(offset+sizeof(block), size, emonPi);
    Serial.print(F("\n"));
    if (emonPi)
      Serial.print(F("|"));
  }
}

void printblock(int start, int size, bool emonPi)
{
  // private to the library
  if (emonPi)
    Serial.print(F("|"));
  for (int j=0; j<size;)
  {
    if (EEPROM[start + j] < 0x10)
      Serial.print('0');
    Serial.print(EEPROM[start + j++],16);Serial.print(" ");

    if (j % 8 == 0)
      Serial.print(" ");

    if (j % 16 == 0)
    {
      Serial.print(F("\n"));
      if (emonPi)
        Serial.print(F("|"));
    }
  }
}
        
        
/*
*  Retain Energy variables     
*/

EEWL EVmem(mem, ((E2END>>2) * 3) / (sizeof(mem)+1), E2END - (((E2END>>2) * 3) / (sizeof(mem)+1))*(sizeof(mem)+1));


bool recoverEValues(long *M1, long *M2, long *M3, long *M4, unsigned long *pulses1)
{
  bool success = EVmem.get(mem);
  if (success)
  {
    if (M1)
      *M1 = mem.m1;
    if (M2)
      *M2 = mem.m2;
    if (M3)
      *M3 = mem.m3;
    if (M4)
      *M4 = mem.m4;
    if (pulses1)
      *pulses1 = mem.m5;
  }
  return success;
}
 
bool recoverEValues(long *M1, long *M2, unsigned long *pulses1, unsigned long *pulses2)
{
  bool success = EVmem.get(mem);
  if (success)
  {
    if (M1)
      *M1 = mem.m1;
    if (M2)
      *M2 = mem.m2;
    if (pulses1)
      *pulses1 = mem.m3;
    if (pulses2)
      *pulses2 = mem.m4;
  }
  return success;
}

void storeEValues(long E1, long E2, long E3, long E4, unsigned long pulses1)
{
#ifdef EEWL_DEBUG 
  Serial.print(F("E1 diff="));Serial.print(abs(E1-mem.m1));
  Serial.print(F("  E2 diff="));Serial.print(abs(E2-mem.m2));
  Serial.print(F("  E3 diff="));Serial.print(abs(E3-mem.m3));
  Serial.print(F("  E4 diff="));Serial.print(abs(E4-mem.m4));
  Serial.print(F("  P1 diff="));Serial.println(abs(pulses1-mem.m5));
#endif

  if ((abs(E1-mem.m1) >= WHTHRESHOLD_1)
    || (abs(E2-mem.m2) >= WHTHRESHOLD_2)
    || (abs(E3-mem.m3) >= WHTHRESHOLD_3)
    || (abs(E4-mem.m4) >= WHTHRESHOLD_4)
    || (((long)pulses1-mem.m5) >= WHTHRESHOLD_5))
    {
      mem.m1 = E1;
      mem.m2 = E2;
      mem.m3 = E3;
      mem.m4 = E4;
      mem.m5 = pulses1;
      #ifdef EEWL_DEBUG
        Serial.println(F("Writing..."));
      #endif
      EVmem.put( mem );
      #ifdef EEWL_DEBUG
        EVmem.dump_stats(); 
      #endif
    }
}

void storeEValues(long E1, long E2, unsigned long pulses1, unsigned long pulses2)
{
#ifdef EEWL_DEBUG 
  Serial.print(F("E1 diff="));Serial.print(abs(E1-mem.m1));
  Serial.print(F("  E2 diff="));Serial.print(abs(E2-mem.m2));
  Serial.print(F("  P1 diff="));Serial.print(abs(pulses1-mem.m3));
  Serial.print(F("  P2 diff="));Serial.println(abs(pulses2-mem.m4));
#endif

  if ((abs(E1-mem.m1) >= WHTHRESHOLD_1)
    || (abs(E2-mem.m2) >= WHTHRESHOLD_2)
    || (abs((long)pulses1-mem.m3) >= WHTHRESHOLD_3)
    || (abs((long)pulses2-mem.m4) >= WHTHRESHOLD_4))
    {
      mem.m1 = E1;
      mem.m2 = E2;
      mem.m3 = pulses1;
      mem.m4 = pulses2;
      #ifdef EEWL_DEBUG
        Serial.println(F("Writing..."));
      #endif
      EVmem.put( mem );
      #ifdef EEWL_DEBUG
        EVmem.dump_stats(); 
      #endif
    }
}

void zeroEValues(void)
{
#ifdef EEWL_DEBUG 
  Serial.print(F("All stored energy values set to zero"));
#endif
  EVmem.fastFormat();
}



