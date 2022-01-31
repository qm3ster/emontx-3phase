#ifndef rfmTxLib_h
#define rfmTxLib_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include <SPI.h>								                 // SPI bus for the RFM module
#include <util/crc16.h>                          // Checksum for JeeLib compatibility

enum rfband {RFM_433MHZ = 1, RFM_868MHZ, RFM_915MHZ }; // frequency band.

/*
 * "Public" functions for general use
 */
 
void rfm_init(void);

bool rfm_send(const byte *data, const byte size, const byte group, const byte node, const uint8_t rf_freq = RFM_433MHZ, const byte rf_power = 25, const int threshold = -97, const byte timeout = 15);

/* rf_power: 69CW range is min = 0 to max = 31. 12B range is 8 - 25 
   threshold: 69CW range is 0 = 0 dBm to -127 dBm   12B range is -73 dBm to -103 dBm
   timeout: the maximum hold-off time in ms. Special value: timeout = 0 means there is NO check for band occupancy
*/

#ifndef RFMSELPIN
 #define RFMSELPIN 10   // RFM select pin
#endif
#ifndef RFMIRQPIN
 #define RFMIRQPIN 2    // RFM interrupt pin
#endif

/*
 * Private functions
 */
 

#ifdef RFM69CW
void writeReg(uint8_t addr, uint8_t value);
uint8_t readReg(uint8_t addr);
void select();
void unselect();
void rfm_sleep(void);

#define REG_FIFO            0x00	
#define REG_OPMODE          0x01
#define MODE_SLEEP          0x00
#define MODE_TRANSMITTER    0x0C
#define MODE_RECEIVER       0x10
#define REG_DIOMAPPING1     0x25	
#define REG_IRQFLAGS1       0x27
#define MODE_READY          0x80
#define REG_IRQFLAGS2       0x28
#define IRQ2_FIFOFULL       0x80
#define IRQ2_FIFONOTEMPTY   0x40
#define IRQ2_PACKETSENT     0x08
#define IRQ2_FIFOOVERRUN    0x10
#define REG_PACKET_CONFIG2  0x3D
#define RESTART_RX          0x04
#define REG_RSSI_CONFIG     0x23
#define RSSI_START          0x01
#define RSSI_DONE           0x02
#define REG_RSSI_VALUE      0x24
#endif

#ifdef RFM12B
word rfm_write(word cmd);
#ifndef SDOPIN
  #define SDOPIN 12  // SDO pin
#endif
#define REG_RSSI          0x0080
#endif

#include <rfmTxLib.cpp>

#endif
