/*
================================================================================================================

  Routines for Transmit-only, using the "Classic" JeeLib packet format.
  (To use the "Native" Hope RFM69 format, use rfm69nTxLib

  Derived from JeeLib and work by Martin Roberts by Robert Wall
  
  -----------------------------------------
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
  -----------------------------------------

  
  Change Log:
  Version 1.0  27/6/2020 First release
  Version 2.0  10/3/2021 Frequency setting moved into transmit function, ability to ignore 'busy channel detection' (timeout = 0) added.


================================================================================================================
These definitions are required in the main sketch if different:

#define RFMSELPIN 10   // RFM select pin
#define RFMIRQPIN 2    // RFM interrupt pin

================================================================================================================


Functions:

void rfm_init(void);   
    Initialises the radio module to JeeLib protocol standards
                       
bool rfm_send(const byte *data, const byte size, const byte group, const byte node, const uint8_t rf_freq, const byte rf_power, const int threshold, const byte timeout) 
    Transmits the data
        data      - byte stream to be transmitted
        size      - length of the data
        group     - transmission group: 210 for OEM
        node      - the unique ID of this node (1 - 630)
        rf_freq   - must be either 1 (=433 MHz), 2 (=868 MHz) or 3 (=915 MHz)
        rf_power  - 0 - 31 = -18 dBm (min value) - +13 dBm (max value). RFM12B equivalent: 25 (+7 dBm)
        threshold - the RSSI level below which the radio channel is considered clear (suggested value: -97)
        timeout   - the maximum time in milliseconds that the function will wait for the channel to become clear,
                      after which it transmits regardless. Suggested value: 10
        returns:    true if no timeout occurred, otherwise false.


*/


#include <rfmTxLib.h>


/*
Interface for the RFM69CW Radio Module in "RFM69 Classic" mode, with Channel Busy detection.
*/

#ifdef RFM69CW


void rfm_init(void)
{	
	// Set up to drive the Radio Module
	digitalWrite(RFMSELPIN, HIGH);
	pinMode(RFMSELPIN, OUTPUT);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(0);
	SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
	
	// Initialise RFM69CW
	do 
		writeReg(0x2F, 0xAA); // RegSyncValue1
	while (readReg(0x2F) != 0xAA) ;
	do
	  writeReg(0x2F, 0x55); 
	while (readReg(0x2F) != 0x55);
	
	writeReg(0x01, 0x04); // RegOpMode: RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY
	writeReg(0x02, 0x00); // RegDataModul: RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 = no shaping
	writeReg(0x03, 0x02); // RegBitrateMsb  ~49.23k BPS
	writeReg(0x04, 0x8A); // RegBitrateLsb
	writeReg(0x05, 0x05); // RegFdevMsb: ~90 kHz 
	writeReg(0x06, 0xC3); // RegFdevLsb
//	writeReg(0x0B, 0x20); // RegAfcCtrl:
//	writeReg(0x11, 0x99); // Transmit Power - set at transmit time
	writeReg(0x1E, 0x2C); //
	writeReg(0x25, 0x80); // RegDioMapping1: DIO0 is used as IRQ 
	writeReg(0x26, 0x03); // RegDioMapping2: ClkOut off
	writeReg(0x28, 0x00); // RegIrqFlags2: FifoOverrun

	// RegPreamble (0x2c, 0x2d): default 0x0003
	writeReg(0x2E, 0x88); // RegSyncConfig: SyncOn | FifoFillCondition | SyncSize = 2 bytes | SyncTol = 0
	writeReg(0x2F, 0x2D); // RegSyncValue1: Same as JeeLib
	writeReg(0x37, 0x00); // RegPacketConfig1: PacketFormat=fixed | !DcFree | !CrcOn | !CrcAutoClearOff | !AddressFiltering >> 0x00
}


// transmit data via the RFM69CW

bool rfm_send(const byte *data, const byte size, const byte group, const byte node, const uint8_t rf_freq, const byte rf_power, const int threshold, const byte timeout) 
{
  // rf_power:  0 - 31 = -18 dBm (min value) - +13 dBm (max value). RFM12B equivalent: 25 (+7 dBm)  
  // timeout:  set to zero to inhibit channel occupancy check.
  
	if (rf_freq == RFM_868MHZ)
  {
		writeReg(0x07, 0xD9); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0xD90000 = 868.00 MHz as used by JeeLib  
		writeReg(0x08, 0x00); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
	}
  else if (rf_freq == RFM_915MHZ) // JeeLib uses 912.00 MHz
  {	
		writeReg(0x07, 0xE4); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0xE40000 = 912.00 MHz as used by JeeLib 
		writeReg(0x08, 0x00); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
  }
	else // default to 433 MHz band
	{
    writeReg(0x07, 0x6C); // RegFrfMsb: Frf = Rf Freq / 61.03515625 Hz = 0x6C8000 = 434.00 MHz as used by JeeLib 
		writeReg(0x08, 0x80); // RegFrfMid
		writeReg(0x09, 0x00); // RegFrfLsb
	}


  unsigned long t_start = millis();
  bool success = true;                                     // return false if timed out, else true

  if (timeout)
  {
    success = false;
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | MODE_RECEIVER);		// Receive mode - listen for channel is busy
    while ((millis()-t_start)<(unsigned long)timeout)
    {
      while((readReg(REG_IRQFLAGS1) & MODE_READY) == 0)      
        ;
      writeReg(REG_RSSI_CONFIG, RSSI_START);
      while((readReg(REG_RSSI_CONFIG) & RSSI_DONE) == 0x00)
        ; 
      
      if (readReg(REG_RSSI_VALUE) > (threshold * -2))         // because REG_RSSI_VALUE is upside down!
      {
        success = true;
        break;                                                // Nothing heard - go ahead and transmit
      }
      writeReg(REG_PACKET_CONFIG2, (readReg(REG_PACKET_CONFIG2) & 0xFB) | RESTART_RX);  // Restart the receiver
    }
    // We have waited long enough - go ahead and transmit anyway
  }

  writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | MODE_SLEEP);	      	// Sleep

	while (readReg(REG_IRQFLAGS2) & (IRQ2_FIFONOTEMPTY | IRQ2_FIFOOVERRUN))		// Flush FIFO
        readReg(REG_FIFO);
	writeReg(0x30, group);                                    // RegSyncValue2

  writeReg(REG_DIOMAPPING1, 0x00); 										      // PacketSent
		
	volatile uint8_t txstate = 0;
	byte i = 0;
	uint16_t crc = _crc16_update(~0, group);	

	while(txstate < 5)
	{
		if ((readReg(REG_IRQFLAGS2) & IRQ2_FIFOFULL) == 0)			// FIFO !full
		{
			uint8_t next;
			switch(txstate)
			{
			  case 0: next=node & 0x1F; txstate++; break;    		  // Bits: CTL, DST, ACK, Node ID(5)
			  case 1: next=size; txstate++; break;				   	    // No. of payload bytes
			  case 2: next=data[i++]; if(i==size) txstate++; break;
			  case 3: next=(byte)crc; txstate++; break;
			  case 4: next=(byte)(crc>>8); txstate++; break;
			}
			if(txstate<4) crc = _crc16_update(crc, next);
			writeReg(REG_FIFO, next);								              // RegFifo(next);
		}
	}
  while (txstate++ < 17) 
    writeReg(REG_FIFO, 0xAA);                               // Dummy bytes - if total FIFO < 12 bytes, locks up                   
    
    // Transmit buffer is now filled, transmit it
    // rf_power: RegPaLevel = 0x9F = PA0 on, +13 dBm (max value) Min value is 0x80 (-18 dBm), RFM12B equivalent: +7 dBm = 0x99  

	writeReg(0x11, (rf_power & 0x1F)|0x80); 
	writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | MODE_TRANSMITTER);		// Transmit mode - 60 Bytes max payload
   
  rfm_sleep();
  return success;
    
}

void rfm_sleep(void)
{   
    // Put into sleep mode when buffer is empty
	while (!(readReg(REG_IRQFLAGS2) & IRQ2_PACKETSENT))				// wait for transmission to complete (not present in JeeLib) 
	    delay(1);	
	writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | 0x01); 		// Standby Mode	
} 



void writeReg(uint8_t addr, uint8_t value)
{
	select();
	SPI.transfer(addr | 0x80);
	SPI.transfer(value);
	unselect();
}

uint8_t readReg(uint8_t addr)
{
	select();
	SPI.transfer(addr & 0x7F);
	uint8_t regval = SPI.transfer(0);
	unselect();
	return regval;
}

// select the transceiver
void select() 
{
	noInterrupts();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
	digitalWrite(RFMSELPIN, LOW);
}

// UNselect the transceiver chip
void unselect() {
	digitalWrite(RFMSELPIN, HIGH);
	interrupts();
}
#endif


/*********************************************************************************
Interface for the RFM12B Radio Module
*********************************************************************************/

#ifdef RFM12B

void rfm_init(void)
{	
	// Set up to drive the Radio Module
	pinMode (RFMSELPIN, OUTPUT);
	digitalWrite(RFMSELPIN,HIGH);
	// start the SPI library:
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(0);
	SPI.setClockDivider(SPI_CLOCK_DIV8);
	// initialise RFM12
	delay(200); // wait for RFM12 POR
	rfm_write(0x0000); // clear SPI
	rfm_write(0x8208); // Turn on crystal,!PA
	rfm_write(0xC606); // approx 49.2 Kbps, as used by emonTx
	//rfm_write(0xC657); // approx 3.918 Kbps, better for long range
	rfm_write(0xCC77); // PLL 
	rfm_write(0xC2AC); // AL,!ml,DIG,DQD4 
	rfm_write(0xCA83); // FIFO8,2-SYNC,!ff,DR 
	rfm_write(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN 
	rfm_write(0xE000); // wake up timer - not used 
	rfm_write(0xC800); // low duty cycle - not used 
	rfm_write(0xC000); // 1.0MHz,2.2V 
}


// transmit data via the RFM12
bool rfm_send(const byte *data, const byte size, const byte group, const byte node, const uint8_t rf_freq, const byte rf_power, const int threshold, const byte timeout)
{
	byte i=0,next,txstate=0;
	word crc=~0;
  byte pwr = rf_power & 0x1F;
  byte rssi = -threshold;
  
  unsigned long t_start = millis();
  bool success = true;                                     // return false if timed out, else true

  if (rssi < 73)
    rssi = 73;
  if (rssi > 103)
    rssi = 103;
  rssi = (103 - rssi) / 6.0; // scale as per RFM69

  if (pwr > 25)
    pwr = 25;
  if (pwr < 8)
    pwr = 8;
  pwr = (25 - pwr) / 2.4; // scale as per RFM69
  
  if (rf_freq == RFM_868MHZ)
  {	  
    rfm_write(0x80E7); // EL (ena dreg), EF (ena RX FIFO), 868 MHz, 12.0pF 
	  rfm_write(0xA640); // 868.00 MHz as used JeeLib 
	}
  else if (rf_freq == RFM_915MHZ) // JeeLib uses 912.00 MHz
  {	
	  rfm_write(0x80F7); // EL (ena dreg), EF (ena RX FIFO), 915 MHz, 12.0pF 
	  rfm_write(0xA640); // 912.00 MHz as used JeeLib 	
  }
	else // default to 433 MHz band @ 434 MHz
	{
	  rfm_write(0x80D7); // EL (ena dreg), EF (ena RX FIFO), 433 MHz, 12.0pF 
	  rfm_write(0xA640); // 434.00 MHz as used JeeLib 
	}  
	rfm_write(0x94A0 | rssi); // VDI,FAST,134kHz,0dBm,threshold -103dBm+rssi
	rfm_write(0xCE00 | group); // SYNC=2DD2 => group 210
	rfm_write(0x9850 | pwr); // !mp,90kHz,MAX OUT - 0x9850 = 0dB, 0x9857 = -17.5 dB in steps of 2.5 dB
  
  if (timeout)
  {
    success = false;
    rfm_write(0x8298); // er,es,ex - Receive mode - sniff for channel is busy
    while ((millis()-t_start)<(unsigned long)timeout)
    {
      if ((rfm_write(0x00) & REG_RSSI) == 0)                  // REG_RSSI_VALUE < threshold
      {
        success = true;
        break;                                                // Nothing heard - go ahead and transmit
      }
    }                                                         // We have waited long enough - go ahead and transmit anyway
  }
  
	rfm_write(0x8228); // OPEN PA
	rfm_write(0x8238);

	digitalWrite(RFMSELPIN,LOW);
	SPI.transfer(0xb8); // tx register write command
  
	while(txstate<13)
	{
		while(digitalRead(SDOPIN)==0); // wait for SDO to go high
		switch(txstate)
		{
			case 0:
			case 1:
			case 2: next=0xaa; txstate++; break;
			case 3: next=0x2d; txstate++; break;
			case 4: next=group; txstate++; break;
			case 5: next=node; txstate++; break; // node ID
			case 6: next=size; txstate++; break; // length
			case 7: next=data[i++]; if(i==size) txstate++; break;
			case 8: next=(byte)crc; txstate++; break;
			case 9: next=(byte)(crc>>8); txstate++; break;
			case 10:
			case 11:
			case 12: next=0xaa; txstate++; break; // dummy bytes (if <3 CRC gets corrupted sometimes)
		}
		if((txstate>4)&&(txstate<9)) crc = _crc16_update(crc, next);
		SPI.transfer(next);
	}
	digitalWrite(RFMSELPIN,HIGH);
	rfm_write( 0x8208 ); // CLOSE PA
	rfm_write( 0x8200 ); // enter sleep
  return success;
}


// Write a command to or read from the RFM12
word rfm_write(word cmd)
{
	word result;
  
	digitalWrite(RFMSELPIN,LOW);
	result=(SPI.transfer(cmd>>8)<<8) | SPI.transfer(cmd & 0xff);
	digitalWrite(RFMSELPIN,HIGH);
	return result;
}

#endif