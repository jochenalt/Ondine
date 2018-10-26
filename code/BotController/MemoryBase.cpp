/*
 * Memory.cpp
 *
 * Created: 04.04.2013 15:41:22
 *  Author: JochenAlt
 */ 

#include "Arduino.h"
#include "EEPROM.h"
#include "Util.h"
#include "MemoryBase.h"

#define EEMEM_MAGICNUMBER 1587 					// thats my birthday, used to check if eeprom has been initialized
void* magicMemoryNumberAddress = (void*)0;  	// my birthday is stored at this address
void* memoryAddress = (void*)sizeof(int16_t);	// address of user-defined EEPROM area

MemoryBase::MemoryBase (void *pMem_RAM, size_t pLen) {
	somethingToSave = false;
	memRAM = pMem_RAM;
	len = pLen;
	saveJustHappened = false;
}

boolean MemoryBase::setup() {
	if (!isEEPROMInitialized()) {

		// hopefully the defaults have been initialized in the constructor of the derived
		save();
		
		// write magic number in the eeprom to indicate initialization
		markEEPROMInitialized();
		return true;
		
	} else
		read();
	return false;		
}

void MemoryBase::read() {
	eeprom_read_block(memRAM, memoryAddress,len);
}

void MemoryBase::loop(uint32_t now) {
	if (somethingToSave) {
		if (memTimer.isDue_ms(writeDelay, now)) {
			save();
			somethingToSave = false;
			saveJustHappened = true;	
			logger->println(F("EEPROM saved."));
		}
	}
}

void MemoryBase::delayedSave(uint16_t pDelayMS) {
	somethingToSave = true;
	writeDelay = pDelayMS;
	memTimer.setDueTime(0);
}

boolean MemoryBase::hasBeenSaved() {
	if (saveJustHappened) {
		saveJustHappened = false;
		return true; 
	}
	return false;
}

void MemoryBase::save() {
	eeprom_write_block( (const void*)memRAM, (void*)memoryAddress, len);
	somethingToSave = false;
}

boolean MemoryBase::isEEPROMInitialized() {
	return (eeprom_read_word((const uint16_t *)magicMemoryNumberAddress) == EEMEM_MAGICNUMBER);
}		

void  MemoryBase::markEEPROMInitialized() {
	eeprom_write_word((uint16_t *)magicMemoryNumberAddress, EEMEM_MAGICNUMBER);
}
