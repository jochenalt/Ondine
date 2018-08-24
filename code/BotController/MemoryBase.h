/*
 * Memory.h
 *
 * Base class to store persistent data in EEPROM. Use by deriving your memory
 * class from MemoryBase, and initializing it within the constructor.
 *
 * Example:
 * class DerivedMemClass : MemoryBase {
 * ...
 * 	struct  {
 *		... my persistent data ...
 *	} persMem;
 * }
 *
 * DerivedMemClass::DerivedMemClass()
 * MemoryBase((void*)&(persMem),sizeof(DerivedMemClass::persMem)) {
 *    ... initialize default settings of data in persMem
 * }
 *
 * to initialize, call MemoryBase::setup(). In the loop, call MemoryBase::loop.
 * When changing persMem, call delayedSave(), which queues up the change to be written in a couple of seconds.
 *
 * Author: JochenAlt
 */ 


#ifndef MEMORY_H_
#define MEMORY_H_

#include "Arduino.h"
#include "TimePassedBy.h"

class MemoryBase {
	protected:
		// initialize by passing the persistent block of derived class
		MemoryBase (void *pMem_RAM, size_t pMemSize);
	public:
		// initializes and reads the persistent memory from eeprom.
		// returns true, if this is the first call 
		boolean setup();

		// To be called after a change of persistent data. Can be called very
		// often, after a couple of seconds, all changes are written to epprom.
		void delayedSave(uint16_t pDelayMS = 10000);

		// saves persistent memory immediately to EEPROM.
		void save();

		// return strue, when every call of delaySave has been saved already.
		boolean hasBeenSaved();

		// to be called in uC's loop, checks whether a call of delaySave has to be written to EEPROM
		void loop(uint32_t now);
	private:
		void read();
		boolean isEEPROMInitialized();
		void  markEEPROMInitialized();
		TimePassedBy memTimer;
		uint16_t writeDelay;
		boolean somethingToSave;
		boolean saveJustHappened;
		void* memRAM;
		uint8_t len;
};

#endif /* MEMORY_H_ */
