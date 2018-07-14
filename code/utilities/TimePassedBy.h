/*
 * TimePassedBy.h
 *
 * Created: 21.11.2014 23:01:24
 *  Author: JochenAlt
 */


#ifndef TIMEPASSEDBY_H_
#define TIMEPASSEDBY_H_

// small helper class to measure the passed time since an event and to check
// whether something that is supposed to run at a specific time (due_ms/due_us)
// use:
//     TimePassedBy timer(MS);						// initialize timer that is supposed to execute something periodically
//     int16_t passed_ms;
//     while (true) {								// to be used in a loop
//			<do what you like>
//			if (timer.due_ms(200, passed_ms))		// check whether 200ms have been passed and
//				<do something that has to be called every 200ms>
//		}
//
class TimePassedBy {
	public:
	// initialize this timer to work
	TimePassedBy () {
		mLastCall_ms = millis();
		mRate = 0;
		firstCalldT = true;
	}
	TimePassedBy(uint32_t rate) {
		mRate = rate;
		firstCalldT = true;
	}

	void setRate(uint32_t rate ) { mRate = rate; };
	uint32_t getRate() { return mRate; };

	// true, if at least <ms> milliseconds passed since last invocation that returned true.
	// returns the actual passed time in addition

	// true, if at least <ms> milliseconds passed since last invocation that returned true.
	// returns the actual passed time in addition
	bool isDue_ms(uint16_t ms, uint16_t &passed_ms, uint32_t now) {
		passed_ms = now-mLastCall_ms;
		if (passed_ms>=ms) {
			mLastCall_ms = now;
			return true;
		}
		return false;
	}

	float dT() {
		uint32_t now = millis();
		float duration = (now - mLastCall_ms)/1000.0;
		mLastCall_ms = now;
		if (firstCalldT) {
			firstCalldT = false;
			return 0;
		};
		return duration;
	};

	bool isDue() {
		uint16_t passed_ms;
		return isDue_ms(mRate, passed_ms, millis());
	}

	void delayNextFire(uint32_t delay_ms) {
		mLastCall_ms += delay_ms;
	}

	void setDueTime (uint32_t due_ms) {
		mLastCall_ms = due_ms-mRate;
	}

	// returns when the timer fires the next time [ms]
	int32_t getDueTime () {
		return int32_t(mLastCall_ms + mRate);
	}

	bool isDue_ms(uint16_t ms, uint32_t now) {
		uint16_t passed_ms;
		return isDue_ms(ms, passed_ms,now);
	}

	uint32_t mLastCall_ms;	// last due time in milliseconds
	uint32_t mRate;			// rate
	bool firstCalldT = true;
};


#endif /* TIMEPASSEDBY_H_ */
