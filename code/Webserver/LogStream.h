/*
 * LogBuffer.h
 *
 * circular buffer used to store log entries and display them on the web page
 *
 *  Created on: 03.10.2018
 *      Author: JochenAlt
 */

#ifndef LOGSTREAM_H_
#define LOGSTREAM_H_

#include <Arduino.h>

class LogStream : public Stream
{
public:
	LogStream() {};
	virtual ~LogStream() {};

    int available(void) override { 	return size;};
    virtual int read() override { return get(0);pop(); };
    virtual int peek() override { return get(0); };
    virtual int peek(int idx) { return get(idx); };
	virtual size_t write(uint8_t c)  override {
		push(c);
		return 1;
	}

private:


    static const int BufferSize = 1024;
	uint8_t buffer[BufferSize];

	int  inIdx = 0;
	int  outIdx = 0;
	int size = 0;

	void push(uint8_t c) {
		// make room for one character
		if (size == BufferSize)
			pop();

		buffer[inIdx++] = c;
		if (inIdx >= BufferSize)
			inIdx = 0;
		size++;
	}

	uint8_t pop() {
		if (size > 0) {
			uint8_t c = buffer[outIdx++];
			if (outIdx >= BufferSize)
				outIdx = 0;
			size--;
			return c;
		}
		else
			return 0;
	}

	uint8_t get(int idx) {
		if (idx < size) {
			if (outIdx + idx >= BufferSize)
				return buffer[outIdx+idx-BufferSize];
			else
				return buffer[outIdx+idx];
		}
		else
			return 0;
	}

};


extern LogStream* logger;

#endif /* LOGSTREAM_H_ */
