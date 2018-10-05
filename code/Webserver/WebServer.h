/*
 * WebServer.h
 *
 *  Created on: 03.10.2018
 *      Author: JochenAlt
 */

#ifndef WEBSERVER_H_
#define WEBSERVER_H_

#include <Arduino.h>
class WebServer {
public:
	WebServer() {};
	virtual ~WebServer() {};

	void setup();
	void loop();
private:
	// Variable to store the HTTP request
	String request;
};

#endif /* WEBSERVER_H_ */
