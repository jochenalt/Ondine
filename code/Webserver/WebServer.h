/*
 * WebServer.h
 *
 *  Created on: 03.10.2018
 *      Author: JochenAlt
 */

#ifndef WEBSERVER_H_
#define WEBSERVER_H_

class WebServer {
public:
	WebServer() {};
	virtual ~WebServer() {};


	void setup();
	void loop();
};

#endif /* WEBSERVER_H_ */
