/*
 * GPS.h
 *
 *  Created on: Oct 18, 2020
 *      Author: rishgoel
 */

#ifndef INC_GPS_GPS_H_
#define INC_GPS_GPS_H_

#include "usart.h"
#include "lwgps.h"

struct location
{
	float lattitude;
	float longitude;
};

struct velocity
{
	float speed;
	float bearing;
};

class GPS {
public:
	GPS();
	virtual ~GPS();

	void init(UART_HandleTypeDef* handle);
	bool update();

	location getLocation()
	{
		return pos;
	}

	velocity getVelocity()
	{
		return vel;
	}

	char data[79];
	bool has_update;

private:
	location pos;
	velocity vel;

	UART_HandleTypeDef huart;
	lwgps_t hlwgps;
};

#endif /* INC_GPS_GPS_H_ */
