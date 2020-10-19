/*
 * GPS.h
 *
 *  Created on: Oct 15, 2020
 *      Author: rishgoel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "usart.h"
#include "GPS/lwgps.h"

struct location{
	double latitude;
	double longitude;

};

struct velocity{
	double speed;
	double bearing;
};

class GPS {
public:
	GPS();
	virtual ~GPS();

	void init(UART_HandleTypeDef* handle);

	bool update();

	location getPosition()
	{
		return curr_position;
	}
	velocity getVelocity()
	{
		return curr_velocity;
	}

	char data[200];
	bool has_data;

private:
	UART_HandleTypeDef* huart;
	lwgps_t lwgps_handle;

	location curr_position;
	velocity curr_velocity;

};

#endif /* INC_GPS_H_ */
