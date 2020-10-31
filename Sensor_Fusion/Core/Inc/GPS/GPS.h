/*
 * GPS.h
 *
 *  Created on: Oct 15, 2020
 *      Author: rishgoel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
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

	void init(UART_HandleTypeDef* handle, xSemaphoreHandle gps_sem_handle);

	bool update();

	location getPosition()
	{
		return curr_position;
	}
	velocity getVelocity()
	{
		return curr_velocity;
	}

	char data[54];

private:
	UART_HandleTypeDef* huart;
	lwgps_t lwgps_handle;

	location curr_position;
	velocity curr_velocity;

	xSemaphoreHandle semHandle;

};

#endif /* INC_GPS_H_ */
