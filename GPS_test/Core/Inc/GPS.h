/*
 * GPS.h
 *
 *  Created on: Oct 15, 2020
 *      Author: rishgoel
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "usart.h"
#include "lwgps.h"

struct location{
	double latitude;
	double longitude;

};

class GPS {
public:
	GPS();
	virtual ~GPS();

	void init(UART_HandleTypeDef* handle);

	bool update();

	location getPosition();

	//TODO Get velocity function

private:
	UART_HandleTypeDef* huart;
	lwgps_t lwgps_handle;

	location curr_position;
};

#endif /* INC_GPS_H_ */
