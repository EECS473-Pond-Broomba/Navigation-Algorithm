/*
 * GPS.cpp
 *
 *  Created on: Oct 18, 2020
 *      Author: rishgoel
 */

#include <GPS/GPS.h>

GPS::GPS() {
	// TODO Auto-generated constructor stub
	has_update = false;

}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

void GPS::init(UART_HandleTypeDef* handle)
{
	huart = handle;
	lwgps_init(&hlwgps);

}

bool GPS::update()
{
	if(has_update)
	{
		if(hlwgps.is_valid)
		{
			pos.lattitude = hlwgps.latitude;
			pos.longitude = hlwgps.longitude;
			return true;
		}
	}
	return false;
}
