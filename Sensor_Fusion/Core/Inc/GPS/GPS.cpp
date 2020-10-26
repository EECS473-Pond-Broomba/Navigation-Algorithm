/*
 * GPS.cpp
 *
 *  Created on: Oct 19, 2020
 *      Author: rishgoel
 */

#include <GPS/GPS.h>
#include "stdio.h"
#include "string.h"
GPS::GPS() {
	// TODO Auto-generated constructor stub
	has_data = false;
}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

void GPS::init(UART_HandleTypeDef* handle)
{
	huart = handle;

	lwgps_init(&lwgps_handle);

	HAL_UART_Receive_IT(huart, (uint8_t*)data, sizeof(data));
}

bool GPS::update()
{
	if(has_data)
	{
		//Store buffer here
		char temp[200];
		strcpy(temp, data);
		has_data = false;
		lwgps_process(&lwgps_handle, temp, sizeof(temp));

		if(lwgps_handle.is_valid)
		{
			curr_position.latitude = lwgps_handle.latitude;
			curr_position.longitude = lwgps_handle.longitude;

			curr_velocity.speed = lwgps_handle.speed;
			curr_velocity.bearing = lwgps_handle.course;

			HAL_UART_Receive_IT(huart, (uint8_t*)data, 200);

			return true;
		}
		else
		{
			HAL_UART_Receive_IT(huart, (uint8_t*)data, 200);

			return false;
		}


	}

	return false;
}
