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
}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

void GPS::init(UART_HandleTypeDef* handle, xSemaphoreHandle gps_sem_handle)
{
	huart = handle;
	semHandle = gps_sem_handle;

	lwgps_init(&lwgps_handle);

}

bool GPS::update()
{
	HAL_UART_Receive_IT(huart, (uint8_t*)data, 54);

	xSemaphoreTake(semHandle, portMAX_DELAY);

	char temp[200];
	strcpy(temp, data);
	lwgps_process(&lwgps_handle, temp, sizeof(temp));

	if(lwgps_handle.is_valid)
	{
		curr_position.latitude = lwgps_handle.latitude;
		curr_position.longitude = lwgps_handle.longitude;

		curr_velocity.speed = lwgps_handle.speed;
		curr_velocity.bearing = lwgps_handle.course;


		return true;
	}
	else
	{
		return false;
	}
}
