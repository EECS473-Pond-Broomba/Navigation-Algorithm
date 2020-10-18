/*
 * GPS.cpp
 *
 *  Created on: Oct 15, 2020
 *      Author: rishgoel
 */

#include <GPS.h>

GPS::GPS() {
	// TODO Auto-generated constructor stub

}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

void GPS::init(UART_HandleTypeDef* handle){
	//Set the UART handle for gps data
	huart = handle;
	//Initialize the lwgps parser
	lwgps_init(&lwgps_handle);
}

bool GPS::update(){
	char* data = "null";
	//TODO Maybe change this to an interrupt based thing
	//HAL_UART_Receive(huart,(uint8_t*) data, sizeof(data), 1000);

	lwgps_process(&lwgps_handle, data, sizeof(data));

	if(lwgps_handle.is_valid)
	{
		curr_position.latitude = lwgps_handle.latitude;
		curr_position.longitude = lwgps_handle.longitude;
		return true;
	}
	return false;
}

location GPS::getPosition(){
	return curr_position;
}
