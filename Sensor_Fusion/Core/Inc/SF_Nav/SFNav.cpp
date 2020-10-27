/*
 * SFNav.cpp
 *
 *  Created on: Oct 26, 2020
 *      Author: rishgoel
 */

#include <SF_Nav/SFNav.h>
#include <cmath>

SF_Nav::SF_Nav() {
	// TODO Auto-generated constructor stub

}

SF_Nav::~SF_Nav() {
	// TODO Auto-generated destructor stub
}

void SF_Nav::init(UART_HandleTypeDef* uh, I2C_HandleTypeDef* ih,float refresh_time)
{
	//Initialize IMU and GPS
	imu->initializeIMU(ih);
	gps->init(uh);

	//Initialize all matrices and set refresh time
	t = refresh_time;

	A << 1, 0, t, 0,
		 0, 1, 0, t,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	B << 0.5*t*t, 0      , 0, 0,
		 0      , 0.5*t*t, 0, 0,
		 t      , 0      , 0, 0,
		 0      , t      , 0, 0;

	P << Eigen::Matrix4f::Identity();
	H << Eigen::Matrix4f::Identity();
	//TODO: Get an estimate for Q and R
	Q << Eigen::Matrix4f::Identity();
	R << Eigen::Matrix4f::Identity();

}

void SF_Nav::update()
{
	double dist, bearing;

	//Get inputs u_n and z_n
	gps->update();
	curr_location = gps->getPosition();
	curr_vel = gps->getVelocity();

	lwgps_distance_bearing(prev_location.latitude, prev_location.longitude, curr_location.latitude, curr_location.longitude, &dist, &bearing);

	//Now convert the distance and bearing to and x and y
	state.x = state.x + sind(bearing)* dist;
	state.y = state.y + cosd(bearing)* dist;
	state.vX = sind(bearing) * curr_vel.speed;
	state.vY = cosd(bearing) * curr_vel.speed;




}
