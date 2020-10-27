/*
 * SFNav.h
 *
 *  Created on: Oct 26, 2020
 *      Author: rishgoel
 */

#ifndef INC_SF_NAV_SFNAV_H_
#define INC_SF_NAV_SFNAV_H_

#include "IMU/IMU.h"
#include "GPS/GPS.h"
#include "Eigen/Dense"

struct state_var
{
	float x;
	float y;
	float vX;
	float vY;
};


class SF_Nav {
public:
	SF_Nav();
	virtual ~SF_Nav();

	void init(float refresh_time);

	void update();

private:
	IMU* imu;
	GPS* gps;
	Eigen::Matrix4f A, B, Q, H, R, P_pred, P, S, K;
	Eigen::RowVector4f x_pred, x, u_n, y, z_n;
	float t;
	location curr_location, prev_location;
	velocity curr_vel, prev_vel;
	state_var state;

};

#endif /* INC_SF_NAV_SFNAV_H_ */
