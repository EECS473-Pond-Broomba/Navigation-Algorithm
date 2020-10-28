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
	double x;
	double y;
	double vX;
	double vY;
};


class SF_Nav {
public:
	SF_Nav();
	virtual ~SF_Nav();

	void init(UART_HandleTypeDef* uh, I2C_HandleTypeDef* ih,int refresh_time);

	void update();
	state_var get_state();

private:
	IMU* imu;
	GPS* gps;
	Eigen::Matrix4f A, B, Q, H, R, P_pred, P, S, K;
	Eigen::RowVector4f x_pred, x, u_n, y, z_n;
	float t;
	location curr_location, prev_location;
	velocity curr_vel, prev_vel;
	state_var state;

	inline double sind(double x)
	{
	    return sin(x * M_PI / 180);
	}

	inline double cosd(double x)
	{
	    return cos(x * M_PI / 180);
	}

};

#endif /* INC_SF_NAV_SFNAV_H_ */
