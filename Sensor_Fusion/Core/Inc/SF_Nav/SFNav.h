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



class SF_Nav {
public:
	SF_Nav();
	virtual ~SF_Nav();

private:
	IMU* imu;
	GPS* gps;
};

#endif /* INC_SF_NAV_SFNAV_H_ */
