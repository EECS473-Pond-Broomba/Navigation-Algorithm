/*
 * IMU.cpp
 *
 *  Created on: Oct 3, 2020
 *      Author: cy
 */

#include <imu/IMU.h>

IMU::IMU() {
	// Auto-generated constructor stub
}

IMU::~IMU() {
	// Auto-generated destructor stub
}

double IMU::getOrientation(Axes axis);

double IMU::getAngVel(Axes axis);

double IMU::getTotalAcceleration(Axes axis);

double IMU::getLinearAcceleration(Axes axis);
