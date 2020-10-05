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

void IMU::initializeIMU() {
	// Set mode to NDOF
	// Set Eular Angles units to degrees or radians
	// Set Angular Rate units to Dps or Rps
	// Set Acceleration units to m/s^2 or mg
}

double IMU::getOrientation(Axes axis) {

}

double IMU::getAngVel(Axes axis) {

}

double IMU::getTotalAcceleration(Axes axis) {

}

double IMU::getLinearAcceleration(Axes axis) {

}

void IMU::setMode(IMU_MODE) {

}

void IMU::write8(Registers reg, uint8_t value) {

}
