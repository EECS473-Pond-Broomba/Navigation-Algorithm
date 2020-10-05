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

void IMU::initializeIMU(const I2C_HandleTypeDef &handle) {
	hi2c = handle;
	// Set mode to NDOF
	setMode(IMU_MODE::OPR_MODE_NDOF);
	// Set Eular Angles units to degrees or radians
	// Set Angular Rate units to Dps or Rps
	// Set Acceleration units to m/s^2 or mg
}

double IMU::getOrientation(Axes axis) {
	return 0.0;
}

double IMU::getAngVel(Axes axis) {
	return 0.0;
}

double IMU::getTotalAcceleration(Axes axis) {
	return 0.0;
}

double IMU::getLinearAcceleration(Axes axis) {
	return 0.0;
}

void IMU::setMode(IMU_MODE) {
	return;
}

void IMU::write8(Registers reg, uint8_t value) {
	HAL_I2C_Master_Transmit(&hi2c, reg, &value, 1, TIMEOUT);
	return;
}
