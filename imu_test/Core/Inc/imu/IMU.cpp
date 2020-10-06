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
	setMode(IMU_Mode::OPR_MODE_NDOF);
	// Set Euler Angles units to degrees
	eulerAngleUnits = false;
	// Set Angular Rate units to Dps
	gyroscopeUnits = false;
	// Set Acceleration units to m/s^2
	totalAccelerationUnits = false;
	// Set Temperature units to C
	tempUnits = false;
	// Set data output format to Windows format
	write8(Registers::BNO055_UNIT_SEL_ADDR, 0x00);
}

double IMU::getOrientation(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_EULER_P_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_EULER_R_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_EULER_H_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	uint16_t data = read16(registerToRead);
	return eulerAngleUnits ? (double)data / 900.0 : (double)data / 16.0;
}

double IMU::getAngVel(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_GYRO_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_GYRO_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_GYRO_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	uint16_t data = read16(registerToRead);
	return gyroscopeUnits ? (double)data / 900.0 : (double)data / 16.0;
}

double IMU::getTotalAcceleration(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_ACCEL_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_ACCEL_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_ACCEL_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	uint16_t data = read16(registerToRead);
	// Section 3.6.4.1 of datasheet for conversion from LSBs to m/s^2
	return totalAccelerationUnits ? (double)data : (double)data / 100.0;
}

double IMU::getLinearAcceleration(Axes axis) {
	// Set register we need to read depending on the axis passed in
	uint8_t registerToRead = 0;
	switch(axis) {
	case Axes::x:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
		break;
	case Axes::y:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR;
		break;
	case Axes::z:
		registerToRead = Registers::BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR;
		break;
	default:
		break;
	}

	// Read the data registers
	uint16_t data = read16(registerToRead);
	// Section 3.6.5.6 of datasheet for conversion from LSBs to m/s^2
	return (double)data / 100.0;
}

void IMU::setMode(IMU_Mode mode) {
	write8(Registers::BNO055_OPR_MODE_ADDR, mode);
	currentMode = mode;
	// Time required to switch between operating modes (Datasheet Table 3-6)
	mode == IMU_Mode::OPR_MODE_CONFIGMODE ? vTaskDelay(19) : vTaskDelay(7);
	return;
}

HAL_StatusTypeDef IMU::write8(uint8_t reg, uint8_t value) {
	HAL_StatusTypeDef ret;
	// Combine reg and value into a buffer
	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = value;
	// Send buffer over
	ret = HAL_I2C_Master_Transmit(&hi2c, IMU_I2C_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
	return ret;
}

uint8_t IMU::read8(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t value = 0;
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(&hi2c, IMU_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(&hi2c, IMU_I2C_ADDR << 1, &value, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	return value;
}

uint16_t IMU::read16(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(&hi2c, IMU_I2C_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(&hi2c, IMU_I2C_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// The LSB is always at the lower register address, so cast buffer[0] into 16 bits and shift it left by 8
	// And then OR with MSB to combine into 2 bytes
	uint16_t value = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
	return value;
}
