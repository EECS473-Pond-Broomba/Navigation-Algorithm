/*
 * IMU.cpp
 *
 *  Created on: Oct 3, 2020
 *      Author: cy
 */

#include <IMU/IMU.h>

IMU::IMU() {
	// Auto-generated constructor stub
}

IMU::~IMU() {
	// Auto-generated destructor stub
}

void IMU::initializeIMU(I2C_HandleTypeDef* handle) {
	hi2c = handle;
	// Set mode to NDOF
	setMode(IMU_Mode::OPR_MODE_NDOF);
	// Set acceleration samples to 0 and time step to 0
	accelTimeSteps = 0;
	accelerationSamples[0][0] = 0.0;
	accelerationSamples[0][1] = 0.0;
	accelerationSamples[1][0] = 0.0;
	accelerationSamples[1][1] = 0.0;
	accelerationSamples[2][0] = 0.0;
	accelerationSamples[2][1] = 0.0;
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
	vTaskDelay(20);
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
	int16_t data = read16(registerToRead);
	// Section 3.6.5.4 of datasheet for conversion from LSBs to deg/rad
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
	int16_t data = read16(registerToRead);
	// Table 3-22 of datasheet for conversion from LSBs to Dps/Rps
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
	int16_t data = read16(registerToRead);
	// Section 3.6.4.1 of datasheet for conversion from LSBs to m/s^2
	return totalAccelerationUnits ? (double)data : (double)data / 100.0;
}

// Shifts elements in column 1 into column 0 and new readings into column 1
void IMU::storeTotalAcceleration() {
	accelerationSamples[0][0] = accelerationSamples[0][1];
	accelerationSamples[0][1] = getTotalAcceleration(Axes::x);
	accelerationSamples[1][0] = accelerationSamples[1][1];
	accelerationSamples[1][1] = getTotalAcceleration(Axes::y);
	accelerationSamples[2][0] = accelerationSamples[2][1];
	accelerationSamples[2][1] = getTotalAcceleration(Axes::z);
	accelTimeSteps++;
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
	int16_t data = read16(registerToRead);
	// Section 3.6.5.6 of datasheet for conversion from LSBs to m/s^2
	return (double)data / 100.0;
}

// Shifts elements in column 1 into column 0 and new readings into column 1
void IMU::storeLinearAcceleration() {
	accelerationSamples[0][0] = accelerationSamples[0][1];
	accelerationSamples[0][1] = getLinearAcceleration(Axes::x);
	accelerationSamples[1][0] = accelerationSamples[1][1];
	accelerationSamples[1][1] = getLinearAcceleration(Axes::y);
	accelerationSamples[2][0] = accelerationSamples[2][1];
	accelerationSamples[2][1] = getLinearAcceleration(Axes::z);
	accelTimeSteps++;
}

void IMU::calculateLinearVelocity() {
	// If no acceleration time steps taken, return to avoid divide by zero
	if(accelTimeSteps == 0) {
		return;
	}
	velocity[0] += (accelerationSamples[0][0] + accelerationSamples[0][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
	velocity[1] += (accelerationSamples[1][0] + accelerationSamples[1][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
	velocity[2] += (accelerationSamples[2][0] + accelerationSamples[2][1]) / 2.0 * (ACCELERATION_TIME_STEP / 1000.0);
}

double IMU::getLinearVelocity(Axes axis) {
	switch(axis) {
	case Axes::x:
		return velocity[0];
		break;
	case Axes::y:
		return velocity[1];
		break;
	case Axes::z:
		return velocity[2];
		break;
	case Axes::xy:
		return sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
		break;
	default:
		break;
	}
	return 0.0;
}

void IMU::setMode(IMU_Mode mode) {
	write8(Registers::BNO055_OPR_MODE_ADDR, mode);
	currentMode = mode;
	// Time required to switch between operating modes (Datasheet Table 3-6)
	mode == IMU_Mode::OPR_MODE_CONFIGMODE ? vTaskDelay(pdMS_TO_TICKS(19)) : vTaskDelay(pdMS_TO_TICKS(7));
	return;
}

uint8_t IMU::getCalibStatus() {
	return (uint8_t)read8(Registers::BNO055_CALIB_STAT_ADDR);
}

uint8_t IMU::getSysStatus() {
	return (uint8_t)read8(Registers::BNO055_SYS_STAT_ADDR);
}

uint8_t IMU::getSysError() {
	return (uint8_t)read8(Registers::BNO055_SYS_ERR_ADDR);
}

HAL_StatusTypeDef IMU::write8(uint8_t reg, uint8_t value) {
	HAL_StatusTypeDef ret;
	// Combine reg and value into a buffer
	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = value;
	// Send buffer over
	ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDR << 1, buffer, 2, 2);
	return ret;
}

int8_t IMU::read8(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t value = 0;
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDR << 1, &reg, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, IMU_I2C_ADDR << 1, &value, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFF;
	}
	return (int8_t)value;
}

int16_t IMU::read16(uint8_t reg) {
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];
	// Tell sensor that we want to read from reg
	ret = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDR << 1, &reg, 1, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// Read 1 byte from reg
	ret = HAL_I2C_Master_Receive(hi2c, IMU_I2C_ADDR << 1, buffer, 2, I2C_TIMEOUT);
	if(ret != HAL_OK) {
		return 0xFFFF;
	}
	// The LSB is always at the lower register address, so cast buffer[0] into 16 bits and shift it left by 8
	// And then OR with MSB to combine into 2 bytes
	int16_t value = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
	return value;
}
