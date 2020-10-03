/*
 * IMU.h
 *
 *  Created on: Oct 3, 2020
 *      Author: cy
 */

#ifndef INC_IMU_IMU_H_
#define INC_IMU_IMU_H_

class IMU {
public:
	IMU();
	virtual ~IMU();

	// z - Yaw. Vertical axis that we are interested in, should almost always use z
	// x & y - Pitch and roll
	enum Axes { x, y, z};

	// Returns Euler angle of boat around input axis
	// Should almost always pass z in
	double getOrientation(Axes axis);

	// Returns angular velocity in rad/s
	// Should almost always pass z in
	double getAngVel(Axes axis);

	// Returns gravity + linear motion acceleration vector in m/s^2
	double getTotalAcceleration(Axes axis);

	// Returns linear motion acceleration without gravity in m/s^2
	double getLinearAcceleration(Axes axis);

private:
	
};

#endif /* INC_IMU_IMU_H_ */
