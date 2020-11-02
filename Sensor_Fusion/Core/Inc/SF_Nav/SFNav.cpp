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

void SF_Nav::init(UART_HandleTypeDef* uh, I2C_HandleTypeDef* ih, int refresh_time)
{
	//Initialize IMU and GPS


	//Initialize all matrices and set refresh time
	t = refresh_time;


//	h << Eigen::Matrix4f::Identity();


	// EKF
	I.setIdentity();	// 6x6 Identity
	// f is a function of previous state and action
	f << 1, 0, 0, t, 0, 0, 0.5*t*t, 0,
		 0, 1, 0, 0, t, 0, 0, 		0.5*t*t,
		 0, 0, 1, 0, 0, t, 0, 		0,
		 0, 0, 0, 1, 0, 0, t, 		0,
		 0, 0, 0, 0, 1, 0, 0, 		t,
		 0, 0, 0, 0, 0, 1, 0, 		0;
	// F is a jacobian of f*states vector w.r.t each state (acceleration not a state)
	F << 1, 0, 0, t, 0, 0,
		 0, 1, 0, 0, t, 0,
		 0, 0, 1, 0, 0, t,
		 0, 0, 0, 1, 0, 0,
		 0, 0, 0, 0, 1, 0,
		 0, 0, 0, 0, 0, 1;
	P_n = I;
	//TODO: Get an estimate for w, Q and v, R
	w << 0.1,
		 0.1,
		 0.1,
		 0.1,
		 0.1,
		 0.1;
	// Jacobian of w w.r.t states
	W << 0.1, 0, 0, 0, 0, 0,
		 0, 0.1, 0, 0, 0, 0,
		 0, 0, 0.1, 0, 0, 0,
		 0, 0, 0, 0.1, 0, 0,
		 0, 0, 0, 0, 0.1, 0,
		 0, 0, 0, 0, 0, 0.1;
//	Q << Eigen::Matrix6f::Identity();
	Q = I;
	v << 0.1,
		 0.1,
		 0.1,
		 0.1,
		 0.1,
		 0.1;
	// Jacobian of w w.r.t states
	V << 0.1, 0, 0, 0, 0, 0,
		 0, 0.1, 0, 0, 0, 0,
		 0, 0, 0.1, 0, 0, 0,
		 0, 0, 0, 0.1, 0, 0,
		 0, 0, 0, 0, 0.1, 0,
		 0, 0, 0, 0, 0, 0.1;
//	R << Eigen::Matrix6f::Identity();
//	h << Eigen::Matrix6f::Identity();
//	H << Eigen::Matrix6f::Identity();
	R = I;
	h = I;
	H = I;

	imu.initializeIMU(ih);
	gps.init(uh);
	while(!gps.update()) {
		start_location = gps.getPosition();
	}
}

void SF_Nav::update()
{
	double dist, bearing;

	//Get inputs u_n and z_n
	if(gps.update()) {
		curr_location = gps.getPosition();
		curr_vel = gps.getVelocity();

		lwgps_distance_bearing(prev_location.latitude, prev_location.longitude, curr_location.latitude, curr_location.longitude, &dist, &bearing);

		//Now convert the distance and bearing to and x and y
		state.x = state.x + sind(bearing)* dist;
		state.y = state.y + cosd(bearing)* dist;
		state.b = bearing;
		state.vX = sind(bearing) * curr_vel.speed;
		state.vY = cosd(bearing) * curr_vel.speed;
		state.vB = imu.getAngVel(IMU::Axes::z);

		// Set u_n and z_n
		u_n <<  imu.getLinearAcceleration(IMU::Axes::x),
				imu.getLinearAcceleration(IMU::Axes::y);
		z_n <<  state.x,
				state.y,
				state.b,
				state.vX,
				state.vY,
				state.vB;

		// Step 1: Predicted mean
		muu << x_n, u_n;	// Concatenate state at n-1 and actions
		x_pred = f*muu;		// Get next predicted state

		// Step 2: Predicted covariance
		P_pred = F*P_pred*F.transpose()+W*Q*W.transpose();

		// Step 3: Innovation
		y = z_n-x_pred;

		// Step 4: Innovation covariance
		S = H*P_pred*H.transpose()+V*R*V.transpose();

		// Step 5: Filter gain
		K_n = P_pred*H.transpose()*S.inverse();

		// Step 6: Corrected mean
		x_n = x_pred+K_n*y;

		// Step 7: Corrected covariance
		P_n = (I-K_n*H)*P_pred;
	}
}
