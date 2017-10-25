#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen\Eigen>
#include <Eigen\Geometry>

using namespace std;

class EulerAngle
{
public:
	float pitch_y = 0;
	float roll_x = 0;
	float yaw_z = 0;

	EulerAngle()
	{
		float pitch_y = 0;
		float roll_x = 0;
		float yaw_z = 0;
	}

	EulerAngle(float roll_x, float pithc_y, float yaw_z)
	{
		this->roll_x = roll_x;
		this->pitch_y = pithc_y;
		this->yaw_z = yaw_z;
	};

	void euler_to_eigen_quaternion(Eigen::Quaternionf& qua)
	{
		// Abbreviations for the various angular functions
		float cy = cos(yaw_z * 0.5);
		float sy = sin(yaw_z * 0.5);
		float cr = cos(roll_x * 0.5);
		float sr = sin(roll_x * 0.5);
		float cp = cos(pitch_y * 0.5);
		float sp = sin(pitch_y * 0.5);

		qua.w() = cy * cr * cp + sy * sr * sp;
		qua.x() = cy * sr * cp - sy * cr * sp;
		qua.y() = cy * cr * sp + sy * sr * cp;
		qua.z() = sy * cr * cp - cy * sr * sp;
	};

	void quaternion_to_euler(Eigen::Quaternionf& q)
	{
		// roll (x-axis rotation)
		float sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
		float cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
		roll_x = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		float sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
		if (fabs(sinp) >= 1)
			pitch_y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			pitch_y = asin(sinp);

		// yaw (z-axis rotation)
		float siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
		float cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
		yaw_z = atan2(siny, cosy);
	};

	void euler_to_eigen_matrix(Eigen::Matrix4f& max)
	{
		float sa = sin(pitch_y);
		float ca = cos(pitch_y);
		float sb = sin(roll_x);
		float cb = cos(roll_x);
		float sh = sin(yaw_z);
		float ch = cos(yaw_z);

		max << ch*ca, sh*sb - ch*sa*cb, ch*sa*sb + sh*cb, 0,
			sa, ca*cb, -ca*sb, 0,
			-sh*ca, sh*sa*cb + ch*sb, -sh*sa*sb + ch*cb, 0,
			0, 0, 0, 1;
	};

	void matrix_to_euler(Eigen::Matrix4f m)
	{
		// Assuming the angles are in radians.
		if (m(1,0) > 0.998) { // singularity at north pole
			yaw_z = atan2(m(0,2), m(2,2));
			pitch_y = (M_PI / 2);
			roll_x = 0;
			return;
		}
		if (m(1,0) < -0.998) { // singularity at south pole
			yaw_z = atan2(m(0,2), m(2,2));
			pitch_y = (-M_PI / 2);
			roll_x = 0;
			return;
		}
		yaw_z = atan2(-m(2,0), m(0,0));
		pitch_y = atan2(-m(1,2), m(1,1));
		roll_x = asin(m(1,0));
	};

	EulerAngle& operator+(EulerAngle& other);

	EulerAngle& operator-(EulerAngle& other);

	EulerAngle& operator/(EulerAngle& other);

	EulerAngle operator*(EulerAngle& other);
};