#include "my_utils.h"
#include <sstream>
#include <iomanip>

//http://misc.flogisoft.com/bash/tip_colors_and_formatting
//const char* gRed = "\033[1;31m";
const char* gRed = "\033[41m";
//const char* gGreen = "\033[1;32m";
const char* gGreen = "\033[42m";
const char* gYellow = "\033[1;33m";
const char* gCyan = "\033[1;36m";
const char* gMagenta = "\033[1;35m";
const char* gReset = "\033[0m";

Eigen::Matrix3f getCrossMatrix(Vector3f t)
{
	Eigen::Matrix3f res;
	res <<	0,	 -t(2),	 t(1),
    			t(2), 0, 	-t(0),
    			-t(1), t(0), 0;

    return res;
}

QDebug operator<<(QDebug dbg, const Vector3f& v)
{
    dbg.nospace() << "(" << v.x() << "," << v.y() <<  "," << v.z() << ")";

    return dbg.space();
}

QDebug operator<<(QDebug dbg, const Vector4f& v)
{
    dbg.nospace() << "(" << v[0]/v[3] << "," << v[1]/v[3] <<  "," << v[2]/v[3] << "," << v[3] <<")";

    return dbg.space();
}

QDebug operator<<(QDebug dbg, const Quaternionf& q)
{
    dbg.nospace() <<"["<<q.w() <<"," << q.x() << "," << q.y() <<  "," << q.z() << "]";

    return dbg.space();
}

QDebug operator<<(QDebug dbg, const Matrix4i& m)
{
	dbg.nospace() <<	'\n' << m(0,0) << m(0,1) << m(0,2) << m(0,3) <<  
										'\n' << m(1,0) << m(1,1) << m(1,2) << m(1,3) << 
										'\n' << m(2,0) << m(2,1) << m(2,2) << m(2,3) << 
										'\n' << m(3,0) << m(3,1) << m(3,2) << m(3,3) << '\n'; 
	return dbg.space();
}

QDebug operator<<(QDebug dbg, const Matrix3f& m)
{
	dbg.space() <<		'\n' << m(0,0) << m(0,1) << m(0,2) <<  
										'\n' << m(1,0) << m(1,1) << m(1,2) << 
										'\n' << m(2,0) << m(2,1) << m(2,2) << '\n'; 
	return dbg.space();
}
Matrix3f matrixFromPYR(float pitch, float yaw, float roll)
{
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
	Eigen::Matrix3f rotationMatrix = q.matrix();

	return rotationMatrix;
}

Quaternion<float> quatFromPYR(float pitch, float yaw, float roll)
{
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;

	return q;
}

Quaternion<float> quatFromPYRAngles(float pitch, float yaw, float roll)
{
	pitch *= M_PI / 180.0f;
	yaw *= M_PI / 180.0f;
	roll *= M_PI / 180.0f;
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;

	return q;
}

Vector3f PYRFromQuat(Quaternionf& q)
{
	const float roll  =	atan2(2*q.y()*q.w() - 2*q.x()*q.z(), 1 - 2*q.y()*q.y() - 2*q.z()*q.z());
	const float pitch =	atan2(2*q.x()*q.w() - 2*q.y()*q.z(), 1 - 2*q.x()*q.x() - 2*q.z()*q.z());
	const float	yaw   =	asin(2*q.x()*q.y() + 2*q.z()*q.w());

	return Vector3f(yaw, pitch, roll);
}

Vector3f PYRAnglesFromQuat(Quaternionf& q)
{
	return PYRFromQuat(q) * 180.0f / M_PI;
}

bool isVectorsEqual(const Vector3f a, const Vector3f b, float eps /*=0.001*/)
{
	for (int i=0; i<3; ++i)
	{
		if (fabs(a[i] - b[i]) > eps)
			return false;
	}
	return true;
}

std::string VecToStr(const Vector3f& v)
{
	std::ostringstream stringStream;
  stringStream <<	std::setprecision(3)  << std::fixed << "(" << v.x() << ", " << v.y() <<  ", " << v.z() << ")";
  std::string res = stringStream.str();
	return res; 
}

float Clamp(float a, float lo, float hi)
{
  return std::max(lo, std::min(a, hi));
}
