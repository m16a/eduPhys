#include "my_utils.h"

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
    dbg.nospace() << "(" << v.x() << ", " << v.y() <<  ", " << v.z() << ")";

    return dbg.space();
}

QDebug operator<<(QDebug dbg, const Quaternionf& q)
{
    dbg.nospace() <<"["<<q.w() <<"," << q.x() << ", " << q.y() <<  ", " << q.z() << "]";

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

Vector3f PYRFromQuat(Quaternionf& q)
{
  Matrix3f m = q.toRotationMatrix();
	Vector3f ea = m.eulerAngles(0,1,2);
	return ea /* 180.0f / M_PI*/;
}
