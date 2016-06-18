#ifndef _MY_UTILS_H_
#define _MY_UTILS_H_

#include <Eigen/Geometry>
#include <QtDebug>

using namespace Eigen;

extern Eigen::Matrix3f getCrossMatrix(Vector3f t);

extern QDebug operator<<(QDebug dbg, const Vector3f& v);
extern QDebug operator<<(QDebug dbg, const Quaternionf& q);

extern Eigen::Matrix3f matrixFromPYR(float pitch, float yaw, float roll);

extern Quaternion<float> quatFromPYR(float pitch, float yaw, float roll);

extern Vector3f PYRFromQuat(Quaternionf& q);
#endif// _MY_UTILS_H_
