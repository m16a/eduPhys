#ifndef _MY_UTILS_H_
#define _MY_UTILS_H_

#include <Eigen/Geometry>
#include <QtDebug>

using namespace Eigen;

extern Eigen::Matrix3f getCrossMatrix(Vector3f t);

extern QDebug operator<<(QDebug dbg, const Vector3f& v);
extern QDebug operator<<(QDebug dbg, const Vector4f& v);
extern QDebug operator<<(QDebug dbg, const Quaternionf& q);

extern Eigen::Matrix3f matrixFromPYR(float pitch, float yaw, float roll);

extern Quaternion<float> quatFromPYR(float pitch, float yaw, float roll);

extern Vector3f PYRFromQuat(Quaternionf& q);

bool isVectorsEqual(const Vector3f a, const Vector3f b);

extern const char* gRed;
extern const char* gGreen;
extern const char* gYellow;
extern const char* gCyan;
extern const char* gMagenta;
extern const char* gReset;
#endif// _MY_UTILS_H_
