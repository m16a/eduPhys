#ifndef _MY_UTILS_H_
#define _MY_UTILS_H_

#include <Eigen/Geometry>
#include <QtDebug>

using namespace Eigen;

extern Eigen::Matrix3f getCrossMatrix(Vector3f t);

extern QDebug operator<<(QDebug dbg, const Vector3f& v);

#endif// _MY_UTILS_H_
