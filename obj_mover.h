#ifndef _OBJ_MOVER_H_
#define _OBJ_MOVER_H_

#include <Eigen/Geometry>

using Eigen::Vector3f;

class ObjMover
{
public:
	void OnMouseMove(const Vector3f& in);
	
	Vector3f m_lastIn;
};

#endif
