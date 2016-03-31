#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;

extern const Vector3f g_Gravity;

struct  IPhysEnt
{
	virtual ~IPhysEnt(){};
	IPhysEnt();
	virtual void Draw() = 0;
	virtual void Step(float t) = 0;
	virtual void AddImpulse(Vector3f value, Vector3f pt = Vector3f(0.f,0.f,0.f)) = 0;
	virtual void AddAngularImpulse(Vector3f value) = 0;

	
	Vector3f m_pos;
	Quaternionf m_rot;

	Vector3f m_v;
	Vector3f m_w;

	float m_minv;
	Matrix3f m_Jinv;

	std::vector<Vector3f> m_forces;

	int m_id;
};


#endif//_GEOMETRY_H_
