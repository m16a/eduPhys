#ifndef _BOX_H_
#define _BOX_H_

#include "geometry.h"
#include "icosphere.h"


using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;


class Box : public IPhysEnt
{

public:
//	static IcoSphere m_icoSphere;
	Box();
	virtual ~Box(){};

	virtual void Draw();
	virtual void Step(float t);
	virtual void AddImpulse(Vector3f value, Vector3f pt = Vector3f(0.f,0.f,0.f));
	virtual void AddAngularImpulse(Vector3f value);
	Vector3f Size()
	{
		return Vector3f(m_a, m_b, m_c);
	}

public:

	float m_a;
	float m_b;
	float m_c;

};

#endif
