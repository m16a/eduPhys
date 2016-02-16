#ifndef _SPHERE_H_
#define _SPHERE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "icosphere.h"

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;


class Sphere
{

public:
	static IcoSphere m_icoSphere;
	Sphere();

	void Draw();
	void Step(float t);
	void AddImpulse(Vector3f value, Vector3f pt = Vector3f(0.f,0.f,0.f));
	void AddAngularImpulse(Vector3f value);

public:
	Vector3f m_pos;
	Quaternionf m_rot;
	float m_r;

	Vector3f m_v;
	Vector3f m_w;

	float m_minv;
	Matrix3f m_Jinv;

	int m_id;

};

#endif
