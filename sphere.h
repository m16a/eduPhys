#ifndef _SPHERE_H_
#define _SPHERE_H_

#include "geometry.h"
#include "icosphere.h"


using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;


class Sphere : public IPhysEnt
{

public:
	static IcoSphere m_icoSphere;
	Sphere(bool isStatic = false);
	virtual ~Sphere(){};

	virtual void Draw();
	virtual void Step(float t);
	virtual void AddImpulse(Vector3f value, Vector3f pt = Vector3f(0.f,0.f,0.f));
	virtual void AddAngularImpulse(Vector3f value);
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit);
	virtual float CalcKineticEnergy() {assert(0); return 0;};
	virtual void Serialize(ser::SerPhys* sp);
	virtual void Deserialize(const ser::SerPhys* sp);

public:

	float m_r;

};

#endif
