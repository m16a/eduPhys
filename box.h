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
	Box(const Vector3f& size = Vector3f(.1f,.2f,.3f), bool isStatic = false);
	virtual ~Box(){};

	virtual void Draw();
	virtual void Step(float t);
	virtual void AddImpulse(Vector3f value, Vector3f pt = Vector3f(0.f,0.f,0.f));
	virtual void AddAngularImpulse(Vector3f value);
	virtual float CalcKineticEnergy();
	virtual void FullDump();

	virtual void Serialize(ser::SerPhys* sp);
	virtual void Deserialize(const ser::SerPhys* sp);
	Vector3f Size() const
	{
		return m_size; 
	}
	
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit);

public:

	Vector3f m_size;
};

void getBoxVerticies(const Box* b, Vector3f out_arr[6]);

#endif
