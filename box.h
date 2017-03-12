#ifndef _BOX_H_
#define _BOX_H_

#include "phys_ent.h"
#include "icosphere.h"


using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;

class Box : public IPhysEnt
{
public:
	Box(float minv = 1.0f, const Vector3f& size = Vector3f(.1f,.2f,.3f), bool isStatic = false);
	virtual ~Box(){};

	virtual void Draw();
	virtual void Step(float t);
	virtual bool AddImpulse(const Vector3f& value, const Vector3f& pt = Vector3f(0.f,0.f,0.f));
	virtual void AddAngularImpulse(const Vector3f& value);
	virtual void FullDump();
	virtual void UpdatedPosRot();
	virtual float CalcKineticEnergy();
	
	virtual void Serialize(ser::SerPhys* sp);
	virtual void Deserialize(const ser::SerPhys* sp);
	Vector3f Size() const;
	
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit);
	void UpdateCachedVerticies();
public:
	Vector3f m_size;

	Vector3f m_cachedVertcs[8];
};

void getBoxVerticies(const Box& b, const Vector3f** out_arr);

#endif
