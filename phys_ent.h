#ifndef _PHYSENT_H_
#define _PHYSENT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "serialization/phys_ent.pb.h"

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;

struct SRay;
struct SRayHit;

extern const Vector3f g_Gravity;

struct  IPhysEnt
{
	virtual ~IPhysEnt(){};
	IPhysEnt(bool isStatic = false);
	virtual void Draw() = 0;
	virtual void Step(float t) = 0;
	virtual void AddImpulse(const Vector3f& value, const Vector3f& pt = Vector3f(0.f,0.f,0.f)) = 0;
	virtual void AddAngularImpulse(const Vector3f& value) = 0;
	virtual void FullDump(){};
	virtual void UpdatedPosRot() = 0;
	virtual float CalcKineticEnergy() = 0;
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit) = 0;

	virtual void Serialize(ser::SerPhys* sp);
	virtual void Deserialize(const ser::SerPhys* sp);
	
	Vector3f m_pos;
	Quaternionf m_rot;

	Vector3f m_v;
	Vector3f m_w;

	float m_minv;
	Matrix3f m_Jinv;
	bool m_active;
	std::vector<Vector3f> m_forces;
	Vector3f m_bbox[2];
	int m_id;
};


#endif//_PHYSENT_H_
