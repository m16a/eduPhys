#ifndef _RWI_H_
#define _RWI_H_ 

#include <Eigen/Core>
#include <Eigen/Geometry>

class IPhysEnt;

struct SRay
{
	Vector3f m_org; //origin
	Vector3f m_dir;
};

struct SRayHit
{
	Vector3f m_pt;
	Vector3f m_n;
	float m_dist;
	IPhysEnt* m_pEnt;
};

#endif //_RWI_H_ 
