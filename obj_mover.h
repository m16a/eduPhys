#ifndef _OBJ_MOVER_H_
#define _OBJ_MOVER_H_

#include <Eigen/Geometry>

using Eigen::Vector3f;
using Eigen::Quaternionf;

struct IPhysEnt; 
struct SRay;

struct STorus 
{
	STorus();
	void Draw();
	
	void line_intersect(const Vector3f& org, const Vector3f& dir, int * num_intersections,
			  float * intersections) const;
	Vector3f m_pos;
	Quaternionf m_rot;
	float m_rMajor;
	float m_rMinor;
	Vector3f m_col;
};

struct SRotationHelper
{
	void Draw(){};
	
	STorus m_helpers[3];	
};


class ObjMover
{
public:
	ObjMover();
	bool OnMouseMove(const Vector3f& in, const SRay& r);
		
	void OnSelect(IPhysEnt* e);	
	void OnDeselect(IPhysEnt* e);	
	
	bool RWI(const SRay& r);//returns true if hit hellper
	void Update();
	Vector3f m_lastIn;
	IPhysEnt* m_pSelectedEnt;
	SRotationHelper m_rotHlpr;
	int m_activeHelperIndx;
};

#endif
