#ifndef _OBJ_MOVER_H_
#define _OBJ_MOVER_H_

#include <Eigen/Geometry>

using Eigen::Vector3f;
using Eigen::Quaternionf;

struct IPhysEnt; 


struct STorus 
{
	STorus();
	void Draw();
	
	int line_intersect(const Vector3f& org, const Vector3f& dir, int * num_intersections,
			  float * intersections, float * shade) const;
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
	void OnMouseMove(const Vector3f& in);
	
	void OnSelect(IPhysEnt* e);	
	void OnDeselect(IPhysEnt* e);	
	
	void Update();
	Vector3f m_lastIn;
	IPhysEnt* m_pSelectedEnt;
	SRotationHelper m_rotHlpr;
};

#endif
