#ifndef _OBJ_MOVER_H_
#define _OBJ_MOVER_H_

#include <Eigen/Geometry>

using Eigen::Vector3f;
using Eigen::Quaternionf;

struct IPhysEnt; 


struct STorus 
{
	void Draw();
	
	Vector3f m_pos;
	Quaternionf m_rot;
	float m_rIn;
	float m_rOut;
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
