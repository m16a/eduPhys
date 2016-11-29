#ifndef _DEBUG_DRAW_H_
#define _DEBUG_DRAW_H_

#include "geometry.h"
#include <list>

using Eigen::Vector3f;
using Eigen::Quaternionf;
using Eigen::Matrix3f;

struct IDebugItem
{
	virtual void Draw() = 0;
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit) = 0;
};

struct SDebugPlane : public IDebugItem 
{
	SDebugPlane(){};
	SDebugPlane(const Vector3f& point, const Vector3f& normal);
 
	virtual void Draw();
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit);
	
	Vector3f m_n;
	float m_d;
};


struct SDebugVector : public IDebugItem 
{
	Vector3f m_pos;
	Vector3f m_dir;
	float m_len;

	virtual void Draw();
	virtual int IntersectRay(const SRay& r, SRayHit& out_hit) {assert(0);};
};

struct SDebugMngr
{
//	void AddDebugItem(const IDebugItem* pDI);
	void DrawPlane(const Vector3f& n, const float d);
	void DrawVector(const Vector3f& pos, const Vector3f& dir, const float len);
	std::list<IDebugItem*> m_list;
	void Draw(bool isPause = false);
};

SDebugMngr* DebugManager();


#endif
