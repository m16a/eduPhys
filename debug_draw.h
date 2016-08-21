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
};

struct SDebugPlane : public IDebugItem 
{
	Vector3f m_n;
	float m_d;

	virtual void Draw();
};


struct SDebugMngr
{
//	void AddDebugItem(const IDebugItem* pDI);
	void DrawPlane(const Vector3f& n, const float d);
	std::list<IDebugItem*> m_list;
	void Draw(bool isPause = false);
};

SDebugMngr* DebugManager();


#endif
