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


struct SDebugVector : public IDebugItem 
{
	Vector3f m_pos;
	Vector3f m_dir;
	float m_len;

	virtual void Draw();
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
