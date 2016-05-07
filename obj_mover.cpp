#include "obj_mover.h"
#include <cmath>
#include <QtDebug>

void ObjMover::OnMouseMove(const Vector3f& in)
{
	float distScr = (in - m_lastIn).norm();
	float angleScr = atan2((-in[1]+m_lastIn[1]), (in[0]-m_lastIn[0]));	
	
	qDebug() << "2D moving: " << distScr << " " << angleScr*180.0f/M_PI;
	m_lastIn = in;
}
