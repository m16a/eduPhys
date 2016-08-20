#include "debug_draw.h"
#include <QtDebug>
#include "my_utils.h"
#include "gpuhelper.h"

void SDebugPlane::Draw()
{
	Vector3f u,v;
	if(isVectorsEqual(m_n, Vector3f(0,0,1)))
	{
		 u = Vector3f(1,0,0); // x
		 v = Vector3f(0,1,0); // y
	}
	else
	{
		 u = m_n.cross(Vector3f(0,0,1)); // cross product -> note that u lies on the plane
		 v = m_n.cross(u); // v is orthogonal to both N and u (again is in the plane)  
	}
	// now simply draw a quad centered in a arbitrary point of the plane
	// and large enough to seems a plane
	Vector3f P0 = -m_n * m_d;        // "arbitrary" point
	float  f  = 1;  // large enough
	Vector3f fu =  u * f;
	Vector3f fv =  v * f;
	Vector3f P1 = P0 - fu - fv;
	Vector3f P2 = P0 + fu - fv;
	Vector3f P3 = P0 + fu + fv;
	Vector3f P4 = P0 - fu + fv;

  gpu.drawVector(P0, 0.2*m_n, Color(1,1,1,1));
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_POLYGON);
	glColor4f(1.0, 0.0, 0.0, 0.1);
	qDebug() << m_n << "  " << m_d;
//	glNormal3f(m_n[0],m_n[1],m_n[2]);

	glVertex3f(P1[0], P1[1], P1[2]);
	glVertex3f(P2[0], P2[1], P2[2]);
	glVertex3f(P3[0], P3[1], P3[2]);
	glVertex3f(P4[0], P4[1], P4[2]);
	glEnd();
	glDisable(GL_BLEND);
}

void SDebugMngr::DrawPlane(const Vector3f& n, float d)
{
	SDebugPlane* p = new SDebugPlane();
	p->m_n = n;
	p->m_d = d;

	m_list.push_front(p);
}

SDebugMngr* DebugManager()
{
	static SDebugMngr* sDbgMngr = 0;
	if (0 == sDbgMngr)
		sDbgMngr = new SDebugMngr;

	return sDbgMngr;
}

void SDebugMngr::Draw()
{
	while (!m_list.empty())
	{
		IDebugItem* i = m_list.front();
		i->Draw();
		m_list.pop_front();
	}
}
