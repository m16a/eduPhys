#include "debug_draw.h"
#include <QtDebug>
#include "my_utils.h"
#include "gpuhelper.h"
#include "rwi.h"

IcoSphere SDebugSphere::m_icoSphere = IcoSphere();

SDebugPlane::SDebugPlane(const Vector3f& point, const Vector3f& normal)
{
	m_n = normal;
	m_d = - m_n.dot(point);
}

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
	float  f  = 6;  // large enough
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
	glColor4f(1.0, 0.0, 0.0, 0.8);
//	glNormal3f(m_n[0],m_n[1],m_n[2]);

	glVertex3f(P1[0], P1[1], P1[2]);
	glVertex3f(P2[0], P2[1], P2[2]);
	glVertex3f(P3[0], P3[1], P3[2]);
	glVertex3f(P4[0], P4[1], P4[2]);
	glEnd();
	glDisable(GL_BLEND);
}

int SDebugPlane::IntersectRay(const SRay& r, SRayHit& out_hit)
{
	int res = 0;
	Vector3f planesPoint(0,0,0);

	if (m_n[0] > 0.001)
		planesPoint[0] = - m_d / m_n[0];
	else if (m_n[1] > 0.001)
		planesPoint[1] = - m_d / m_n[1];
	else if (m_n[2] > 0.001)
		planesPoint[2] = - m_d / m_n[2];
	//qDebug() << "point on plane:" << planesPoint << "normal" << m_n;

	float t = (planesPoint - r.m_org).dot(m_n) / m_n.dot(r.m_dir);

	if (t >0)
	{
		out_hit.m_pt = r.m_org + r.m_dir * t;	
		res = 1;
	}
	else
	{
	//	assert(0);//don't count back collision for now
	qWarning()<< "Back collision, t:"<<t;
	}

	return res;
}

void SDebugVector::Draw()
{
  gpu.drawVector(m_pos, m_len*m_dir, Color(1,0,0,1));
}

void SDebugMngr::DrawPlane(const Vector3f& n, float d)
{
	SDebugPlane* p = new SDebugPlane();
	p->m_n = n;
	p->m_d = d;

	m_list.push_front(p);
}

void SDebugSphere::Draw()
{
	glEnable(GL_NORMALIZE);
	Affine3f t = Translation3f(m_pos) * Scaling(m_r);
	glColor4fv(m_color.data());
	gpu.pushMatrix(GL_MODELVIEW);
	gpu.multMatrix(t.matrix(),GL_MODELVIEW);
	m_icoSphere.draw(2);
	gpu.popMatrix(GL_MODELVIEW);
	glDisable(GL_NORMALIZE);
}

void SDebugMngr::DrawSphere(const Vector3f& pos, float r, const Color& col)
{
	SDebugSphere* v = new SDebugSphere();
	v->m_pos = pos;
	v->m_r = r;
	v->m_color = col;
	m_list.push_front(v);
}

void SDebugMngr::DrawVector(const Vector3f& pos, const Vector3f& dir, float len)
{
	SDebugVector* v = new SDebugVector();
	v->m_pos = pos;
	v->m_dir = dir;
	v->m_len = len;
	m_list.push_front(v);
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
	std::list<IDebugItem*>::iterator it = m_list.begin();
	while (it != m_list.end())
	{
		(*it)->Draw();
		++it;
	}
}

void SDebugMngr::Clear()
{
	for (std::list<IDebugItem*>::iterator it = m_list.begin() ; it != m_list.end(); ++it)
	{
	 delete (*it);
	} 
	m_list.clear();
}
