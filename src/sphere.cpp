#include "sphere.h"
#ifdef WIN32
#include <windows.h>
#endif
#include "gpuhelper.h"
#include <QtDebug>
#include "rwi.h"


IcoSphere Sphere::m_icoSphere = IcoSphere();

Sphere::Sphere(bool isStatic):IPhysEnt(isStatic)
{
	m_r = 0.2;

	if (!isStatic)
	{
		Matrix3f J;
		float a = 2.f/5.f / m_minv * m_r*m_r;
		J <<	a, 0, 0,
				0, a, 0,
				0, 0, a;
		m_Jinv = J.inverse();
	}
}


void Sphere::Step(float t)
{
	assert(m_id > 0);
	//qDebug() << "step " << t ;
	if (!m_active)
		return;
	//Symplectic Euler integration
	Vector3f f_sum = m_extForce;
	if (m_isGravity)
		f_sum += g_Gravity;

	if (t >= 0)
	{
		m_v += f_sum * m_minv * t;
		m_pos += m_v * t;

		Vector3f dw = m_w*t;
		float n = dw.norm();
		if (n > 0)
		{
			Vector3f r = dw / n * sin(n/2.f) ;
			Quaternionf dq(cos(n/2.f), r.x(), r.y(), r.z());
			m_rot *= dq;
		}
	}
	else
	{
		Vector3f dw = m_w*t;
		float n = dw.norm();
		if (n > 0)
		{
			Vector3f r = dw / n * sin(n/2.f) ;
			Quaternionf dq(cos(n/2.f), r.x(), r.y(), r.z());
			m_rot *= dq;
		}

		m_pos += m_v * t;
		m_v += f_sum * m_minv * t;
	}
	
	UpdatedPosRot();	

//	qDebug() << m_pos.x() << " " << m_pos.y() << " " << m_pos.z();
}

bool Sphere::AddImpulse(const Vector3f& value, const Vector3f& pt)
{
	if (!IPhysEnt::AddImpulse(value, pt))
		return false;

	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return false;
	}
	
	if (pt.norm() > 0.0001)
	{
		Vector3f n = (m_pos - pt);
		n.normalize();

		Vector3f normImpulse = (n.dot(value)) * n;

		AddAngularImpulse((pt - m_pos).cross(value - normImpulse));

		Vector3f dv = normImpulse * m_minv;
		m_v += dv;

	}
	else //impulse to the center of the mass
	{
		Vector3f dv = value * m_minv;
		m_v += dv;
	}
	return true;
}

void Sphere::AddAngularImpulse(const Vector3f& value)
{
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return;
	}

	Vector3f dw = m_Jinv * value;
	m_w += dw;
}

int Sphere::IntersectRay(const SRay& r, SRayHit& out_hit)
{
	//TODO: take into accoun r.m_dist
	int res = 0;
	
	float t,t0,t1;
	
	Vector3f L = m_pos - r.m_org;
	float tca = L.dot(r.m_dir);
	if (tca < 0) return res;
	float d2 = L.dot(L) - tca*tca;
	if (d2 > m_r*m_r) return res;
	float thc = sqrt(m_r*m_r - d2);
	t0 = tca - thc; 
  t1 = tca + thc; 
  if (t0 > t1) std::swap(t0, t1); 
	if (t0 < 0)
	{ 
			t0 = t1; // if t0 is negative, let's use t1 instead 
			if (t0 < 0) return res; // both t0 and t1 are negative 
	} 

	t = t0;
	res = 1;

	out_hit.m_pt = r.m_org + t * r.m_dir;
	out_hit.m_n = out_hit.m_pt - m_pos;
	out_hit.m_n.normalize();
	out_hit.m_dist = t;
	out_hit.m_pEnt = this;

	return res;
}

void Sphere::UpdatedPosRot()
{
	assert(!"Implement BBox for sphere");
}

void Sphere::Draw()
{
     glEnable(GL_NORMALIZE);
     Affine3f t = Translation3f(m_pos) * m_rot * Scaling(m_r);
     gpu.pushMatrix(GL_MODELVIEW);
     gpu.multMatrix(t.matrix(),GL_MODELVIEW);
     m_icoSphere.draw(2);
     gpu.popMatrix(GL_MODELVIEW);
     glDisable(GL_NORMALIZE);
}

void Sphere::Serialize(ser::SerPhys* sp)
{
	ser::Sphere* b = sp->MutableExtension(ser::Sphere::sphere);
	sp->set_type(ser::SerPhys::Sphere);
	b->set_r(m_r); 
	IPhysEnt::Serialize(sp);
}

void Sphere::Deserialize(const ser::SerPhys* sp)
{
	ser::Sphere b = sp->GetExtension(ser::Sphere::sphere);
	m_r = b.r();
	IPhysEnt::Deserialize(sp);
}
