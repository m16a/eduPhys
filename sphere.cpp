#include "sphere.h"
#include "gpuhelper.h"
#include <QtDebug>

IcoSphere Sphere::m_icoSphere = IcoSphere();

Sphere::Sphere()
{
	m_pos = Vector3f(0.f, 0.f, 0.f);
	m_rot = Quaternionf::Identity();
	m_r = 25;

	m_v = Vector3f(0.f, 0.f, 0.f);
	m_w = Vector3f(0.f, 0.f, 0.f);
	m_minv = 0.01;//100kg

	Matrix3f J;
	float a = 2.f/5.f / m_minv * m_r*m_r;
	J <<	a, 0, 0,
			0, a, 0,
			0, 0, a;

	m_Jinv = J.inverse();
	m_id = 0;
}


void Sphere::Step(float t)
{
	assert(m_id > 0);
	//qDebug() << "step " << t ;

	m_pos += m_v * t;

	Vector3f dw = m_w*t;
	float n = dw.norm();
	if (n > 0)
	{
		Vector3f r = dw / n * sin(n/2.f) ;
		Quaternionf dq(cos(n/2.f), r.x(), r.y(), r.z());
		m_rot *= dq;
	/*
	qDebug() 	<< dq.w() << " " 
					<< dq.x() << " " 
					<< dq.y() << " " 
					<< dq.z(); 
	*/
					/*
	qDebug() 	<< m_rot.w() << " " 
				<< m_rot.x() << " " 
				<< m_rot.y() << " " 
				<< m_rot.z();
	*/
	
	}
//	qDebug() << m_pos.x() << " " << m_pos.y() << " " << m_pos.z();
}

void Sphere::AddImpulse(Vector3f value, Vector3f pt)
{
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return;
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
}

void Sphere::AddAngularImpulse(Vector3f value)
{
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return;
	}

	Vector3f dw = m_Jinv * value;
	m_w += dw;

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
