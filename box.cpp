#include "box.h"
#include "gpuhelper.h"
#include <QtDebug>


Box::Box()
{
	m_a = 15;
	m_b = 30;
	m_c = 45;

	Matrix3f J;
	float a = 2.f/5.f / m_minv * m_a*m_a;
	J <<	a, 0, 0,
			0, a, 0,
			0, 0, a;

	m_Jinv = J.inverse();
	//TODO: define inertia tensor for box
}


void Box::Step(float t)
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

void Box::AddImpulse(Vector3f value, Vector3f pt)
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

void Box::AddAngularImpulse(Vector3f value)
{
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return;
	}

	Vector3f dw = m_Jinv * value;
	m_w += dw;

}

void Box::Draw()
{
     glEnable(GL_NORMALIZE);
     Affine3f t = Translation3f(m_pos) * m_rot * Scaling(m_a, m_b, m_c);
     gpu.pushMatrix(GL_MODELVIEW);
     gpu.multMatrix(t.matrix(),GL_MODELVIEW);
    
     // Многоцветная сторона - ПЕРЕДНЯЯ
	glBegin(GL_POLYGON);
	 
	glColor3f( 1.0, 0.0, 0.0 );     glVertex3f(  0.5, -0.5, -0.5 );      // P1 is red
	glColor3f( 0.0, 1.0, 0.0 );     glVertex3f(  0.5,  0.5, -0.5 );      // P2 is green
	glColor3f( 0.0, 0.0, 1.0 );     glVertex3f( -0.5,  0.5, -0.5 );      // P3 is blue
	glColor3f( 1.0, 0.0, 1.0 );     glVertex3f( -0.5, -0.5, -0.5 );      // P4 is purple
	 
	glEnd();

	// Белая сторона - ЗАДНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  1.0, 1.0 );
	glVertex3f(  0.5, -0.5, 0.5 );
	glVertex3f(  0.5,  0.5, 0.5 );
	glVertex3f( -0.5,  0.5, 0.5 );
	glVertex3f( -0.5, -0.5, 0.5 );
	glEnd();
	 
	// Фиолетовая сторона - ПРАВАЯ
	glBegin(GL_POLYGON);
	glColor3f(  1.0,  0.0,  1.0 );
	glVertex3f( 0.5, -0.5, -0.5 );
	glVertex3f( 0.5,  0.5, -0.5 );
	glVertex3f( 0.5,  0.5,  0.5 );
	glVertex3f( 0.5, -0.5,  0.5 );
	glEnd();
	 
	// Зеленая сторона - ЛЕВАЯ
	glBegin(GL_POLYGON);
	glColor3f(   0.0,  1.0,  0.0 );
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();
	 
	// Синяя сторона - ВЕРХНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   0.0,  0.0,  1.0 );
	glVertex3f(  0.5,  0.5,  0.5 );
	glVertex3f(  0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glEnd();
	 
	// Красная сторона - НИЖНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  0.0,  0.0 );
	glVertex3f(  0.5, -0.5, -0.5 );
	glVertex3f(  0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();


     gpu.popMatrix(GL_MODELVIEW);
     glDisable(GL_NORMALIZE);
}
