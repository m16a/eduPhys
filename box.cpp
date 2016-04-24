#include "box.h"
#include "gpuhelper.h"
#include <QtDebug>
#include "my_utils.h"
#include "my_eulerAngles.h"
#include "rwi.h"

Box::Box()
{
	m_a = 10;
	m_b = 10;
	m_c = 0.4;

	Matrix3f J;
	float a = 1.f/12.f / m_minv * (m_b*m_b + m_c*m_c);

	float b = 1.f/12.f / m_minv * (m_a*m_a + m_c*m_c);
	float c = 1.f/12.f / m_minv * (m_b*m_b + m_a*m_a);
	J <<	a, 0, 0,
			0, b, 0,
			0, 0, c;

	m_Jinv = J.inverse();

//infinite inertia tensor
#if 1
	m_Jinv <<	0, 0, 0,
						0, 0, 0,
						0, 0, 0;
#endif

}


void Box::Step(float t)
{
	assert(m_id > 0);
	//qDebug() << "step " << t ;
	
	if (!m_active)
		return;

	//Symplectic Euler integration
	Vector3f f_sum(0,0,0);
	for (int i=0; i<m_forces.size(); ++i)
		f_sum += m_forces[i];

	if (t>=0)
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
	}else
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

/*		
		qDebug() << "m_pos: " << m_pos.x() << " " << m_pos.y() << " " << m_pos.z();	
		qDebug() << "nn: " << n.x() << " " << n.y() << " " << n.z();	
		qDebug() << "impulse: " << value.x() << " " << value.y() << " " << value.z();	
	
		qDebug() << "normal impulse: " << normImpulse.x() << " " <<  normImpulse.y() << " " <<	 normImpulse.z();	
*/
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
//	qDebug() << "angular impulse " << value.norm();
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small impulse";
		return;
	}

	Vector3f dw = m_Jinv * value;
	m_w += dw;
//	qDebug() << "sphere rotation was added";

}

int Box::IntersectRay(const SRay& r, SRayHit& out_hit)
{
	int res = 0;



	return res;
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
	glNormal3f(0,0,-1);

	glColor3f( 0.0, 1.0, 0.0 );     glVertex3f(  0.5,  0.5, -0.5 );      // P2 is green
	glNormal3f(0,0,-1);

	glColor3f( 0.0, 0.0, 1.0 );     glVertex3f( -0.5,  0.5, -0.5 );      // P3 is blue
	glNormal3f(0,0,-1);

	glColor3f( 1.0, 0.0, 1.0 );     glVertex3f( -0.5, -0.5, -0.5 );      // P4 is purple
	glNormal3f(0,0,-1);

	glEnd();

	// Белая сторона - ЗАДНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  1.0, 1.0 );
	glVertex3f(  0.5, -0.5, 0.5 );
	glNormal3f(0,0,1);
	glVertex3f(  0.5,  0.5, 0.5 );
	glNormal3f(0,0,1);

	glVertex3f( -0.5,  0.5, 0.5 );
	glNormal3f(0,0,1);

	glVertex3f( -0.5, -0.5, 0.5 );
	glNormal3f(0,0,1);

	glEnd();
	 
	// Фиолетовая сторона - ПРАВАЯ
	glBegin(GL_POLYGON);
	glColor3f(  1.0,  0.0,  1.0 );
	glNormal3f(1,0,0);

	glVertex3f( 0.5, -0.5, -0.5 );
	glVertex3f( 0.5,  0.5, -0.5 );
	glVertex3f( 0.5,  0.5,  0.5 );
	glVertex3f( 0.5, -0.5,  0.5 );
	glEnd();
	 
	// Зеленая сторона - ЛЕВАЯ
	glBegin(GL_POLYGON);
	glNormal3f(-1,0,0);
	glColor3f(   0.0,  1.0,  0.0 );
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();
	 
	// Синяя сторона - ВЕРХНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   0.0,  0.0,  1.0 );
	glNormal3f(0,1,0);
	glVertex3f(  0.5,  0.5,  0.5 );
	glVertex3f(  0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glEnd();
	 
	// Красная сторона - НИЖНЯЯ
	glBegin(GL_POLYGON);
	glColor3f(   1.0,  0.0,  0.0 );
	glNormal3f(0,-1,0);
	glVertex3f(  0.5, -0.5, -0.5 );
	glVertex3f(  0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();


     gpu.popMatrix(GL_MODELVIEW);
     glDisable(GL_NORMALIZE);
}
