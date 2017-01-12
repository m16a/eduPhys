#include "box.h"
#include "gpuhelper.h"
#include <QtDebug>
#include "my_utils.h"
#include "my_eulerAngles.h"
#include "rwi.h"

Box::Box(const Vector3f& size, bool isStatic):IPhysEnt(isStatic)
{
	m_size = size;
	if (!isStatic)
	{
		Matrix3f J;
		float a = 1.f/12.f / m_minv * (m_size[1]*m_size[1] + m_size[2]*m_size[2]);
		float b = 1.f/12.f / m_minv * (m_size[0]*m_size[0] + m_size[2]*m_size[2]);
		float c = 1.f/12.f / m_minv * (m_size[1]*m_size[1] + m_size[0]*m_size[0]);

		J <<	a, 0, 0,
					0, b, 0,
					0, 0, c;

		//qDebug() << J;
		m_Jinv = J.inverse();
		//qDebug() << m_Jinv;;
	}
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

float Box::CalcKineticEnergy()
{
	//TODO: count zero inertia tensor too
	if (m_minv == .0f)
		return 0.0f;

	Matrix3f rotM1 = m_rot.toRotationMatrix();
	Matrix3f J1 = rotM1 * m_Jinv.inverse() * rotM1.transpose(); 

	return 0.5f * (m_v.dot(m_v) / m_minv + (J1 * m_w).dot(m_w));
}

void Box::AddImpulse(Vector3f value, Vector3f pt)
{

	qDebug() << "Impulse box:" << m_id << "pt:" << pt << "impulse:" << value;
	if (value.norm() < 0.0001)
	{
		qDebug() << "Small norm impulse";
		return;
	}
	

	if ( pt.norm() > 0.0001)
	{
/*		
		qDebug() << "m_pos: " << m_pos.x() << " " << m_pos.y() << " " << m_pos.z();	
		qDebug() << "nn: " << n.x() << " " << n.y() << " " << n.z();	
		qDebug() << "impulse: " << value.x() << " " << value.y() << " " << value.z();	
*/
		AddAngularImpulse((pt - m_pos).cross(value));

		Vector3f dv = value * m_minv;
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
		qDebug() << "Small angular impulse";
		return;
	}

	Matrix3f rotM1 = m_rot.toRotationMatrix();
	Matrix3f invJ1 = rotM1 * m_Jinv * rotM1.transpose(); 
	Vector3f dw = invJ1 * value;
	m_w += dw;
//	qDebug() << "sphere rotation was added";

}

struct AABB
{
	Vector3f vMin;
	Vector3f vMax;
};

bool ClipLine(int d, const AABB& aabbBox, const Vector3f& v0, const Vector3f& v1, float& f_low, float& f_high)
{
	// f_low and f_high are the results from all clipping so far. We'll write our results back out to those parameters.

	// f_dim_low and f_dim_high are the results we're calculating for this current dimension.
	float f_dim_low, f_dim_high;

	// Find the point of intersection in this dimension only as a fraction of the total vector http://youtu.be/USjbg5QXk3g?t=3m12s
	f_dim_low = (aabbBox.vMin[d] - v0[d])/(v1[d] - v0[d]);
	f_dim_high = (aabbBox.vMax[d] - v0[d])/(v1[d] - v0[d]);

	// Make sure low is less than high
	if (f_dim_high < f_dim_low)
		std::swap(f_dim_high, f_dim_low);

	// If this dimension's high is less than the low we got then we definitely missed. http://youtu.be/USjbg5QXk3g?t=7m16s
	if (f_dim_high < f_low)
		return false;

	// Likewise if the low is less than the high.
	if (f_dim_low > f_high)
		return false;

	// Add the clip from this dimension to the previous results http://youtu.be/USjbg5QXk3g?t=5m32s
	f_low = std::max(f_dim_low, f_low);
	f_high = std::min(f_dim_high, f_high);

	if (f_low > f_high)
		return false;

	return true;
}

// Find the intersection of a line from v0 to v1 and an axis-aligned bounding box http://www.youtube.com/watch?v=USjbg5QXk3g
bool LineAABBIntersection(const AABB& aabbBox, const Vector3f& v0, const Vector3f& v1, Vector3f& vecIntersection, float& flFraction)
{
	float f_low = 0;
	float f_high = 1;

	if (!ClipLine(0, aabbBox, v0, v1, f_low, f_high))
		return false;	
	
	if (!ClipLine(1, aabbBox, v0, v1, f_low, f_high))
		return false;

	if (!ClipLine(2, aabbBox, v0, v1, f_low, f_high))
		return false;

	// The formula for I: http://youtu.be/USjbg5QXk3g?t=6m24s
	Vector3f b = v1 - v0;
	vecIntersection = v0 + b * f_low;

	flFraction = f_low;

	return true;
}

int Box::IntersectRay(const SRay& r, SRayHit& out_hit)
{
	int res = 0;

	AABB aabbBox;
	aabbBox.vMin = Vector3f(-m_size[0]/2, -m_size[1]/2, -m_size[2]/2);
	aabbBox.vMax = Vector3f(m_size[0]/2, m_size[1]/2, m_size[2]/2);

	Quaternionf sT = (m_rot).conjugate();
	SRay trans_r = r;
	trans_r.m_org = sT * (r.m_org - m_pos);	
	trans_r.m_dir = sT * r.m_dir;	
	//qDebug()<< "box_rwi:" << r.m_org << " " << trans_r.m_org;	
	if (LineAABBIntersection(aabbBox, trans_r.m_org, trans_r.m_org + trans_r.m_dir * trans_r.m_dist, out_hit.m_pt, out_hit.m_dist))
	{
		res = 1;
		out_hit.m_pEnt = this;
		out_hit.m_dist *= r.m_dist;
		out_hit.m_pt = m_rot * out_hit.m_pt + m_pos;
	}

	return res;
}

void Box::Draw()
{
	glEnable(GL_NORMALIZE);
	Affine3f t = Translation3f(m_pos) * m_rot * Scaling(m_size[0], m_size[1], m_size[2]);
	gpu.pushMatrix(GL_MODELVIEW);
	gpu.multMatrix(t.matrix(),GL_MODELVIEW);

	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);	
	// Многоцветная сторона - ПЕРЕДНЯЯ
	glBegin(GL_POLYGON);
	 
//	glColor3f(232/255.0f,209/255.0f,116/255.0f);
	glColor3f(51/255.0f,51/255.0f,51/255.0f);
  glVertex3f(  0.5, -0.5, -0.5 );      // P1 is red
	glNormal3f(0,0,-1);
  glVertex3f(  0.5,  0.5, -0.5 ); 
  glVertex3f( -0.5,  0.5, -0.5 );      // P3 is blue
	glVertex3f( -0.5, -0.5, -0.5 );      // P4 is purple
	glEnd();

	// Белая сторона - ЗАДНЯЯ
	glBegin(GL_POLYGON);
//	glColor3f(227/255.0f,158/255.0f,84/255.0f);
	glNormal3f(0,0,1);
	glVertex3f(  0.5, -0.5, 0.5 );
	glVertex3f(  0.5,  0.5, 0.5 );
	glVertex3f( -0.5,  0.5, 0.5 );

	glVertex3f( -0.5, -0.5, 0.5 );

	glEnd();
	 
	// Фиолетовая сторона - ПРАВАЯ
//	glColor3f(77/255.0f,115/255.0f,88/255.0f);
	glBegin(GL_POLYGON);
	glNormal3f(1,0,0);
	glVertex3f( 0.5, -0.5, -0.5 );
	glVertex3f( 0.5,  0.5, -0.5 );
	glVertex3f( 0.5,  0.5,  0.5 );
	glVertex3f( 0.5, -0.5,  0.5 );
	glEnd();
	 
	// Зеленая сторона - ЛЕВАЯ
//	glColor3f(214/255.0f,77/255.0f,77/255.0f);
	glBegin(GL_POLYGON);
	glNormal3f(-1,0,0);
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();
	 
	// Синяя сторона - ВЕРХНЯЯ
//	glColor3f(158/255.0f,214/255.0f,112/255.0f);
	glBegin(GL_POLYGON);
	glNormal3f(0,1,0);
	glVertex3f(  0.5,  0.5,  0.5 );
	glVertex3f(  0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5, -0.5 );
	glVertex3f( -0.5,  0.5,  0.5 );
	glEnd();
	 
	// Красная сторона - НИЖНЯЯ
//	glColor3f(158/255.0f,214/255.0f,112/255.0f);
	glBegin(GL_POLYGON);
	glNormal3f(0,-1,0);
	glVertex3f(  0.5, -0.5, -0.5 );
	glVertex3f(  0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5,  0.5 );
	glVertex3f( -0.5, -0.5, -0.5 );
	glEnd();
	
	gpu.popMatrix(GL_MODELVIEW);
	glDisable(GL_NORMALIZE);
}

void getBoxVerticies(const Box* b, Vector3f out_arr[8])
{
	int indx = 0;
	for (int i=-1; i<2; i+=2)
	for (int j=-1; j<2; j+=2)
	for (int k=-1; k<2; k+=2)
	{	
		Vector3f v_wrld = b->m_pos + b->m_rot * (Vector3f(i*b->m_size[0]/2.0f, j*b->m_size[1]/2.0f, k*b->m_size[2]/2.0f));
		out_arr[indx++] = v_wrld;
	}
}

void Box::Serialize(ser::SerPhys* sp)
{
	ser::Box* b = sp->MutableExtension(ser::Box::box);
	sp->set_type(ser::SerPhys::Box);
	ser::Vector3f* size = b->mutable_size(); 
	size->set_x(m_size[0]);
	size->set_y(m_size[1]);
	size->set_z(m_size[2]);
	IPhysEnt::Serialize(sp);
}

void Box::Deserialize(const ser::SerPhys* sp)
{
	ser::Box b = sp->GetExtension(ser::Box::box);
	ser::Vector3f size = b.size();
	m_size[0] = size.x();
	m_size[1] = size.y();
	m_size[2] = size.z();
	IPhysEnt::Deserialize(sp);
}
