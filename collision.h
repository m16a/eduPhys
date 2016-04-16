
#ifndef _COLLISION_H_
#define _COLLISION_H_


#include "sphere.h"
#include "box.h"
#include <Eigen/Geometry>
#include "my_utils.h"

using Eigen::Vector3d;
using Eigen::Vector3f;

struct Contact
{
	Vector3f pt;
	Vector3f n;
	float depth;

	Contact():depth(1000.0f){};
};


void collide(Sphere* a, Sphere* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;
	Vector3f diff = b->m_pos - a->m_pos;
	if (diff.norm() < a->m_r + b->m_r)
	{
		c->n = diff;
		c->pt = a->m_pos + c->n * a->m_r / (a->m_r + b->m_r);
		c->n.normalize();
		c->depth = (c->pt - a->m_pos).norm() - a->m_r;
		out_size = 1;
	}
}

bool overlap(Sphere* a, Box* b)
{
	Quaternionf sT = (b->m_rot).conjugate();
	Vector3f s = sT * (a->m_pos - b->m_pos);
	float d = 0.0f;
	float tmp = 0.0f;

	if (s.x() < -b->m_a/2)
	{
		tmp = s.x() - (-b->m_a/2);
		d += tmp*tmp;
	}
	else if (s.x() > b->m_a/2)
	{
		tmp = s.x() - b->m_a/2;
		d += tmp*tmp;
	}

	if (s.y() < -b->m_b/2)
	{
		tmp = s.y() - (-b->m_b/2);
		d += tmp*tmp;
	}
	else if (s.y() > b->m_b/2)
	{
		tmp = s.y() - b->m_b/2;
		d += tmp*tmp;
	}

	if (s.z() < -b->m_c/2)
	{
		tmp = s.z() - (-b->m_c/2);
		d += tmp*tmp;
	}
	else if (s.z() > b->m_c/2)
	{
		tmp = s.z() - b->m_c/2;
		d += tmp*tmp;
	}
	
	bool res = d <= a->m_r*a->m_r;

//qDebug() << "overlap test " << a->m_id << " " << b->m_id << " " << s << "\n" << 
//	"res d:"<< d << " r^2:" << a->m_r*a->m_r;
	if (res)
		qDebug() << "OVERLAP";

	return res;
}

float distPointPlane(Vector3f plane_normal, float d, Vector3f point)
{
	return (plane_normal.dot(point) + d)/plane_normal.norm();
}

bool isPointInsideBox(const Vector3f size, const Vector3f p)
{
	if (abs(p.x()) <= size.x()/2.0f &&
		abs(p.y()) <= size.y()/2.0f &&
		abs(p.z()) <= size.z()/2.0f)
		return true;

	return false;
}

void collide(Sphere* sphere, Box* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;

	if (!overlap(sphere,b))
		return;

	Quaternionf sT = (b->m_rot).conjugate();
	Vector3f s = sT * (sphere->m_pos - b->m_pos);
//	qDebug() << "internal s " << (s).x() << " " << (s).y() <<" " << (s).z(); 

	Vector3f dir = Vector3f(0.0f,0.0f,0.0f);

	dir.x() = std::min(0.0f, b->m_a/2.0f - (float)fabs(s.x())) * ((s.x() >= 0.0f) ? 1.0f : -1.0f);
	dir.y() = std::min(0.0f, b->m_b/2.0f - (float)fabs(s.y())) * ((s.y() >= 0.0f) ? 1.0f : -1.0f);
	dir.z() = std::min(0.0f, b->m_c/2.0f - (float)fabs(s.z())) * ((s.z() >= 0.0f) ? 1.0f : -1.0f);
//	qDebug() << "internal dir " << (dir).x() << " " << (dir).y() <<" " << (dir).z(); 

	out_size = 1;
//	qDebug() << "internal coll " << (s+dir).x() << " " << (s+dir).y() <<" " << (s+dir).z(); 
	c->pt = b->m_rot*(s + dir) + b->m_pos;
	c->n = c->pt - sphere->m_pos;
	assert(c->n.norm() > 10e-5);
	c->depth = (c->pt - sphere->m_pos).norm() - sphere->m_r;
	c->n.normalize();	
}


#endif//_COLLISION_H_
