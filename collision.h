
#ifndef _COLLISION_H_
#define _COLLISION_H_


#include "sphere.h"
#include "box.h"
#include <Eigen/Geometry>

using Eigen::Vector3d;
using Eigen::Vector3f;

struct Contact
{
	Vector3f pt;
	Vector3f n;
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
		out_size = 1;
	}
}

bool overlap(Sphere* a, Box* b)
{
	Vector3f s = b->m_rot * (a->m_pos - b->m_pos);

	if (s.x() < 0)
		s.x() += b->m_a/2;
	else if (s.x() > 0)
		s.x() -= b->m_a/2;

	if (s.y() < 0)
		s.y() += b->m_b/2;
	else if (s.y() > 0)
		s.y() -= b->m_b/2;

	if (s.z() < 0)
		s.z() += b->m_c/2;
	else if (s.z() > 0)
		s.z() -= b->m_c/2;

	bool res = s.dot(s) <= a->m_r*a->m_r;

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

	Vector3f s = b->m_rot * (sphere->m_pos - b->m_pos);
	qDebug() << "internal s " << (s).x() << " " << (s).y() <<" " << (s).z(); 

	Vector3f dir = Vector3f(0.0f,0.0f,0.0f);

	dir.x() = std::min(0.0f, b->m_a/2.0f - abs(s.x())) * ((s.x() >= 0.0f) ? 1.0f : -1.0f);
	dir.y() = std::min(0.0f, b->m_b/2.0f - abs(s.y())) * ((s.y() >= 0.0f) ? 1.0f : -1.0f);
	dir.z() = std::min(0.0f, b->m_c/2.0f - abs(s.z())) * ((s.z() >= 0.0f) ? 1.0f : -1.0f);
	qDebug() << "internal dir " << (dir).x() << " " << (dir).y() <<" " << (dir).z(); 

	out_size = 1;
	qDebug() << "internal coll " << (s+dir).x() << " " << (s+dir).y() <<" " << (s+dir).z(); 
	c->pt = b->m_rot*(s + dir) + b->m_pos;
	c->n = c->pt - sphere->m_pos;
	c->n.normalize();	
}


#endif//_COLLISION_H_