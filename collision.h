
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

//iterate through 6 planes
	
/*
	------
	|    |
left|    |right
	|    |
	|    |
	------
*/

	Vector3f ns[] = {	Vector3f(1,0,0), Vector3f(-1,0,0),
						Vector3f(0,1,0), Vector3f(0,-1,0),
						Vector3f(0,0,1), Vector3f(0,0,-1)};

	Vector3f cc;//collision_candidat
	bool isColl = false;
	do{
		//right
		float d = -1.0f*b->m_a/2.0f;
		float dist = distPointPlane(ns[0], d, s);

		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[0];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

		//left
		d = -1.0f*b->m_a/2.0f;
		dist = distPointPlane(ns[1], d, s);
//		qDebug() << "dist:" << dist << " sphereX:" << sphere->m_pos.x();

		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[1];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

		//front
		d = -1.0f*b->m_b/2.0f;
		dist = distPointPlane(ns[2], d, s);
		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[2];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

		//back
		d = -1.0f*b->m_b/2.0f;
		dist = distPointPlane(ns[3], d, s);
		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[3];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

		//top
		d = -1.0f*b->m_c/2.0f;
		dist = distPointPlane(ns[4], d, s);
		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[4];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

		//bottom
		d = -1.0f*b->m_c/2.0f;
		dist = distPointPlane(ns[5], d, s);
		if (dist < sphere->m_r)
		{
			cc = sphere->m_pos - sphere->m_r * ns[5];
			if (isPointInsideBox(b->Size(), cc))
			{
				isColl = true;
				break;
			}
		}

	}while(0);
//TODO: provide collision with corner points
	if (isColl)
	{
		out_size = 0;
		c->n = cc - sphere->m_pos;
		c->n.normalize();
		c->pt = cc;
		//collision
		qDebug() << "COLLISION";
	}

	
}


#endif//_COLLISION_H_