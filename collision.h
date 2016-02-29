
#ifndef _COLLISION_H_
#define _COLLISION_H_


#include "sphere.h"
#include "box.h"

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

return s.dot(s) <= a->m_r*a->m_r;

}


void collide(Sphere* a, Box* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;

	if (overlap(a,b))
		qDebug() << "OVERLAP";
}


#endif//_COLLISION_H_