
#ifndef _COLLISION_H_
#define _COLLISION_H_


#include "sphere.h"

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


#endif//_COLLISION_H_