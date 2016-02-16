#include "core.h"
#include "sphere.h"
#include <QtDebug>
#include "collision.h"

#include "my_utils.h"

Core::Core()
{


}

void Core::Step(float t)
{
	//qDebug() << "step " << t ;
/*
//collide
	int size = m_objects.size();
	for (int i = 0; i < size; ++i)
		for (int j = 0; j < size; ++j)
		{
			Sphere *a = static_cast<Sphere*>(m_objects[i]);
			Sphere *b = static_cast<Sphere*>(m_objects[j]);

			if (a->m_id < b->m_id)
			{
				Contact c[1];
				int s = 1;
				collide(a, b, c, s);

				if (s > 0)
				{
					qDebug() << "COLLISION!!!" << c[0].pt.x() << " "<< c[0].pt.y();
					
					Vector3f rAP = c[0].pt - a->m_pos;
					Vector3f rBP = c[0].pt - b->m_pos;

					Matrix3f rAPcross = getCrossMatrix(rAP);
					Matrix3f rBPcross = getCrossMatrix(rBP);
					

					float p = (a->m_v - b->m_v).dot(c[0].n) / 
						(a->m_minv + b->m_minv + (rAPcross*a->m_Jinv*rAPcross * c[0].n).dot(c[0].n)
											   + (rBPcross*b->m_Jinv*rBPcross * c[0].n).dot(c[0].n)
						);

					a->AddImpulse(-2.0*p * c[0].n);
					b->AddImpulse(2.0*p * c[0].n);
				}			
			}
		}
*/

//step
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
	{
		(*it)->Step(t);
	}
}

void Core::Draw()
{
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();

	for (; it != m_objects.end(); ++it)
	{
		(*it)->Draw();
	}	
}