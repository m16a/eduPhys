#include "core.h"
#include "sphere.h"
#include "box.h"
#include <QtDebug>
#include "collision.h"

#include "my_utils.h"

Core::Core()
{


}

void Core::Step(float t)
{
	//qDebug() << "step " << t ;

//collide
	int size = m_objects.size();
	for (int i = 0; i < size; ++i)
		for (int j = 0; j < size; ++j)
		{

			if (m_objects[i]->m_id < m_objects[j]->m_id)
			{

				IPhysEnt *a = m_objects[i];
				IPhysEnt *b = m_objects[j];

				Contact c[1];
				int s = 1;

				//TODO: ugly, refactor on adding new collision geom
				if (Box* a1 = dynamic_cast<Box*>(a))
				{
					if (Sphere* b1 = dynamic_cast<Sphere*>(b))
					{
						collide(b1, a1, c, s);
						a = b1; b = a1;
					}
				}else if (Sphere* a1 = dynamic_cast<Sphere*>(a))
				{
					if (Box* b1 = dynamic_cast<Box*>(b))
					{
						collide(a1, b1, c, s);
						a = a1; b = b1;

					}
					else if (Sphere* b1 = dynamic_cast<Sphere*>(b))
 					{	
 						collide(a1, b1, c, s);
						a = a1; b = b1;
					}

				}

				if (s > 0)
				{					
					Vector3f rAP = c[0].pt - a->m_pos;
					Vector3f rBP = c[0].pt - b->m_pos;

					Matrix3f rAPcross = getCrossMatrix(rAP);
					Matrix3f rBPcross = getCrossMatrix(rBP);
					

					float p = (a->m_v - b->m_v).dot(c[0].n) / 
						(a->m_minv + b->m_minv + (rAPcross*a->m_Jinv*rAPcross * c[0].n).dot(c[0].n)
											   + (rBPcross*b->m_Jinv*rBPcross * c[0].n).dot(c[0].n)
						);

					qDebug() << "COLLISION point: " << c[0].pt.x() << " "<< c[0].pt.y() << " "<< c[0].pt.z() << 
									" impulse: " << p;

					a->AddImpulse(-2.0*p * c[0].n, c[0].pt);
					b->AddImpulse(2.0*p * c[0].n, c[0].pt);
				}			
			}
		}


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