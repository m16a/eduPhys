#include "core.h"
#include "sphere.h"
#include "box.h"
#include <QtDebug>
#include "collision.h"

#include "my_utils.h"

Core::Core()
{

}

void Core::DumpAll()
{
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
	{
		qDebug() << "\tObjID:" << (*it)->m_id << " pos:"<<  (*it)->m_pos << " rot:" <</*PYRFromQuat*/((*it)->m_rot);
	}
}

void Core::StepAll(float dt)
{
	//qDebug() << "step:" << dt;
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
		(*it)->Step(dt);
//	DumpAll();
}

float Core::FindCollisions(bool applyImpulses)
{
	float res = 10000.0f;
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
					if (c[0].depth < res)
						res = c[0].depth;

					if (!applyImpulses)
						continue;

					Vector3f rAP = c[0].pt - a->m_pos;
					Vector3f rBP = c[0].pt - b->m_pos;

					Matrix3f rAPcross = getCrossMatrix(rAP);
					Matrix3f rBPcross = getCrossMatrix(rBP);
					
					float e = 0.9f;//restitution coef
					float p = (1 + e)*(a->m_v + (a->m_w).cross(rAP) - (b->m_v + (b->m_w).cross(rBP))).dot(c[0].n) / 
						(a->m_minv + b->m_minv + (rAPcross*a->m_Jinv*rAPcross * c[0].n).dot(c[0].n)
											   + (rBPcross*b->m_Jinv*rBPcross * c[0].n).dot(c[0].n)
						);
					qDebug() << "COLLISION";
					qDebug() << " point:" << c[0].pt << 
									" impulse:" << p;
	
					qDebug() << " normal:" << c[0].n << " depth:" << c[0].depth;
					
					a->AddImpulse(-p * c[0].n, c[0].pt);
					b->AddImpulse(p * c[0].n, c[0].pt);
				}			
			}
		}

	return res;
}

void Core::Step(float reqStep)
{
	while (reqStep > 0)
	{
		StepAll(reqStep);
		float d = FindCollisions(false);
		StepAll(-reqStep);
	
		int i = 0;
		float sStep = 0.0f;
		float fStep = reqStep;
		float mid = reqStep;
//		qDebug() << "penetration depth:" << d << "/"<< COLLISION_DEPTH_TOLERANCE;
		if (d < -COLLISION_DEPTH_TOLERANCE)
			while (i<MAX_COLLISIONS_ITERATIONS)
			{
				mid = (sStep + fStep) / 2.0f;
				
		//		qDebug() << "PRE";	
		//		DumpAll();	
				StepAll(mid);
			//	qDebug() << "IN";	
			//	DumpAll();	
				float depth = FindCollisions(false);
				StepAll(-mid);
			//	qDebug() << "POST";
			//	DumpAll();

			//	qDebug() << "coll iter:" << i <<" mid:" << mid <<" depth:" << depth;
				
				if (depth < 0 && depth >= -COLLISION_DEPTH_TOLERANCE)
					break;

				if ( depth >= -COLLISION_DEPTH_TOLERANCE)
					sStep = mid;
				else
					fStep = mid;						

				++i;
			}
		
		//	qDebug() << "step time:" << mid;
		StepAll(mid);
		FindCollisions(true);

		reqStep-= mid;
	};
	DumpAll();
}

void Core::Draw()
{
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
		(*it)->Draw();
}
