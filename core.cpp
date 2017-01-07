#include "core.h"
#include "sphere.h"
#include "box.h"
#include <QtDebug>
#include "collision.h"
#include "rwi.h"
#include <cfloat>
#include "my_utils.h"
#include "serialization/phys_ent.pb.h"
#include <iostream>
#include <fstream>

#define DEBUG_STEP 0


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
	//DumpAll();
}

float Core::FindCollisions(bool applyImpulses)
{
	float res = 10000.0f;
	int size = m_objects.size();
	for (int i = 0; i < size; ++i)
		for (int j = 0; j < size; ++j)
		{
			if (m_objects[i]->m_id < m_objects[j]->m_id)
			{
				IPhysEnt *a = m_objects[i];
				IPhysEnt *b = m_objects[j];

				Contact c[4];
				int cntct_cnt = 0;

				//TODO: ugly, refactor on adding new collision geom
				if (Box* a1 = dynamic_cast<Box*>(a))
				{
					if (Sphere* b1 = dynamic_cast<Sphere*>(b))
					{
						collide(b1, a1, c, cntct_cnt);
						a = b1; b = a1;
					}
					else if (Box* b1 = dynamic_cast<Box*>(b))
					{
						collide(a1, b1, c, cntct_cnt);
						a = a1; b = b1;
					}
				}else if (Sphere* a1 = dynamic_cast<Sphere*>(a))
				{
					if (Box* b1 = dynamic_cast<Box*>(b))
					{
						collide(a1, b1, c, cntct_cnt);
						a = a1; b = b1;

					}
					else if (Sphere* b1 = dynamic_cast<Sphere*>(b))
 					{	
 						collide(a1, b1, c, cntct_cnt);
						a = a1; b = b1;
					}

				}

				if (cntct_cnt > 0)
				{
					if (c[0].depth < res)
						res = c[0].depth;

					if (!applyImpulses)
						continue;
					
					//TODO: remove after LCP implementing
					if (cntct_cnt > 1)
					{
						Vector3f tmp(0,0,0);
						for (int cnt_i=0; cnt_i<cntct_cnt; ++cnt_i)
							tmp += c[cnt_i].pt;

						c[0].pt = tmp / cntct_cnt;
					}

					assert(c[0].n.norm() > 0.1);
					Vector3f rAP = c[0].pt - a->m_pos;
					Vector3f rBP = c[0].pt - b->m_pos;

					Matrix3f rAPcross = getCrossMatrix(rAP);
					Matrix3f rBPcross = getCrossMatrix(rBP);
					
					float e = 0.8f;//restitution coef
					Vector3f v_contact = (a->m_v + (a->m_w).cross(rAP) - (b->m_v + (b->m_w).cross(rBP))); 
					if (0 && v_contact.dot(c[0].n) < RESTING_CONTACT_SPEED)
					{
						qDebug() << "vCon" << v_contact << c[0].n;	
						a->m_active = false;
						b->m_active = false;
						a->m_v = b->m_v = Vector3f(0,0,0);	
						continue;
					}

					a->m_active = true;
					b->m_active = true;	
					float p = (1 + e)*v_contact.dot(c[0].n) / 
						(a->m_minv + b->m_minv + (rAPcross*a->m_Jinv*rAPcross * c[0].n).dot(c[0].n)
											   + (rBPcross*b->m_Jinv*rBPcross * c[0].n).dot(c[0].n)
						);
					qDebug() << "COLLISION numOfPts:" << cntct_cnt;
	
					qDebug() << "pt:"<<c[0].pt <<  "normal:" << c[0].n << " depth:" << c[0].depth;
					
					a->AddImpulse(-p * c[0].n, c[0].pt);
					b->AddImpulse(p * c[0].n, c[0].pt);
						
					DebugManager()->DrawVector(c[0].pt, c[0].n, p*3);	 
					DebugManager()->DrawVector(c[0].pt, -c[0].n, p*3);	 
				}			
			}
		}

	return res;
}

void Core::Step(float reqStep)
{
	int subStep = 0;
	while (reqStep > 0)
	{
		if (subStep > 5)
		{
			qCritical() << "Over 5 substeps";
			assert(0);
		}

		int i = 0;
		float sStep = 0.0f;
		float fStep = reqStep;
		float mid = reqStep;

#if DEBUG_STEP
		qDebug() << gRed << "subStep[collPath]:" << gReset << subStep++ << "time:" << mid << "/" << reqStep;
#endif

		StepAll(reqStep);
		float d = FindCollisions(false);
		StepAll(-reqStep);
	
#if DEBUG_STEP
		qDebug() << "penetration depth:" << d << "/"<< COLLISION_DEPTH_TOLERANCE;
#endif

		if (d < -COLLISION_DEPTH_TOLERANCE)
			while (i<MAX_COLLISIONS_ITERATIONS)
			{
#if DEBUG_STEP
				qDebug() << gGreen << "Collison iteration:" << gReset << i;
#endif
				mid = (sStep + fStep) / 2.0f;
				
			//	qDebug() << "PRE";	
			//	DumpAll();	
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

				if (depth >= -COLLISION_DEPTH_TOLERANCE)
					sStep = mid;
				else
					fStep = mid;						

				++i;
			}
		
#if DEBUG_STEP
		qDebug() << gRed << "[" << gReset << "impulsePath] time:" << mid << "/" << reqStep;
#endif

		StepAll(mid);
		FindCollisions(true);

		reqStep-= mid;
	};
	//DumpAll();
}

void Core::Draw()
{
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
		(*it)->Draw();
}

int Core::RWI(const SRay& r, SRayHit& out_hit)
{
	int res = 0;
	float dist = FLT_MAX; 
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
	{
		SRayHit o;
		if ((*it)->IntersectRay(r, o))
		{
			if (dist > o.m_dist)
			{
				res = 1;
				out_hit = o;
				dist = o.m_dist;
			}
		}
	}

	return res;
}


void Core::SerializeToFile(const char* name)
{
	qDebug() << "Saving scene to file: " << name;
	ser::Core c;

	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
	{
		ser::SerPhys* sp = c.add_objct();		
		(*it)->Serialize(sp);
	}
	std::fstream output(name, std::ios::out | std::ios::trunc | std::ios::binary);
	c.SerializeToOstream(&output);
}

void Core::DeserializeFromFile(const char* name)
{
	qDebug() << "Load scene from file: " << name;
	ser::Core c;
	std::fstream input(name, std::ios::in | std::ios::binary);
  c.ParseFromIstream(&input);

	m_objects.clear();	

	for (int i=0; i<c.objct_size(); ++i)
	{
		const ser::SerPhys& sp = c.objct(i);
		ser::SerPhys_Type t = sp.type();
		IPhysEnt* e = 0;
		switch (t)
		{
			case ser::SerPhys::Sphere:
				e = new Sphere();
				break;
			case ser::SerPhys::Box:
				e = new Box();
				break;
			default:
				assert(0);
				break;
		}	
		assert(e);	
		e->Deserialize(&sp);
		m_objects.push_back(e);
	}
}
