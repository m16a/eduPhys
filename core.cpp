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
#define DEBUG_COLLISIONS 0

const float Core::COLLISION_DEPTH_TOLERANCE = 2*1e-3;

Core::Core()
{
}

void Core::Dump(int entId)
{
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
	{
		if (entId == -1 || entId == (*it)->m_id)
		Debug()<< "\tObjID:" << (*it)->m_id << " active:" << ((*it)->m_active ? 1 : 0 ) << " pos" <<  (*it)->m_pos << " vel" << (*it)->m_v << " rot" << (*it)->m_rot << " w_rot" << (*it)->m_w;
	}
}

float Core::CalcKineticEnergy()
{
	float result = 0;
	int size = m_objects.size();
	for (int i = 0; i < size; ++i)
	{
		result += m_objects[i]->CalcKineticEnergy(); 
	}
	return result;
}

void Core::StepAll(float dt)
{
	//qDebug() << "step:" << dt;
	std::vector<IPhysEnt*>::iterator it = m_objects.begin();
	for (; it != m_objects.end(); ++it)
		(*it)->Step(dt);
	//DumpAll();
}

static bool overlapTestAABB(Vector3f a[2], Vector3f b[2]) 
{
  return (a[0][0] <= b[1][0] && a[1][0] >= b[0][0]) &&
         (a[0][1] <= b[1][1] && a[1][1] >= b[0][1]) &&
         (a[0][2] <= b[1][2] && a[1][2] >= b[0][2]);
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

				IPhysEnt* a = m_objects[i];
				IPhysEnt* b = m_objects[j];

				if ((!a->m_active || a->m_isStatic) &&
						(!b->m_active || b->m_isStatic))
					continue;
				
				//aka broad phase					
				if (!overlapTestAABB(a->m_bbox, b->m_bbox))
					continue;

				//aka narrow phase					
				Contact c[8];
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
					float min_depth = 10000.f;
#if DEBUG_STEP
					Debug() << "Total contacts: "<< cntct_cnt;
#endif
					for (int cnt_indx=0; cnt_indx<cntct_cnt; ++cnt_indx)
					{
						bool isSeparatingContact = true;
						{
							Contact cntct = c[cnt_indx];	
							assert(fabs(cntct.n.norm()-1.0f) < 0.001);
							Vector3f rAP = cntct.pt - a->m_pos;
							Vector3f rBP = cntct.pt - b->m_pos;

							Matrix3f rAPcross = getCrossMatrix(rAP);
							Matrix3f rBPcross = getCrossMatrix(rBP);
							
							Vector3f v_contact = ((b->m_v + (b->m_w).cross(rBP)) - (a->m_v + (a->m_w).cross(rAP))); 
							isSeparatingContact = v_contact.dot(cntct.n) > 0;
						}
						if (c[cnt_indx].depth == 1000.f)
						{
							assert(!"Possible missed depth in contact calculation");
						}

						if (c[cnt_indx].depth < min_depth)
							if (!isSeparatingContact)
								min_depth = c[cnt_indx].depth;
							else
							{
#if DEBUG_STEP
								qDebug() << "separating contact was skipped";
#endif
							}

					}

					if (min_depth < res)
						res = min_depth; 

					if (!applyImpulses)
						continue;
					
					//TODO: remove after LCP solver implementing
					if (0 && cntct_cnt > 1)
					{

						Vector3f tmp(0,0,0);
						for (int cnt_i=0; cnt_i<cntct_cnt; ++cnt_i)
							tmp += c[cnt_i].pt;

						c[0].pt = tmp / cntct_cnt;
					}
					Vector3f center(0,0,0);
					int separatingContactsCount = 0;
					Vector3f normal;
					for (int cnt_indx=0; cnt_indx<cntct_cnt; ++cnt_indx)
					{
						Contact cntct = c[cnt_indx];	
						assert(fabs(cntct.n.norm()-1.0f) < 0.001);
						Vector3f rAP = cntct.pt - a->m_pos;
						Vector3f rBP = cntct.pt - b->m_pos;

						Vector3f v_contact = ((b->m_v + (b->m_w).cross(rBP)) - (a->m_v + (a->m_w).cross(rAP))); 
							
						/*
						if (0 && v_contact.dot(c[0].n) < RESTING_CONTACT_SPEED)
						{
							qDebug() << "vCon" << v_contact << c[0].n;	
							a->m_active = false;
							b->m_active = false;
							a->m_v = b->m_v = Vector3f(0,0,0);	
							continue;
						}
						*/

#if DEBUG_COLLISIONS
					qDebug() << "COLLISION" << a->m_id << ":" << b->m_id << "numOfPts:" << cntct_cnt;
						qDebug() << "pt:"<< cntct.pt << "normal:" << cntct.n << " depth:" << cntct.depth << "v_con:" << v_contact << "v_conN:" << v_contact.dot(cntct.n);
#endif
						if (v_contact.dot(cntct.n) > 0)
						{					
#if DEBUG_COLLISIONS
			qDebug() << "Positive contact speed:" << v_contact.dot(cntct.n);
#endif
							continue;
						}
						center += cntct.pt;
						separatingContactsCount++; 
						normal = cntct.n;
					}

					if (!separatingContactsCount)
						return res;

					assert(separatingContactsCount);
					
					assert(fabs(normal.norm()-1.0f) < 0.001);
					
					center /= separatingContactsCount;

					Vector3f rAP = center - a->m_pos;
					Vector3f rBP = center - b->m_pos;

					Vector3f v_contact = ((b->m_v + (b->m_w).cross(rBP)) - (a->m_v + (a->m_w).cross(rAP))); 

					Matrix3f rAPcross = getCrossMatrix(rAP);
					Matrix3f rBPcross = getCrossMatrix(rBP);

					Matrix3f rotM1 = a->m_rot.toRotationMatrix();
					Matrix3f rotM2 = b->m_rot.toRotationMatrix();

					Matrix3f invJ1 = rotM1 * a->m_Jinv * rotM1.transpose(); 
					Matrix3f invJ2 = rotM2 * b->m_Jinv * rotM2.transpose(); 
				
					float e = 1;//0.8f;//restitution coef
					float p = -(1 + e)*v_contact.dot(normal) / 
						(a->m_minv + b->m_minv - (rAPcross*invJ1*rAPcross * normal).dot(normal)
																	 - (rBPcross*invJ2*rBPcross * normal).dot(normal)
						);
					float tmp = (invJ2 * rBP.cross(normal).cross(rBP)).dot(normal);

					//qDebug() << "m16a:" << p << -(1 + e)*v_contact.dot(normal) << a->m_minv <<  b->m_minv << (rAPcross*invJ1*rAPcross * normal).dot(normal) << (rBPcross*invJ2*rBPcross * normal).dot(normal) << tmp;
					
					a->AddImpulse(-p * normal, center);
					b->AddImpulse(p * normal, center);
						
					DebugManager()->DrawVector(center, normal, p*3);	 
					DebugManager()->DrawVector(center, normal, p*3);	 
					DebugManager()->DrawSphere(center, 0.02, Color(0,1,0,1));	 
				}			
			}
		}

	return res;
}

void Core::Step(float reqStep)
{
	int subStep = 0;
	const float fullStep = reqStep;
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
		qDebug() << gRed << "subStep[collPath]:" << gReset << subStep;
#endif
		subStep++;

		StepAll(reqStep);
		const float startDepth = FindCollisions(false);
		float finishDepth = startDepth;
		StepAll(-reqStep);
	
#if DEBUG_STEP
		qDebug() << "start depth" << startDepth;
#endif

		if (startDepth < -COLLISION_DEPTH_TOLERANCE)
			while (i<MAX_COLLISIONS_ITERATIONS)
			{
#if DEBUG_STEP
				qDebug() << gGreen << "Collison iteration:" << gReset << i << "/" << MAX_COLLISIONS_ITERATIONS;
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

#if DEBUG_STEP
				qDebug() << "mid:" << mid <<" depth:" << depth;
#endif

				finishDepth = depth;
				if (depth < 0 && depth >= -COLLISION_DEPTH_TOLERANCE)
					break;

				if (depth >= -COLLISION_DEPTH_TOLERANCE)
					sStep = mid;
				else
					fStep = mid;						

				++i;
			}
		
		StepAll(mid);
		FindCollisions(true);

		reqStep-= mid;

#if DEBUG_STEP
		qDebug() << "penetration depthes - start:" << startDepth << "finish:" << finishDepth << "/"<< COLLISION_DEPTH_TOLERANCE;
		qDebug() << gRed << "[" << gReset << "impulsePath] performed time:" << mid << "left:" << reqStep << "full:" << fullStep;
		Dump();
#endif
	};
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
