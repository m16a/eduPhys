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

#define DEBUG_STEP 1
#define DEBUG_COLLISIONS 1

//const float Core::COLLISION_DEPTH_TOLERANCE = 5*1e-3;
const float Core::COLLISION_DEPTH_TOLERANCE = 1*1e-2;

Core::Core()
{
	m_frameID = 0;
	m_substepID = 0;
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

float Core::FindCollisions(bool updateContacts)
{
	float res = 10000.0f;
	int size = m_objects.size();
	for (int i = 0; i < size; ++i)
		for (int j = 0; j < size; ++j)
		{
			IPhysEnt* a = m_objects[i];
			IPhysEnt* b = m_objects[j];

			if (a->m_id >= b->m_id)
				continue;

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

			float min_depth = 10000.f;
			for (int cnt_indx=0; cnt_indx<cntct_cnt; ++cnt_indx)
			{
				Contact& cntct = c[cnt_indx];	
				cntct.a = a;
				cntct.b = b;
				cntct.lifeFrameID = m_frameID;
				cntct.substepID = m_substepID;

				assert(fabs(cntct.n.norm()-1.0f) < 0.001);
				Vector3f rAP = cntct.pt - a->m_pos;
				Vector3f rBP = cntct.pt - b->m_pos;

				Matrix3f rAPcross = getCrossMatrix(rAP);
				Matrix3f rBPcross = getCrossMatrix(rBP);
				
				Vector3f v_contact = ((b->m_v + (b->m_w).cross(rBP)) - (a->m_v + (a->m_w).cross(rAP))); 
				const bool isSeparatingContact = v_contact.dot(cntct.n) > 0;

				if (c[cnt_indx].depth == 1000.f)
				{
					assert(!"Possible missed depth in contact calculation");
				}

				if (c[cnt_indx].depth < min_depth)
					if (!isSeparatingContact)
						min_depth = c[cnt_indx].depth;

				if (isSeparatingContact)
				{
#if DEBUG_STEP
					qDebug() << "separating contact was skipped";
#endif
					continue;
				}

				if (min_depth < res)
					res = min_depth; 

				if (updateContacts)
					AddContact(cntct);
			}
		}

	return res;
}

void Core::AddContact(const Contact& c)
{
	//check if we have this contact already stored	
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
	{
		if (it->IsEqual(c) && it->IsSleeping())
		{
			(*it).lifeFrameID = m_frameID;
			(*it).substepID = m_substepID;
			return;	
		}
	}

	m_contacts.push_back(c);
}

void Core::ValidateOldContacts()
{
	//keep old sleeping contacts
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
	{
		Contact& c = (*it);
		if (c.IsSleeping())
		{
			c.lifeFrameID = m_frameID;
			c.substepID = m_substepID;
		}
	}
}

void Core::RemoveContacts()
{
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end();)
	{
		Contact& c = (*it);
		if (c.lifeFrameID != m_frameID || c.substepID != m_substepID)
			m_contacts.erase(it++);
		else
			++it;
	}
}

void Core::Step(float reqStep)
{
	m_substepID = 0;
	const float fullStep = reqStep;
	while (reqStep > 0)
	{
		if (m_substepID > 5)
		{
			qCritical() << "Over 5 substeps";
			assert(0);
		}

		int i = 0;
		float sStep = 0.0f;
		float fStep = reqStep;
		float mid = reqStep;

#if DEBUG_STEP
		qDebug() << gRed << "subStep[collPath]:" << gReset << m_substepID;
		Dump();
#endif
		m_substepID++;

		StepAll(reqStep);
		const float startDepth = FindCollisions(false);
		float finishDepth = startDepth;
		StepAll(-reqStep);
	
#if DEBUG_STEP
		qDebug() << "start depth" << startDepth;
#endif

		if (startDepth < -COLLISION_DEPTH_TOLERANCE)
			while (i < MAX_COLLISIONS_ITERATIONS)
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

		ValidateOldContacts();
		RemoveContacts();
		
		ListContacts();
		SolveContacts(mid);

		reqStep-= mid;

#if DEBUG_STEP
		qDebug() << "penetration depthes - start:" << startDepth << "finish:" << finishDepth << "/"<< COLLISION_DEPTH_TOLERANCE;
		qDebug() << gRed << "[" << gReset << "impulsePath] performed time:" << mid << "left:" << reqStep << "full:" << fullStep;
		Dump();
#endif

	};

		Dump();
	//deactivate resting bodies
	TownIsSleeping();

		Dump();
	DrawContacts();
	ListContacts();

	m_frameID++;
}

void Core::ListContacts()
{
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
		it->Dump();
}

void Core::DrawContacts()
{
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
		DebugManager()->DrawSphere(it->pt, 0.02, it->IsSleeping() ? Color(0,0,1,1) : Color(0,1,0,1));	 
}

void Core::TownIsSleeping()
{
	//sleep all bodies
	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
		it->a->m_active = it->b->m_active = false; 
	
	qDebug() << "wake cntcts";

	//wake active
	it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
	{
		const Contact& c = (*it);

		if (c.a->m_active == c.b->m_active && c.a->m_active == true)
			continue;

		assert(fabs(c.n.norm()-1.0f) < 0.001);
		Vector3f rAP = c.pt - c.a->m_pos;
		Vector3f rBP = c.pt - c.b->m_pos;

		Matrix3f rAPcross = getCrossMatrix(rAP);
		Matrix3f rBPcross = getCrossMatrix(rBP);
		
		Vector3f v_contact = ((c.b->m_v + (c.b->m_w).cross(rBP)) - (c.a->m_v + (c.a->m_w).cross(rAP))); 
		const float vn = v_contact.dot(v_contact);
		const bool isRestingContact = fabs(vn) < RESTING_CONTACT_SPEED * RESTING_CONTACT_SPEED;
		Debug() << "cntct vN:" << vn;

		if (!isRestingContact)
		{
			if (!c.a->m_isStatic)
				if (c.a->CalcKineticEnergy() > 0.001)
				{
					c.a->m_active = true;
					Debug() << "wake";
				}

			if (!c.b->m_isStatic)
				if (c.b->CalcKineticEnergy() > 0.001)
				{
					c.b->m_active = true;
					Debug() << "wake";
				}
		}
	}

}

void Core::SolveContacts(float dt)
{
#if DEBUG_STEP
	Debug() << "Solve contacts num: "<< m_contacts.size();
#endif

	if (m_contacts.empty())
		return;

	std::list<Contact>::iterator it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
	{
		Contact cntct = *it;	

		if (cntct.IsSleeping())
			continue;

		assert(fabs(cntct.n.norm()-1.0f) < 0.001);

		float accP = cntct.accP;
		if (accP > 0.0)
		{
			IPhysEnt* a = cntct.a;
			IPhysEnt* b = cntct.b;
			const Vector3f& normal(cntct.n);
			a->AddImpulse(-accP * normal, cntct.pt);
			b->AddImpulse(accP * normal, cntct.pt);
		}
	}

	it = m_contacts.begin();
	for (; it != m_contacts.end(); ++it)
	{
		Contact& cntct = *it;	

		if (cntct.IsSleeping())
			continue;

		assert(fabs(cntct.n.norm()-1.0f) < 0.001);
		IPhysEnt* a = cntct.a;
		IPhysEnt* b = cntct.b;
		const Vector3f& normal(cntct.n);

		
		for (int i=0; i<SI_ITERATIONS; ++i) 
		{
			Vector3f rAP = cntct.pt - a->m_pos;
			Vector3f rBP = cntct.pt - b->m_pos;

			Vector3f v_contact = ((b->m_v + (b->m_w).cross(rBP)) - (a->m_v + (a->m_w).cross(rAP))); 
				
#if DEBUG_COLLISIONS
		//qDebug() << "COLLISION" << a->m_id << ":" << b->m_id << "numOfPts:" << cntct_cnt;
			//qDebug() << "pt:"<< cntct.pt << "normal:" << cntct.n << " depth:" << cntct.depth << "v_con:" << v_contact << "v_conN:" << v_contact.dot(cntct.n);
#endif

			assert(fabs(normal.norm()-1.0f) < 0.001);

			Matrix3f rAPcross = getCrossMatrix(rAP);
			Matrix3f rBPcross = getCrossMatrix(rBP);

			Matrix3f rotM1 = a->m_rot.toRotationMatrix();
			Matrix3f rotM2 = b->m_rot.toRotationMatrix();

			Matrix3f invJ1 = rotM1 * a->m_Jinv * rotM1.transpose(); 
			Matrix3f invJ2 = rotM2 * b->m_Jinv * rotM2.transpose(); 

			const float extraVelBias = ERP / dt * std::min(0.0f, cntct.depth + Core::COLLISION_DEPTH_TOLERANCE); 
			const float e = 0.6f;//restitution coef
			const float dPn = -((1 + e)*v_contact.dot(normal) + extraVelBias) / 
				(a->m_minv + b->m_minv - (rAPcross*invJ1*rAPcross * normal).dot(normal)
															 - (rBPcross*invJ2*rBPcross * normal).dot(normal)
				);
			float tmp = (invJ2 * rBP.cross(normal).cross(rBP)).dot(normal);

			//qDebug() << "m16a:" << p << -(1 + e)*v_contact.dot(normal) << a->m_minv <<  b->m_minv << (rAPcross*invJ1*rAPcross * normal).dot(normal) << (rBPcross*invJ2*rBPcross * normal).dot(normal) << tmp;
			
			if (0 && v_contact.dot(normal) < RESTING_CONTACT_SPEED)
			{
				//qDebug() << "vCon" << v_contact << c[0].n;	
				//a->m_v = b->m_v = Vector3f(0,0,0);	
				
				if (a->CalcKineticEnergy() < 0.001 && b->CalcKineticEnergy() < 0.001)
				{
					a->m_active = false;
					b->m_active = false;
				}
			}
			const float oldP = cntct.accP;
			cntct.accP = std::max(0.0f, oldP + dPn);
			const float p_to_apply = cntct.accP - oldP;

			a->AddImpulse(-p_to_apply * normal, cntct.pt);
			b->AddImpulse(p_to_apply * normal, cntct.pt);
				
			Debug() << "SI:" << i << " p:" << p_to_apply;
			DebugManager()->DrawVector(cntct.pt, normal, p_to_apply*3);	 
		}			
	}
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
		e->UpdatedPosRot();
	}
}
