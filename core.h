#ifndef _CORE_H_
#define _CORE_H_

#include <vector>
#include <list>
#include "collision.h"

class Sphere;
class IPhysEnt;
struct SRay;
struct SRayHit;
 
class Core
{
public:
	static const int MAX_COLLISIONS_ITERATIONS = 10;
	static const int	SI_ITERATIONS = 3;
	static const float COLLISION_DEPTH_TOLERANCE;
	static const float RESTING_CONTACT_SPEED = 1e-2;	

	Core();
	void Draw();
	void Step(float reqStep);
	int RWI(const SRay& r, SRayHit& out_hit);
	float CalcKineticEnergy();
	void Dump(int entId = -1);

	void SerializeToFile(const char* name);
	void DeserializeFromFile(const char* name);

	std::vector<IPhysEnt*> m_objects;
	int m_frameID;
	std::list<Contact> m_contacts;
private:
	void StepAll(float dt);
	//finds collisions and returns deepest penetration  
	float FindCollisions(bool updateContacts);
	
	void SolveContacts();
	void AddContact(const Contact& c);
	void ValidateOldContacts();
	void RemoveContacts();
};

#endif
