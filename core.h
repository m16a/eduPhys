#ifndef _CORE_H_
#define _CORE_H_

#include <vector>

class Sphere;
class IPhysEnt;
struct SRay;
struct SRayHit;
 
class Core
{
public:
	static const int MAX_COLLISIONS_ITERATIONS = 10;
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

private:
	void StepAll(float dt);
	//finds collisions and returns deepest penetration  
	//also contact impulses can be provided
	float FindCollisions(bool applyImpulses);
};

#endif
