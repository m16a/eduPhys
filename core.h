#ifndef _CORE_H_
#define _CORE_H_

#include <vector>

class Sphere;
class IPhysEnt;

class Core
{
public:
	Core();
	std::vector<IPhysEnt*> m_objects;


	void Draw();

	void Step(float t);
};

#endif