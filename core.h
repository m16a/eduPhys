#ifndef _CORE_H_
#define _CORE_H_

#include <vector>

class Sphere;

class Core
{
public:
	Core();
	std::vector<Sphere*> m_objects;


	void Draw();

	void Step(float t);
};

#endif