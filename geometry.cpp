#include "geometry.h"

const Vector3f g_Gravity(0.0f, 0.0f, -98.8f);
IPhysEnt::IPhysEnt()
{
	m_pos = Vector3f(0.f, 0.f, 0.f);
	m_rot = Quaternionf::Identity();

	m_v = Vector3f(0.f, 0.f, 0.f);
	m_w = Vector3f(0.f, 0.f, 0.f);
	m_minv = 0.01;//100kg
	m_active = true;
	m_id = 0;
}
