#include "debug_draw.h"


void SDebugPlane::Draw()
{

}

void SDebugMngr::DrawPlane(const Vector3f& n, const Vector3f& d)
{
	SDebugPlane* p = new SDebugPlane();
	p->m_n = n;
	p->m_d = d;

	m_list.push_front(p);
}

void SDebugMngr::Draw()
{
	while (!m_list.empty())
	{
		IDebugItem* i = m_list.front();
		i->Draw();
		m_list.pop_front();
	}
}
