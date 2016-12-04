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

void IPhysEnt::Serialize(ser::SerPhys* sp)
{
	ser::Vector3f* pos = sp->mutable_pos();
	pos->set_x(m_pos[0]);
	pos->set_y(m_pos[1]);
	pos->set_z(m_pos[2]);

	ser::Quaternionf* rot = sp->mutable_rot();
	rot->set_x(m_rot.x());
	rot->set_y(m_rot.y());
	rot->set_z(m_rot.z());
	rot->set_w(m_rot.w());
	
	ser::Vector3f* v = sp->mutable_v();
	v->set_x(m_v[0]);
	v->set_y(m_v[1]);
	v->set_z(m_v[2]);
	
	ser::Vector3f* w = sp->mutable_w();
	w->set_x(m_w[0]);
	w->set_y(m_w[1]);
	w->set_z(m_w[2]);
	
	sp->set_minv(m_minv);
	sp->set_id(m_id);

	ser::Matrix3f* Jinv = sp->mutable_jinv();
	ser::Vector3f* row1 = Jinv->mutable_row1();
 	row1->set_x(m_Jinv(0,0));
 	row1->set_y(m_Jinv(0,1)); 
	row1->set_z(m_Jinv(0,2));
	ser::Vector3f* row2 = Jinv->mutable_row2();
 	row2->set_x(m_Jinv(1,0));
 	row2->set_y(m_Jinv(1,1)); 
	row2->set_z(m_Jinv(1,2));
	ser::Vector3f* row3 = Jinv->mutable_row3();
 	row3->set_x(m_Jinv(2,0));
 	row3->set_y(m_Jinv(2,1)); 
	row3->set_z(m_Jinv(2,2));
}

