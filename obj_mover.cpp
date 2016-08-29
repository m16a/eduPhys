#include "obj_mover.h"
#include <cmath>
#include <QtDebug>
#include "gpuhelper.h"
#include "geometry.h"

void ObjMover::OnMouseMove(const Vector3f& in)
{
	float distScr = (in - m_lastIn).norm();
	float angleScr = atan2((-in[1]+m_lastIn[1]), (in[0]-m_lastIn[0]));	
	
	//qDebug() << "2D moving: " << distScr << " " << angleScr*180.0f/M_PI;
	m_lastIn = in;
}

void ObjMover::OnSelect(IPhysEnt* e)
{
	m_pSelectedEnt = e;
	m_rotHlpr.m_helpers[0].m_pos = e->m_pos;
	m_rotHlpr.m_helpers[0].m_rot = e->m_rot;
	m_rotHlpr.m_helpers[0].m_col = Vector3f(255/255.0f,0/255.0f,0/255.0f);
	m_rotHlpr.m_helpers[1].m_pos = e->m_pos;
	m_rotHlpr.m_helpers[1].m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0) * e->m_rot;
	m_rotHlpr.m_helpers[1].m_col = Vector3f(0/255.0f,255/255.0f,0/255.0f);
	m_rotHlpr.m_helpers[2].m_pos = e->m_pos;
	m_rotHlpr.m_helpers[2].m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0) * Quaternionf(0.7071067811865476, 0.7071067811865476, 0, 0) * e->m_rot;
	m_rotHlpr.m_helpers[2].m_col = Vector3f(0/255.0f,0/255.0f,255/255.0f);
}

void ObjMover::OnDeselect(IPhysEnt* e)
{
	m_pSelectedEnt = 0;
}

void ObjMover::Update()
{
	if (!m_pSelectedEnt)
		return;
	
	m_rotHlpr.m_helpers[0].Draw();
	m_rotHlpr.m_helpers[1].Draw();
	m_rotHlpr.m_helpers[2].Draw();
}

void STorus::Draw()
{
	Affine3f t = Translation3f(m_pos) * m_rot;
	gpu.pushMatrix(GL_MODELVIEW);
	gpu.multMatrix(t.matrix(),GL_MODELVIEW);
	
//	glColorMaterial(GL_FRONT, GL_DIFFUSE);
//	glEnable(GL_COLOR_MATERIAL);	
	int numc = 100, numt = 100;

	double TWOPI = 2 * M_PI;
	for (int i = 0; i < numc; i++) {
		glBegin(GL_QUAD_STRIP);
		glColor3f(m_col[0], m_col[1], m_col[2]);
		for (int j = 0; j <= numt; j++) {
			for (int k = 1; k >= 0; k--) {
				double s = (i + k) % numc + 0.5;
				double t = j % numt;

				double x = (0.2 + 0.003 * cos(s * TWOPI / numc)) * cos(t * TWOPI / numt);
				double y = (0.2 + 0.003 * cos(s * TWOPI / numc)) * sin(t * TWOPI / numt);
				double z = 0.003 * sin(s * TWOPI / numc);

				glVertex3d(2 * x, 2 * y, 2 * z);
			}
		}
		glEnd();
	}
	gpu.popMatrix(GL_MODELVIEW);
}
/*
struct STorus 
{
	
	Vector3f m_pos;
	Quaternionf m_rot;
	float m_rIn;
	float m_rOut;
};
*/
