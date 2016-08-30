#include "obj_mover.h"
#include <cmath>
#include <QtDebug>
#include "gpuhelper.h"
#include "geometry.h"
#include "my_utils.h"
#include "poly34.h"

ObjMover::ObjMover()
{
	m_pSelectedEnt = 0;
}

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
}

void ObjMover::OnDeselect(IPhysEnt* e)
{
	m_pSelectedEnt = 0;
}

void ObjMover::Update()
{
	if (!m_pSelectedEnt)
		return;
	m_rotHlpr.m_helpers[0].m_pos = m_pSelectedEnt->m_pos;
	m_rotHlpr.m_helpers[0].m_rot = m_pSelectedEnt->m_rot;
	m_rotHlpr.m_helpers[0].m_col = Vector3f(1.0f,0.0f,0.0f);
	m_rotHlpr.m_helpers[1].m_pos = m_pSelectedEnt->m_pos;
	m_rotHlpr.m_helpers[1].m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0) * m_pSelectedEnt->m_rot;
	m_rotHlpr.m_helpers[1].m_col = Vector3f(0/255.0f,255/255.0f,0/255.0f);
	m_rotHlpr.m_helpers[2].m_pos = m_pSelectedEnt->m_pos;
	m_rotHlpr.m_helpers[2].m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0) * Quaternionf(0.7071067811865476, 0.7071067811865476, 0, 0) * m_pSelectedEnt->m_rot;
	m_rotHlpr.m_helpers[2].m_col = Vector3f(0/255.0f,0/255.0f,255/255.0f);
		
	glDisable(GL_LIGHTING);
	m_rotHlpr.m_helpers[0].Draw();
	m_rotHlpr.m_helpers[1].Draw();
	m_rotHlpr.m_helpers[2].Draw();
  glEnable(GL_LIGHTING);
}

STorus::STorus()
{
	m_rMajor = 0.2;
	m_rMinor = 0.003;
}

void STorus::Draw()
{
	Affine3f t = Translation3f(m_pos) * m_rot;
	gpu.pushMatrix(GL_MODELVIEW);
	gpu.multMatrix(t.matrix(),GL_MODELVIEW);
	
	int numc = 100, numt = 100;

	double TWOPI = 2 * M_PI;
	for (int i = 0; i < numc; i++) {
		glBegin(GL_QUAD_STRIP);
		glColor3f(m_col[0], m_col[1], m_col[2]);
		for (int j = 0; j <= numt; j++) {
			for (int k = 1; k >= 0; k--) {
				double s = (i + k) % numc + 0.5;
				double t = j % numt;

				double x = (m_rMajor + m_rMinor * cos(s * TWOPI / numc)) * cos(t * TWOPI / numt);
				double y = (m_rMajor + m_rMinor * cos(s * TWOPI / numc)) * sin(t * TWOPI / numt);
				double z = m_rMinor * sin(s * TWOPI / numc);

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
int STorus::line_intersect(const Vector3f& org, const Vector3f& dir, 
			  int * num_intersections,
			  float * intersections,
			  float * shade) const
//              
// Intersect ray
//          [ax]     [bx]              
//	    [ay] + t [by]
//	    [az]     [bz]
// with torus at origin, major radius R, minor radius r
//
{

  // This struct is syntactic sugar so that a.b,
  // (the dot product) looks just right (:-)
  struct { float a,b; } a;
  a.b = org[0]*dir[0] + org[1]*dir[1] + org[2]*dir[2];
  a.a = org.dot(org);

  // Set up quartic in t:
  //
  //  4     3     2
  // t + A t + B t + C t + D = 0
  //
  float R2 = m_rMajor*m_rMajor;
  float K = a.a - m_rMinor*m_rMinor - R2;
  float A = 4*a.b;
  float B = 2*(2*a.b*a.b + K + 2*R2*dir[2]*dir[2]);
  float C = 4*(K*a.b + 2*R2*org[2]*dir[2]);
  float D = K*K + 4*R2*(org[2]*org[2] - m_rMinor*m_rMinor);

  // Solve quartic...
  double roots[4];
  int nroots = SolveP4(roots,A,B,C,D);

  *num_intersections = 0;
  while(nroots--)
    {
      float t = roots[nroots];
      float x = org[0] + t*dir[0];
      float y = org[1] + t*dir[1];
      float l = m_rMajor*(M_PI/2 - atan2(y,x));
      if (/*l <= vlength &&*/ l >= 0)//TODO: clarify what is vlength
        intersections[(*num_intersections)++] = t;
    }
}
