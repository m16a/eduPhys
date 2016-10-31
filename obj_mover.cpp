#include "obj_mover.h"
#include <cmath>
#include <QtDebug>
#include "gpuhelper.h"
#include "geometry.h"
#include "my_utils.h"
#include "poly34.h"
#include "rwi.h"
#include "float.h"
#include "debug_draw.h"

ObjMover::ObjMover()
{
	m_pSelectedEnt = 0;
	m_activeHelperIndx = -1;
	m_lastIn = Vector3f(0,0,0);
}

bool ObjMover::OnMouseMove(const Vector3f& in, const SRay& r)
{
	bool res = false;

	if (m_pSelectedEnt)
	{
		if (m_activeHelperIndx > -1)//rotation
		{
			Vector3f dR = in - m_lastIn;
			Vector3f axis = m_pSelectedEnt->m_rot*Vector3f(0,0,1);
			
			Quaternionf q;  q = AngleAxis<float>(3.14/10, axis);
			m_pSelectedEnt->m_rot *= q;	
			res = true;

			SDebugPlane pln;
			pln.m_n = axis;
			pln.m_d = - pln.m_n.dot(m_pSelectedEnt->m_pos);

			SRayHit hit;
			if (pln.IntersectRay(r, hit))
			{
				Vector3f planeHitPoint = hit.m_pt;		
			
				if (m_lastIn.dot(m_lastIn) > 0.001) //previous point is  valid	
				{
					float cosa = m_lastIn.dot(planeHitPoint) / m_lastIn.norm() / planeHitPoint.norm();
					qDebug() << "cosa:" << cosa;
				}

				m_lastIn = planeHitPoint;
			}
		}

	}
	
	//qDebug() << "2D moving: " << distScr << " " << angleScr*180.0f/M_PI;
	//m_lastIn = in;
	return res;
}

void ObjMover::OnSelect(IPhysEnt* e)
{
	m_pSelectedEnt = e;
}

void ObjMover::OnDeselect(IPhysEnt* e)
{
	m_pSelectedEnt = 0;
	m_activeHelperIndx = -1;
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
//	m_rotHlpr.m_helpers[1].Draw();
//	m_rotHlpr.m_helpers[2].Draw();
  glEnable(GL_LIGHTING);
}

bool ObjMover::RWI(const SRay& r)
{
	if (!m_pSelectedEnt)
		return false;

	float minDist = FLT_MAX;
	int activeHelper = -1;	
	Vector3f org = r.m_org;
	Vector3f dir = r.m_dir;
	for (int i=0; i<1; ++i)
	{
		STorus tor = m_rotHlpr.m_helpers[i];

		//Vector3f org = Vector3f(0.5,0.3,0);
		//Vector3f dir =  Vector3f(1,0.4,0);
		DebugManager()->DrawVector(org, dir, 2);	 
		dir.normalize();
		int n;
		float pts[4];		
		qDebug() << "----------------------------------------";
		qDebug() << "test ray: " << org << "dir: " << dir;	
		tor.line_intersect(org, dir, &n, pts);

		qDebug() << "torLine hit cnt:" << n << "rayOrg:" <<org << "rayDir:"<< dir;	
		for (int j=0; j<n; ++j)
		{
			if (pts[j] < minDist && pts[j] > 0)
			{
				minDist = pts[j];
				activeHelper = i;
			}		
		}	
		if (minDist < FLT_MAX)
		{
			qDebug() << "res point:" << minDist;
		}
	}

	if (activeHelper > -1)
	{
		Vector3f resPoint = (org+minDist*dir);
		m_activeHelperIndx = activeHelper;
		qDebug() << "active helper:" << activeHelper << "point:" << resPoint;
		return true;
	}
	return false;
}

STorus::STorus()
{
	m_rMajor = 1;//0.2;
	m_rMinor = 0.05;//0.003;
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

				glVertex3d(x, y, z);
			}
		}
		glEnd();
	}
	gpu.popMatrix(GL_MODELVIEW);
}

void STorus::line_intersect(const Vector3f& org, const Vector3f& dir, 
			  int * num_intersections,
			  float * intersections) const
{
	assert(m_rMajor == 1.0f);//don't forget to normalize torus

	//transform ray to torus CS
	Vector3f orgT = org - m_pos;
	Vector3f dirT = m_rot.conjugate() * dir;
	assert(dirT.dot(dirT) - 1.0f < 0.001);
	qDebug() << "Tor r:" << m_rMinor << "cen: " << m_pos;	
	qDebug() << "rayInTor org:" << orgT << "dir:"<< dirT;	

	float orgT2 = orgT.dot(orgT);
	float r2 =  m_rMinor * m_rMinor;
	float beta = dirT.dot(orgT);
	float gamma = orgT2 + 1.0f -r2; 

  float B = 4*beta;
  float C = 4*beta*beta + 2*gamma- 4*(dirT[0]*dirT[0]+dirT[1]*dirT[1]);
  float D = 4*beta*gamma - 8*(dirT[0]*orgT[0]+dirT[1]*orgT[1]);
  float E = gamma*gamma - 4*(orgT[0]*orgT[0]+orgT[1]*orgT[1]);


  float D1 = 8*(dirT[2]*orgT[2]) + 4*beta*(orgT2 - 1.0f - r2);
  float E1 = orgT[0]*orgT[0]*orgT[0]*orgT[0] + orgT[1]*orgT[1]*orgT[1]*orgT[1] + orgT[2]*orgT[2]*orgT[2]*orgT[2] + (1-r2)*(1-r2) + 2*(orgT[0]*orgT[0]*orgT[1]*orgT[1] + orgT[2]*orgT[2]*(1-r2) + (orgT[0]*orgT[0] + orgT[1]*orgT[1])*(orgT[2]*orgT[2]-1-r2));
  // Solve quartic...
  double roots[4];
	//qDebug() << "TEST SolveP4 2 ==" << SolveP4(roots, 1,1,1,0);
	//qDebug() << "TEST SolveP4 4 ==" << SolveP4(roots, 2,-41,-42,360);
  int nroots = SolveP4(roots,B,C,D,E);
	qDebug() << "coefs: 1.0 " << B << C << D << E;	
	qDebug() << "diff in D: " << D-D1 << "in E:" << E-E1;	
	qDebug() << "nroots:" << nroots;	
  *num_intersections = 0;
  while(nroots--)
    {
      float t = roots[nroots];
      //float x = org[0] + t*dir[0];
      //float y = org[1] + t*dir[1];
      //float l = m_rMajor*(M_PI/2 - atan2(y,x));
      //if (/*l <= vlength &&*/ l >= 0)//TODO: clarify what is vlength
        intersections[(*num_intersections)++] = t;
    }
}
