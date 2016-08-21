#ifndef _COLLISION_H_
#define _COLLISION_H_

#include "sphere.h"
#include "box.h"
#include <Eigen/Geometry>
#include "my_utils.h"
#include "float.h"
#include "debug_draw.h"


using Eigen::Vector3d;
using Eigen::Vector3f;

struct Contact
{
	Vector3f pt;
	Vector3f n;
	float depth;

	Contact():depth(1000.0f){};
};

struct SPlane
{
	Vector3f n;
	float d;
};

void collide(Sphere* a, Sphere* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;
	Vector3f diff = b->m_pos - a->m_pos;
	if (diff.norm() < a->m_r + b->m_r)
	{
		c->n = diff;
		c->pt = a->m_pos + c->n * a->m_r / (a->m_r + b->m_r);
		c->n.normalize();
		c->depth = (c->pt - a->m_pos).norm() - a->m_r;
		out_size = 1;
	}
}

bool overlap(Sphere* a, Box* b)
{
	Quaternionf sT = (b->m_rot).conjugate();
	Vector3f s = sT * (a->m_pos - b->m_pos);
	float d = 0.0f;
	float tmp = 0.0f;

	if (s.x() < -b->m_size[0]/2)
	{
		tmp = s.x() - (-b->m_size[0]/2);
		d += tmp*tmp;
	}
	else if (s.x() > b->m_size[0]/2)
	{
		tmp = s.x() - b->m_size[0]/2;
		d += tmp*tmp;
	}

	if (s.y() < -b->m_size[1]/2)
	{
		tmp = s.y() - (-b->m_size[1]/2);
		d += tmp*tmp;
	}
	else if (s.y() > b->m_size[1]/2)
	{
		tmp = s.y() - b->m_size[1]/2;
		d += tmp*tmp;
	}

	if (s.z() < -b->m_size[2]/2)
	{
		tmp = s.z() - (-b->m_size[2]/2);
		d += tmp*tmp;
	}
	else if (s.z() > b->m_size[2]/2)
	{
		tmp = s.z() - b->m_size[2]/2;
		d += tmp*tmp;
	}
	
	bool res = d <= a->m_r*a->m_r;

//qDebug() << "overlap test " << a->m_id << " " << b->m_id << " " << s << "\n" << 
//	"res d:"<< d << " r^2:" << a->m_r*a->m_r;
	if (res)
	{
	//	qDebug() << "OVERLAP";
	}
	return res;
}

float distPointPlane(Vector3f plane_normal, float d, Vector3f point)
{
	return (plane_normal.dot(point) + d)/plane_normal.norm();
}

bool isPointInsideBox(const Vector3f size, const Vector3f p)
{
	if (fabs(p.x()) <= size.x()/2.0f &&
		fabs(p.y()) <= size.y()/2.0f &&
		fabs(p.z()) <= size.z()/2.0f)
			return true;

	return false;
}

void collide(Sphere* sphere, Box* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;

	if (!overlap(sphere,b))
		return;

	Quaternionf sT = (b->m_rot).conjugate();
	Vector3f s = sT * (sphere->m_pos - b->m_pos);
	//qDebug() << "internal s " << (s).x() << " " << (s).y() <<" " << (s).z(); 

	Vector3f dir = Vector3f(0.0f,0.0f,0.0f);

	if (!isPointInsideBox(b->Size(), s))
	{
		dir.x() = std::min(0.0f, b->m_size[0]/2.0f - (float)fabs(s.x())) *
															 ((s.x() >= 0.0f) ? 1.0f : -1.0f);
		dir.y() = std::min(0.0f, b->m_size[1]/2.0f - (float)fabs(s.y())) *
															 ((s.y() >= 0.0f) ? 1.0f : -1.0f);
		dir.z() = std::min(0.0f, b->m_size[2]/2.0f - (float)fabs(s.z())) *
															 ((s.z() >= 0.0f) ? 1.0f : -1.0f);
	}
	else
	{
		qDebug() << "!warning! : [sphere-box collision] big step or speed";
		
		dir.x() =  (b->m_size[0]/2.0f - (float)fabs(s.x())) *
															 ((s.x() >= 0.0f) ? 1.0f : -1.0f);
		dir.y() =  (b->m_size[1]/2.0f - (float)fabs(s.y())) *
															 ((s.y() >= 0.0f) ? 1.0f : -1.0f);
		dir.z() =  (b->m_size[2]/2.0f - (float)fabs(s.z())) *
															 ((s.z() >= 0.0f) ? 1.0f : -1.0f);	
	}
	//qDebug() << "internal dir " << (dir).x() << " " << (dir).y() <<" " << (dir).z(); 

	out_size = 1;
	//	qDebug() << "internal coll " << (s+dir).x() << " " << (s+dir).y() <<" " << (s+dir).z(); 
	c->pt = b->m_rot*(s + dir) + b->m_pos;
	//qDebug() << "internal pt:" << c->pt;
	c->n = c->pt - sphere->m_pos;
	assert(c->n.norm() > 10e-5);
	if(isPointInsideBox(b->Size(), s))
		c->depth = -((c->pt - sphere->m_pos).norm() + sphere->m_r);
	else
		c->depth = (c->pt - sphere->m_pos).norm() - sphere->m_r;
	c->n.normalize();	
}

inline bool _testSA(const Vector3f& a0, const Vector3f& b0, const Vector3f& a_size, const Vector3f& b_in_a_size, const Vector3f& t)
{
	const Vector3f L = a0.cross(b0);
	if (L.dot(L) > 10e-5)
	{
		const float ra = fabs((a_size/2).dot(L));
		const float rb = fabs(b_in_a_size.dot(L));

		if (fabs(t.dot(L)) > ra + rb) 
			return false;
	}
	else
		qWarning() << "SAT is missed due to paralel edges";
	return true;
}

bool overlap(Box* a, Box* b)
{
	//test all separation axes in a's CS
	//TODO: a lot of multiplication reduce can be made here	

	Quaternionf rel = a->m_rot.conjugate() * b->m_rot;
	Vector3f t = a->m_rot * (b->m_pos - a->m_pos);

	Vector3f b_size_in_a = rel * (b->m_size / 2.0f);
	//test *a* main axes
	for (int i=0; i < 3; ++i)
		if (fabs(t[i]) > a->m_size[i] / 2.0f + fabs(b_size_in_a[i]))
			return false;

	//qDebug() << "box test 1";	
	//test *b* main axes
	for (int i=0; i < 3; ++i)
		if (fabs((rel.conjugate()*t)[i]) > b->m_size[i] / 2.0f + fabs((rel.conjugate()*(a->m_size/2.0f))[i]))
			return false;

	//qDebug() << "box test 2";	
	float ra, rb;
	
	//test a0 x b0
	{
		Vector3f a0(a->m_size[0]/2, 0, 0);
		Vector3f b0(b->m_size[0]/2, 0, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}

	//qDebug() << "box test 3";	
	//test a0 x b1
	{
		Vector3f a0(a->m_size[0]/2, 0, 0);
		Vector3f b0(0,b->m_size[1]/2, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}
	
	//qDebug() << "box test 4";	
	//test a0 x b2
	{
		Vector3f a0(a->m_size[0]/2, 0, 0);
		Vector3f b0(0, 0, b->m_size[2]/2);
		b0 = rel * b0;
		
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}

	//qDebug() << "box test 5";	
	//test a1 x b0
	{
		Vector3f a0(0,a->m_size[1]/2, 0);
		Vector3f b0(b->m_size[0]/2, 0, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}

	//qDebug() << "box test 6";	
	//test a1 x b1
	{
		Vector3f a0(0,a->m_size[1]/2, 0);
		Vector3f b0(0,b->m_size[1]/2, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}
	
	//qDebug() << "box test 7";	
	//test a1 x b2
	{
		Vector3f a0(0,a->m_size[1]/2, 0);
		Vector3f b0(0, 0, b->m_size[2]/2);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}

	//qDebug() << "box test 8";	
	//test a2 x b0
	{
		Vector3f a0(0,0,a->m_size[2]/2);
		Vector3f b0(b->m_size[0]/2, 0, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}

	//qDebug() << "box test 9";	
	//test a1 x b1
	{
		Vector3f a0(0,0,a->m_size[2]/2);
		Vector3f b0(0,b->m_size[1]/2, 0);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}
	
	//qDebug() << "box test 10";	
	//test a1 x b2
	{
		Vector3f a0(0,0,a->m_size[2]/2);
		Vector3f b0(0, 0, b->m_size[2]/2);
		b0 = rel * b0;
			
		if (!_testSA(a0, b0, a->m_size, b_size_in_a, t)) return false;
	}
	//qDebug() << "box test 11";	
	return true;
}

void boxGetSupportPlane(const Box* a, const Vector3f& s, SPlane& out_plane)
{
	out_plane.n = s;

	Vector3f aWrld = a->m_rot * Vector3f(a->m_size[0]/2,0,0); 
	Vector3f bWrld = a->m_rot * Vector3f(0,a->m_size[1]/2,0); 
	Vector3f cWrld = a->m_rot * Vector3f(0,0,a->m_size[2]/2); 

	Vector3f bVs[6];	
	getBoxVerticies(a, bVs);
	int i,j;
	for (i=j=0; i<6; ++i)
	{	
		float tmp_d = -(bVs[i][0]*s[0] + bVs[i][1]*s[1]+bVs[i][2]*s[2]);
		for (; j<6; ++j)
		{
			if (bVs[j][0]*s[0] + bVs[j][1]*s[1]+bVs[j][2]*s[2] > -tmp_d)
				break;
		}
		if (j == 6)
		{
			out_plane.d = tmp_d;
			break;
		}
	}
}

float boxBoxSupportDist(const Box* a, const Vector3f& in_s)
{
	float res = FLT_MAX;
	Vector3f bVerts[6];	
	getBoxVerticies(a, bVerts);
	
	for (int i=0; i<6; ++i)
	{	
		res = std::min(res, in_s.dot(bVerts[i]));
	}
//	qDebug() << "bbsd " << res;
	return res;
}

float boxBoxCheckDirection(const Box* a, const Box* b, const Vector3f& in_s)
{
	float res = -INFINITY;
	if (in_s.dot(in_s) > 0.001)
	{
		Vector3f s = in_s.normalized();
		res = boxBoxSupportDist(a, s) + boxBoxSupportDist(b, -s);	
	}
	//qDebug() << "bbcd " << res;
	return res;
}

void boxBoxGetSeparationDirAndDepth(const Box* a, const Box* b, Vector3f& out_s, float& out_d)
{
	out_s = Vector3f(0.0f, 0.0f, 0.0f);
	out_d = -INFINITY;
	Vector3f tmp;
	float d;

	//check each face for *a*
	static const Vector3f arr[] = {Vector3f(1,0,0), Vector3f(0,1,0), Vector3f(0,0,1), Vector3f(-1,0,0), Vector3f(0,-1,0), Vector3f(0,0,-1)};
	std::vector<Vector3f> loc_n(arr, arr+sizeof(arr)/sizeof(arr[0])); 
	for (int i=0; i<loc_n.size(); ++i)
	{
		tmp = a->m_rot * loc_n[i];
		d = boxBoxCheckDirection(a,b,-tmp);
		if (d>out_d)
		{ 
			out_d = d;
			out_s = -tmp;
		}
	}

	//check each face for *b*
	for (int i=0; i<loc_n.size(); ++i)
	{
		tmp = b->m_rot * loc_n[i];
		d = boxBoxCheckDirection(a,b,tmp);
		if (d>out_d)
		{ 
			out_d = d;
			out_s = tmp;
		}
	}

	//check edges, 9 cases
	for (int i=0; i<3;i++)
	for (int j=0; j<3;j++)
	{
		tmp = (a->m_rot*loc_n[i]).cross(b->m_rot*loc_n[j]);
		d = boxBoxCheckDirection(a,b,tmp);
		if (d>out_d)
		{ 
			out_d = d;
			out_s = tmp;
		}
		d = boxBoxCheckDirection(a,b,-tmp);
		if (d>out_d)
		{ 
			out_d = d;
			out_s = -tmp;
		}
	}

	//qDebug() << out_d << out_s;
	if (out_d <= 0)
		return;
	//TODO: check vertex-vertex and vertex-edge cases	
}

void getVerticiesOnSupportPlane(const Box* b, const SPlane& p, Vector3f out_arr[4], size_t& out_size)
{
	const float TOLERANCE = 10e-3f;
	out_size = 0;		
	Vector3f bVs[6];	
	getBoxVerticies(b, bVs);
	
	size_t indx = 0;	
	for (int i=0; i<6; ++i)
	{	
		float d = p.n[0]*bVs[i][0] + p.n[1]*bVs[i][1] + p.n[2]*bVs[i][2] + p.d;
		if (fabs(d) <= TOLERANCE)
			out_arr[indx++] = bVs[i];
	}
	out_size = indx;
	qDebug() << indx;
	assert(indx >=0 && indx <= 4);
}

void collide(Box* a, Box* b, Contact* c, int& out_size)
{
	assert(out_size > 0);
	out_size = 0;

	Vector3f separationAxe;
	float penDepth;
	boxBoxGetSeparationDirAndDepth(a,b,separationAxe,penDepth);
	if (penDepth < 0)
	{
		SPlane p;
		boxGetSupportPlane(a, -separationAxe, p);
		DebugManager()->DrawPlane(p.n, p.d);	 

		Vector3f vs1[4];
		size_t cnt1;
		getVerticiesOnSupportPlane(a, p, vs1, cnt1);
		assert(cnt1 >= 0 && cnt1 <=4);

		Vector3f vs2[4];
		size_t cnt2;
		getVerticiesOnSupportPlane(b, p, vs2, cnt2);
		assert(cnt2 >= 0 && cnt2 <=4);

		qDebug() << "BOX OVERLAP" << cnt1 << cnt2;
		
	}
	else
	{
		SPlane p;
		boxGetSupportPlane(a, -separationAxe, p);
		DebugManager()->DrawPlane(p.n, p.d);	 
	}
}

#endif//_COLLISION_H_

