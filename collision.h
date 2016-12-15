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

	Contact():pt(Vector3f(0,0,0)), n(Vector3f(0,0,0)),depth(1000.0f){};
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

	Vector3f bVs[8];	
	getBoxVerticies(a, bVs);
	int i,j;
	for (i=j=0; i<8; ++i)
	{	
		float tmp_d = -(bVs[i][0]*s[0] + bVs[i][1]*s[1]+bVs[i][2]*s[2]);
		for (; j<8; ++j)
		{
			if (bVs[j][0]*s[0] + bVs[j][1]*s[1]+bVs[j][2]*s[2] < -tmp_d)
				break;
		}
		if (j == 8)
		{
			out_plane.d = tmp_d;
			break;
		}
	}
}

float boxBoxSupportDist(const Box* a, const Vector3f& in_s)
{
	float res = FLT_MAX;
	Vector3f bVerts[8];	
	getBoxVerticies(a, bVerts);
	
	for (int i=0; i<8; ++i)
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
void reorderRectVerticies(const Vector3f n, Vector3f out_arr[4])
{
	//http://stackoverflow.com/questions/242404/sort-four-points-in-clockwise-order
	//Possible cases:
	//1.ABCD
	//2.ABDC
	//3.ACBD
	//4.ACBD
	//5.ADBC
	//6.ADCB
	
	//qDebug() << "verticies reordering";
	//qDebug() << "\tpre:\t" << out_arr[0] << out_arr[1] << out_arr[2] << out_arr[3];
	const Vector3f a = out_arr[1]-out_arr[0];
	const Vector3f b = out_arr[2]-out_arr[0];
	const float orntn = n.dot(a.cross(b));
	bool badOrientation = false;	
	if (orntn > 0.0f)
	{
		//cases 1,2,5
		//check ACD	
		const Vector3f a1 = out_arr[2]-out_arr[0];
		const Vector3f b1 = out_arr[3]-out_arr[0];
		const float orntn1 = n.dot(a1.cross(b1));
		if (orntn1 > 0.0f)
		{
			//case 1. we have ordered vertecies, do nothing
		}
		else if (orntn1 < 0.0f)
		{
			//case 2 or 5
			//check BDC
			const Vector3f a2 = out_arr[3]-out_arr[1];
			const Vector3f b2 = out_arr[2]-out_arr[1];
			const float orntn2 = n.dot(a2.cross(b2));
			
			if (orntn2 > 0.0f)
			{
				//case 2
				//swap D C
				std::swap(out_arr[3], out_arr[2]);			
			}
			else if (orntn2 < 0.0f)
			{
				//case 5
				//swap A D 
				std::swap(out_arr[0], out_arr[3]);			
			}
			else 
				badOrientation = true;
		}
		else 
			badOrientation = true;
		
	}
	else if (orntn < 0.0f)
	{
		//cases 3,4,6
		//check ADC
		const Vector3f a1 = out_arr[3]-out_arr[0];
		const Vector3f b1 = out_arr[2]-out_arr[0];
		const float orntn1 = n.dot(a1.cross(b1));
		if (orntn1 > 0.0f)
		{
			//case 6
			//swap D B
			std::swap(out_arr[3], out_arr[1]);
		}
		else if (orntn1 < 0.0f)
		{
			//case 3 or 4
			//check CBD
			const Vector3f a2 = out_arr[1]-out_arr[2];
			const Vector3f b2 = out_arr[3]-out_arr[2];
			const float orntn2 = n.dot(a2.cross(b2));
			if (orntn2 > 0.0f)
			{
				//case 3
				//swap C B
				std::swap(out_arr[2], out_arr[3]);	
			}
			else if (orntn2 < 0.0f)
			{
				//case 4
				//swap A B
				std::swap(out_arr[0], out_arr[1]);	
			}
			else
				badOrientation = true;
		}	
		else 
			badOrientation = true;
	}
	else 
		badOrientation = true;

	if (badOrientation)
	{
		qCritical() << "bad rect verticies";
		assert(0);
	}

	//qDebug() << "\tpost:\t" << out_arr[0] << out_arr[1] << out_arr[2] << out_arr[3];
}

void getVerticiesOnSupportPlane(const Box* b, const SPlane& p, Vector3f out_arr[4], size_t& out_size)
{
	const float TOLERANCE = 1e-2f;
	out_size = 0;		
	Vector3f bVs[8];	
	getBoxVerticies(b, bVs);
	
	size_t indx = 0;	
	for (int i=0; i<8; ++i)
	{	
		float d = p.n[0]*bVs[i][0] + p.n[1]*bVs[i][1] + p.n[2]*bVs[i][2] + p.d;
		//qDebug() << "test vertex" << bVs[i];
		if (fabs(d) <= TOLERANCE)
		{
			out_arr[indx++] = bVs[i];
			//qDebug() << "pass";
		}
		//else qDebug() << "NOTpass";
	}
	out_size = indx;
	//qDebug() << indx;
	assert(indx >=0 && indx <= 4 && 3 != indx);

	if (4 == indx)
	{
		reorderRectVerticies(p.n, out_arr);
	}
}

Vector3f projectVectorOntoPlane(const Vector3f& v, const Vector3f& plane_normal, const Vector3f& plane_point)
{
	assert(fabs(plane_normal.dot(plane_normal) - 1.0f) < 0.001);
	Vector3f a = v - plane_point;
	return v - a.dot(plane_normal) * plane_normal;
}

void intersectSegmentSegment(const Vector3f& a1, const Vector3f& a2, const Vector3f& b1, const Vector3f& b2, Vector3f out_vrts[2], int& out_cnt, Vector3f& out_normal)
{
	Vector3f d1 = a2 - a1;
	Vector3f d2 = b2 - b1;
	
	if (fabs(d1.cross(d2).norm()) > 1e-2)//segment are not colinear
	{
		//lets try lines intersection first
		out_cnt = 1;	
		float a = d1.dot(d1);
		float b = d1.dot(d2);
		Vector3f r = a1 - b1;
		float c = d1.dot(r);
		float e = d2.dot(d2);
		float f = d2.dot(r);
		float d = a*c - b*b;
		float s = (b*f - c*e)/d; 	
		float t = (a*f - c*b)/d; 	
		if (s<0 || s >1 || t<0 || t>1)
		{	
			//TODO:check commented warning below
			//qWarning() << "segment-segment intersection: point lies out of segment, use more sofisticated method";
			out_cnt = 0;
		}else
		{
			out_cnt = 1;
			Vector3f p1 = (a1 + s*d1);
			Vector3f p2 = (b1 + t*d2);
			out_normal = p2 - p1;
			out_normal.normalize(); 
			out_vrts[0] = (p1 + p2) / 2.0f;
		}
	}
	else //colinear case
	{
		//drop case when segments don't lie on one line
		if ((d1.cross(a2-b1)).norm() > 0.0001)
		{
			out_cnt = 0;	
		}
		else
		{
			out_cnt = 0;	
			qWarning() << "Implement intersection of colinear segements";
			//assert(0);
		}
	}
}

void clampSegmentWithFacesEdge(const Vector3f faceEdge[2], const Vector3f& thirdVrtx, Vector3f in_out_segment[2])
{
	Vector3f outVs[2], notUsed;
	int outN = 0;
	intersectSegmentSegment(faceEdge[0], faceEdge[1], in_out_segment[0], in_out_segment[1], outVs, outN, notUsed);
	if (outN > 0)
	{
		assert(outN == 1);//expecting one intersection point

		//decide how to clamp, what segment's point is left and what substitute with intersection result
		//build plane through point face[0], normal face[2]-face[0] and test segment edges. 
		Vector3f n = (thirdVrtx-faceEdge[0]);
		n.normalize();
		SDebugPlane p(faceEdge[0], n);
		if ((in_out_segment[0].dot(p.m_n) + p.m_d)*(thirdVrtx.dot(p.m_n) + p.m_d) < 0)
			in_out_segment[0] = outVs[0];
		else if ((in_out_segment[1].dot(p.m_n) + p.m_d)*(thirdVrtx.dot(p.m_n) + p.m_d) < 0)
			in_out_segment[1] = outVs[0];
	}
}

void intersectFaceSegment(const Vector3f face[4], const Vector3f segment[2], Vector3f /*out_*/clampedSegment[2], Vector3f& out_normal)
{
	Vector3f faceNormal = (face[0]-face[1]).cross(face[2]-face[1]);
	faceNormal.normalize();

	//Projecct segment onto face
	Vector3f s1 = projectVectorOntoPlane(segment[0], faceNormal, face[0]);
	Vector3f s2 = projectVectorOntoPlane(segment[1], faceNormal, face[0]);
	qDebug() << "projected:" << s1 << s2;
	//Clamp segment by 4 face edges
	clampedSegment[0] = s1; clampedSegment[1] = s2;

	Vector3f faceEdge[2];
	faceEdge[0] = face[0]; faceEdge[1] = face[1];	
	clampSegmentWithFacesEdge(faceEdge, face[2], clampedSegment);

	faceEdge[0] = face[1]; faceEdge[1] = face[2];	
	clampSegmentWithFacesEdge(faceEdge, face[3], clampedSegment);
	
	faceEdge[0] = face[2]; faceEdge[1]= face[3];	
	clampSegmentWithFacesEdge(faceEdge, face[0], clampedSegment);

	faceEdge[0] = face[3]; faceEdge[1] = face[0];	
	clampSegmentWithFacesEdge(faceEdge, face[1], clampedSegment);

	out_normal = faceNormal;//TODO:check normal direction
}

void collide(Box* a, Box* b, Contact* c, int& out_size)
{
	out_size = 0;

	Vector3f separationAxe;
	float penDepth;
	boxBoxGetSeparationDirAndDepth(a,b,separationAxe,penDepth);
	separationAxe.normalized();
	//qDebug() << penDepth;
	if (penDepth < 0)
	{
		SPlane p;
		boxGetSupportPlane(a, separationAxe, p);
		//DebugManager()->DrawPlane(p.n, p.d);	 
		//qDebug() << "spPlane " << p.n << p.d;
		Vector3f vs1[4];
		size_t cnt1;
		getVerticiesOnSupportPlane(a, p, vs1, cnt1);
		assert(cnt1 >= 0 && cnt1 <=4);

		Vector3f vs2[4];
		size_t cnt2;
		getVerticiesOnSupportPlane(b, p, vs2, cnt2);
		assert(cnt2 >= 0 && cnt2 <=4);

		qDebug() << "BOX OVERLAP" << cnt1 << cnt2;
	
		c[0].depth = penDepth;
		out_size = 1;
		if (cnt1 == 1)
		{
			c[0].pt = vs1[0];
			c[0].n = p.n;//TODO:check statement	
		}
		else if (cnt2 == 1)
		{
			c[0].pt = vs2[0];
			c[0].n = p.n;//TODO:check statement	
		}
		else if (cnt2 == 2 && cnt1 == 2)//edge-edge
		{
			Vector3f tmp[2], norm;
			int cnt;
			intersectSegmentSegment(vs1[0], vs1[1], vs2[0], vs2[1], tmp, cnt, norm);
			assert(cnt==1 || cnt==2);
			out_size = cnt;
			c[0].n = norm; 
			c[0].pt = tmp[0];	
			if (cnt == 2)
				c[1].pt = tmp[1];	
		}
		else if (cnt2 == 2 && cnt1 == 4 || cnt1 == 2 && cnt2 == 4)//edge-face
		{
			qDebug() << "Edge-face intersection";
			Vector3f tmp[2], norm;
			if (cnt1 == 4)
			{
				intersectFaceSegment(vs1, vs2, tmp, norm);
				qDebug() << "interest points:" << vs2[0] << vs2[1];
				qDebug() << "interest points2:" << vs1[0] << vs1[1]<< vs1[2] << vs1[3];
			}
			else
			{
				intersectFaceSegment(vs2, vs1, tmp, norm);
			}
			qDebug() << "normNeg:" << a->m_pos << b->m_pos << tmp[0];
			//adjust normal a->b
			if (norm.dot(b->m_pos-tmp[0]) < 0)
			{
				norm *= -1.0f;
				qDebug() << "normal negation";
			}

			out_size = 2;
			c[0].pt = tmp[0];
			c[1].pt = tmp[1];
			c[0].n = c[1].n = norm;
		}
		else if (cnt2 == 4 && cnt1 == 4)//face-face
		{
			qWarning() << "Implement face-face intersection";
			assert(0);
		}
		else
		{
			qCritical() << "Wrong intersection verticies count:" << cnt1 << cnt2;
			assert(0);
		}

	}
	else
	{
		SPlane p;
		boxGetSupportPlane(a, separationAxe, p);
		//DebugManager()->DrawPlane(p.n, p.d);	 
	}
}

#endif//_COLLISION_H_

