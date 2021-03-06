#include "collision.h"
#include "core.h"

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

	const Vector3f* bVs;	
	getBoxVerticies(*a, &bVs);
	
	//Debug() << "getSpPlane:" << a->m_id << "n:" << s;
	out_plane.d = -INFINITY;
	for (int i=0; i<8; ++i)
	{	
		float tmp_d = -(bVs[i][0]*s[0] + bVs[i][1]*s[1]+bVs[i][2]*s[2]);
		if (tmp_d > out_plane.d)
			out_plane.d = tmp_d;
	}
}

float boxBoxSupportDist(const Box* a, const Vector3f& in_s)
{
	float res = FLT_MAX;
	const Vector3f* bVerts;	
	getBoxVerticies(*a, &bVerts);
	
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

void boxBoxGetSeparationDirAndDepth(Box* a, Box* b, Vector3f& out_s, float& out_d)
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
	out_s.normalize();
	//qDebug() << out_d << out_s;
	if (out_d <= 0)
		return;

	//TODO: check vertex-vertex and vertex-edge cases	
	qCritical() << "TODO: check vertex-vertex and vertex-edge cases";
	//assert(0);
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

void getVerticiesOnSupportPlane(const Box* b, const SPlane& p, float tolerance, Vector3f out_arr[4], size_t& out_size)
{
	out_size = 0;		
	const Vector3f* bVs;	
	getBoxVerticies(*b, &bVs);
	
	size_t indx = 0;	
	//qDebug() << "N:" << p.d << p.n;
	for (int i=0; i<8; ++i)
	{	
		float d = p.n[0]*bVs[i][0] + p.n[1]*bVs[i][1] + p.n[2]*bVs[i][2] + p.d;
		qDebug() << "test vertex" << bVs[i];
		qDebug() << "d:" << d;
		//if (fabs(d) <= tolerance)
		//if (d < 0 || d < 0.0001)
		if (d > 0)
		{
			out_arr[indx++] = bVs[i];
			//qDebug() << "pass";
		}
		//else qDebug() << "NOTpass";
	}
	if (indx < 0 || indx > 4)
	{
		Debug() << "veticies cnt:"<< indx;
		assert(indx >=0 && indx <= 4);
	}

	if (3 == indx)
	{
		//add fours vertex to make a face
		for (int i=0; i<8; ++i)
		{
			//skip already kept vertex
			if (isVectorsEqual(out_arr[0], bVs[i]) ||
					isVectorsEqual(out_arr[1], bVs[i]) ||  
					isVectorsEqual(out_arr[2], bVs[i]))
					continue;
				
			//test 4 points on plane
			Matrix3f m;
			m << out_arr[0]-bVs[i], out_arr[1]-bVs[i], out_arr[2]-bVs[i];
			float d = m.determinant();

			//qDebug() << "lastIn" << m_lastIn << planeHitPoint << pln.m_n << d;	
			if (fabs(d)<0.001)
			{
				out_arr[indx++] = bVs[i];
				break;
			}
		}
	}

	if (4 == indx)
		reorderRectVerticies(p.n, out_arr);

	out_size = indx;
}

Vector3f projectVectorOntoPlane(const Vector3f& v, const Vector3f& plane_normal, const Vector3f& plane_point)
{
	assert(fabs(plane_normal.dot(plane_normal) - 1.0f) < 0.001);
	Vector3f a = v - plane_point;
	return v - a.dot(plane_normal) * plane_normal;
}

Vector3f projectVectorOntoPlane(const Vector3f& v, const SPlane& plane)
{
	Vector3f p(0,0,0);
	if (fabs(plane.n[0]) > 0)
		p[0] = -plane.d / plane.n[0];
	else if (fabs(plane.n[1]) > 0)
		p[1] = -plane.d / plane.n[1];
	else if (fabs(plane.n[2]) > 0)
		p[2] = -plane.d / plane.n[2];
	else
	{
		assert(0);
	}

	return projectVectorOntoPlane(v, plane.n, p);
}

void intersectSegmentSegment(const Vector3f& a1, const Vector3f& a2, const Vector3f& b1, const Vector3f& b2, Vector3f out_vrts[2], int& out_cnt, Vector3f& out_normal)
{
	Vector3f d1 = a2 - a1;
	Vector3f d2 = b2 - b1;
	
	if (fabs(d1.cross(d2).norm()) > 1e-6)//segments are not colinear
	{
		//lets try lines intersection first
		float a = d1.dot(d1);
		float b = d1.dot(d2);
		Vector3f r = a1 - b1;
		float c = d1.dot(r);
		float e = d2.dot(d2);
		float f = d2.dot(r);
		float d = a*e - b*b;
		float s = (b*f - c*e)/d; 	
		float t = (a*f - b*c)/d; 	

		if (s<0 || s>1 || t<0 || t>1)
		{	
			//qDebug() << "s:" << s << "t:" << t;	
			//TODO:check commented warning below
			//qWarning() << "segment-segment intersection: point lies out of segment, use more sofisticated method";
			out_cnt = 0;
		}
		else
		{
			out_cnt = 1;
			Vector3f p1 = (a1 + s*d1);
			Vector3f p2 = (b1 + t*d2);
			out_normal = p1 - p2;
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
	//qDebug() << "projected:" << s1 << s2;
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

void intersectFaceFace(const Vector3f face1[4], const Vector3f face2[4], const SPlane& contact_plane, Vector3f out_res[8], size_t& out_size, Vector3f& out_normal)
{
	out_size = 0;
	//project both faces to contact plane
	Vector3f prjFace1[4], prjFace2[4];
	for (int i=0; i<4; ++i)
	{
		prjFace1[i] = projectVectorOntoPlane(face1[i], contact_plane);
		prjFace2[i] = projectVectorOntoPlane(face2[i], contact_plane);
	}
	
	//bild 4x4 matrices to determime how vertices of one face lay towards edges of another face
	Matrix4i B1, B2;
	for (int i=0; i<4; ++i)
		for (int j=0; j<4; ++j)
		{
			{
				//test face1 to face2
				const Vector3f a = prjFace1[i] - prjFace2[j];
				const Vector3f b = prjFace2[(j+2) % 4] - prjFace2[(j+1) % 4];
				B1(i,j) = a.dot(b) > 0 ? 1 : 0;
			}

			{
				//test face2 to face1
				const Vector3f a = prjFace2[i] - prjFace1[j];
				const Vector3f b = prjFace1[(j+2) % 4] - prjFace1[(j+1) % 4];
				B2(i,j) = a.dot(b) > 0 ? 1 : 0;
			}
		}	

	//qDebug() << "B1:" << B1 << "B2:"<< B2;
	//process matrices
	for (int i=0; i<4; ++i)
	{
		int tmp1 = 1;
		int tmp2 = 1;
		for (int j=0; j<4; ++j)
		{
			tmp1 *= B1(i,j);	
			tmp2 *= B2(i,j);	
		}

		
		{
			//case when vertex is inside other face
			if (1 == tmp1)
				out_res[out_size++] = prjFace1[i];

			if (1 == tmp2)
				out_res[out_size++] = prjFace2[i];
		}
	}

	//no more then 8 contacts can be
	assert(8 >= out_size);
	
	//case when one face is liee completly inside another
	if (4 == out_size)
		return;

	//check edge-edge intersections
	for (int i=0; i<4; ++i)
		for (int j=0; j<4; ++j)
		{
			if (B1(i,j) ^ B1((i+1)%4,j) && B2(j,i) ^ B2((j+1)%4,i))
			{
				//segment i,i+1 intersects with  j,j+1
				Vector3f out_vrts[2];
				int cnt = 0;	
				Vector3f n;
				intersectSegmentSegment(prjFace1[i], prjFace1[(i+1)%4], prjFace2[j], prjFace2[(j+1)%4], out_vrts, cnt, n);
				//qDebug() << "Seg-seg intersection:";
				//DumpAll();
				//Debug() << "" << prjFace1[i] << prjFace1[(i+1)%4];
				//Debug() << "" << prjFace2[j] << prjFace2[(j+1)%4];;

				//TODO: proof assert commenting
				//assert(cnt > 0);
				
				for (int k=0; k<cnt; ++k)
					out_res[out_size++] = out_vrts[k];
			}
		}
	assert(8 >= out_size);
}

//NB: contact normal should be pointed outward *a* (a -> b)
void collide(Box* a, Box* b, Contact* c, int& out_size)
{
	out_size = 0;

	Vector3f separationAxe;
	float penDepth;

	//TODO:change sepraration axe direction
	//separation axe is pointed outward *b* (b -> a)
	boxBoxGetSeparationDirAndDepth(a,b,separationAxe,penDepth);
	separationAxe.normalized();
	if (penDepth < 0)
	{
		penDepth = std::min(penDepth, -Core::COLLISION_DEPTH_TOLERANCE); 

		qDebug() << "box-box[" << a->m_id << "-" << b->m_id << "] SATdepth:" << penDepth;
		SPlane pln1;
		boxGetSupportPlane(a, separationAxe, pln1);

		SPlane pln2;
		boxGetSupportPlane(b, -separationAxe, pln2);

		//Debug() << "Ds:" << pln1.d << " " << pln2.d;

		//DebugManager()->DrawPlane(pln1.n, pln1.d);	 
		//DebugManager()->DrawPlane(pln2.n, pln2.d);	 

		Debug() << "spPlane " << pln1.n << pln1.d;

		Vector3f vs1[4];
		size_t cnt1;
		getVerticiesOnSupportPlane(a, pln2, -penDepth*1.1, vs1, cnt1);
		qDebug() << "cnt1:" << cnt1;
		assert(cnt1 > 0 && cnt1 <=4);

		Vector3f vs2[4];
		size_t cnt2;
		SPlane tmp_p = pln1;
		tmp_p.n *= -1;
		tmp_p.d *= -1;

		getVerticiesOnSupportPlane(b, pln1, -penDepth*1.1, vs2, cnt2);
		qDebug() << "cnt2:" << cnt2;
		assert(cnt2 > 0 && cnt2 <=	4);

		qDebug() << "\t" << "id1:" << a->m_id << "(" << cnt1 << ")" << b->m_id << "(" << cnt2 <<")" << "penDepth:" << penDepth;
/*	
		for (int i=0; i<cnt1; ++i)
			DebugManager()->DrawSphere(vs1[i], 0.02, Color(0,0,1,1));	 

		for (int i=0; i<cnt2; ++i)
			DebugManager()->DrawSphere(vs2[i], 0.02, Color(0,0,1,1));	 
*/
		if (cnt1 == 1)
		{
			out_size = 1;
			c[0].pt = vs1[0];
			c[0].n = -pln1.n;//TODO:check statement	
		}
		else if (cnt2 == 1)
		{
			out_size = 1;
			c[0].pt = vs2[0];
			c[0].n = -pln1.n;//TODO:check statement	
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
			
			//Debug() << "edge-edge" << vs1[0] << vs1[1] << vs2[0] << vs2[1] << c[0].pt << norm;
		}
		else if (cnt2 == 2 && cnt1 == 4 || cnt1 == 2 && cnt2 == 4)//edge-face
		{
			//qDebug() << "Edge-face intersection";
			Vector3f tmp[2], norm;
			if (cnt1 == 4)
			{
				intersectFaceSegment(vs1, vs2, tmp, norm);
				//Debug() << "interest points:" << vs2[0] << vs2[1];
				//Debug() << "interest points2:" << vs1[0] << vs1[1]<< vs1[2] << vs1[3];
			}
			else
			{
				intersectFaceSegment(vs2, vs1, tmp, norm);
			}

			//Debug() << "normNeg:" << a->m_pos << b->m_pos << tmp[0];
			//adjust normal a->b
			if (norm.dot(b->m_pos-tmp[0]) < 0)
			{
				norm *= -1.0f;
				//qDebug() << "normal negation";
			}

			out_size = 2;
			c[0].pt = tmp[0];
			c[1].pt = tmp[1];
			c[0].n = c[1].n = norm;
		}
		else if (cnt2 == 4 && cnt1 == 4)//face-face
		{
			Vector3f res[8], normal;
			size_t res_cnt = 0;
			intersectFaceFace(vs1, vs2, pln1, res, res_cnt, normal);
			
			for (int i=0; i<res_cnt; ++i)
			{
				c[i].pt = res[i];
				c[i].n =	-pln1.n;
			}
			out_size = res_cnt;
		}
		else
		{
			qCritical() << "Wrong intersection verticies count:" << cnt1 << cnt2;
			assert(0);
		}
		
		for (int i=0; i<out_size; ++i)
		{
			//most naive case when all contacts have same depth
			c[i].depth = penDepth;
		}
	}
	else
	{
		SPlane p;
		boxGetSupportPlane(a, separationAxe, p);
		//DebugManager()->DrawPlane(p.n, p.d);	 
	}
}

