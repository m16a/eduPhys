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

void collide(Sphere* a, Sphere* b, Contact* c, int& out_size);

bool overlap(Sphere* a, Box* b);

float distPointPlane(Vector3f plane_normal, float d, Vector3f point);

bool isPointInsideBox(const Vector3f size, const Vector3f p);

void collide(Sphere* sphere, Box* b, Contact* c, int& out_size);

inline bool _testSA(const Vector3f& a0, const Vector3f& b0, const Vector3f& a_size, const Vector3f& b_in_a_size, const Vector3f& t);

bool overlap(Box* a, Box* b);

void boxGetSupportPlane(const Box* a, const Vector3f& s, SPlane& out_plane);

float boxBoxSupportDist(const Box* a, const Vector3f& in_s);

float boxBoxCheckDirection(const Box* a, const Box* b, const Vector3f& in_s);

void boxBoxGetSeparationDirAndDepth(const Box* a, const Box* b, Vector3f& out_s, float& out_d);

void reorderRectVerticies(const Vector3f n, Vector3f out_arr[4]);

void getVerticiesOnSupportPlane(const Box* b, const SPlane& p, float tolerance, Vector3f out_arr[4], size_t& out_size);

Vector3f projectVectorOntoPlane(const Vector3f& v, const Vector3f& plane_normal, const Vector3f& plane_point);

Vector3f projectVectorOntoPlane(const Vector3f& v, const SPlane& plane);

void intersectSegmentSegment(const Vector3f& a1, const Vector3f& a2, const Vector3f& b1, const Vector3f& b2, Vector3f out_vrts[2], int& out_cnt, Vector3f& out_normal);

void clampSegmentWithFacesEdge(const Vector3f faceEdge[2], const Vector3f& thirdVrtx, Vector3f in_out_segment[2]);

void intersectFaceSegment(const Vector3f face[4], const Vector3f segment[2], Vector3f /*out_*/clampedSegment[2], Vector3f& out_normal);

void intersectFaceFace(const Vector3f face1[4], const Vector3f face2[4], const SPlane& contact_plane, Vector3f out_res[8], size_t& out_size, Vector3f& out_normal);

//NB: contact normal should be pointed outward *a* (a -> b)
void collide(Box* a, Box* b, Contact* c, int& out_size);

#endif//_COLLISION_H_

