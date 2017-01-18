#include "quaternion_demo.h"
#include <iostream>

using namespace Eigen;

int main(int argc, char *argv[])
{
  std::cout << "	Navigation:\n";
  std::cout << "  left button:           rotate around the target\n";
  std::cout << "  middle button:         zoom\n";
  std::cout << "  left button + ctrl     quake rotate (rotate around camera position)\n";
  std::cout << "  middle button + ctrl   walk (progress along camera's z direction)\n";
  std::cout << "  left button:           pan (translate in the XY camera's plane)\n\n";
  std::cout << "W/A/S/D			: move the camera\n";
  std::cout << "Up/Down			: camera pitch\n";
  std::cout << "Left/Right	: camera yaw\n";
  std::cout << "R						: move the camera to initial position\n";
  std::cout << "C						: clear the animation\n";
  std::cout << "G						: add a key frame\n";
  std::cout << "---------------------------------------------------------------------------\n";

	srand(time(NULL));
  QApplication app(argc, argv);
  QuaternionDemo demo;
  demo.resize(800, 600);
	demo.move(700, 100);

  //set phys initial world
	/*
  IPhysEnt* s1 = new Sphere();
  s1->m_pos = Vector3f(1.f, 1.f, 1.0f);
  //s1->m_v = Vector3f(-10.f, 0.f, 0.f);
  s1->m_id = 1;
  s1->m_minv = 1;
	//s1->AddImpulse(Vector3f(-1.f, 0.f, 0.f) * 200.f);
	//s1->m_forces.push_back(g_Gravity);
 // mRenderingWidget->m_core.get()->m_objects.push_back(s1);

  IPhysEnt* s2 = new Sphere();
  s2->m_pos = Vector3f(-0.5f, -1.f, 1.3f);
  s2->m_id = 2;
  s2->m_minv = 1;
  //s2->m_v = Vector3f(2.f, 0.f, 0.f);
	//mRenderingWidget->m_core.get()->m_objects.push_back(s2);
	//s2->m_forces.push_back(g_Gravity);
  //
  //s2->AddAngularImpulse(Vector3f(10.f, 10.f, 0.f) * 1000.f);
  //s2->AddImpulse(Vector3f(1.f, 0.f, 0.f) * 200.f );
	*/

  Box* s3 = new Box(Vector3f(.2f,6.f,6.f), true);
  s3->m_pos = Vector3f(-3.f, 0.f, 0.0f);
  s3->m_id = 3;
	demo.getCore()->m_objects.push_back(s3);

#if 1
  Box* s4 = new Box(Vector3f(.2f,6.f,6.f), true);
  s4->m_pos = Vector3f(3.f, 0.f, 0.0f);
  s4->m_id = 40;
	demo.getCore()->m_objects.push_back(s4);

  Box* s5 = new Box(Vector3f(6.f,.2f,6.f), true);
  s5->m_pos = Vector3f(.0f, -3.1f, .0f);
  s5->m_id = 5;
	demo.getCore()->m_objects.push_back(s5);

  Box* s6 = new Box(Vector3f(6.f,.2f,6.f), true);
  s6->m_pos = Vector3f(.0f, 3.1f, .0f);
  s6->m_id = 6;
	demo.getCore()->m_objects.push_back(s6);

  Box* s7 = new Box(Vector3f(6.f,6.f,.2f), true);
  s7->m_pos = Vector3f(.0f,.0f,-3.1f);
  s7->m_id = 7;
	demo.getCore()->m_objects.push_back(s7);

  Box* s8 = new Box(Vector3f(6.f,6.f,.2f), true);
  s8->m_pos = Vector3f(.0f,.0f,3.1f);
  s8->m_id = 8;
	demo.getCore()->m_objects.push_back(s8);

  Box* s10 = new Box(Vector3f(.5f,.5f,.5f), false);
  s10->m_pos = Vector3f(.2f,.2f,1.0f);
  s10->m_id = 10;
	//demo.getCore()->m_objects.push_back(s10);
#endif

  Box* s9 = new Box(Vector3f(.5f,.5f,.5f), false);
  s9->m_pos = Vector3f(.0f,.0f,.0f);
  s9->m_id = 9;
	s9->m_v = Vector3f(-2.f, 0.f, .0f);
	demo.getCore()->m_objects.push_back(s9);

/*	
  IPhysEnt* s4 = new Box();
  s4->m_pos = Vector3f(1.f, -0.1f, 0.1f);
  s4->m_id = 4;
  s4->m_minv = 1.f;
 	//s4->m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0); 
	s4->m_rot.normalize();
	s4->m_v = Vector3f(-1.f, 0.f, 0.f);
	demo.getCore()->m_objects.push_back(s4);
*/
	demo.show();
	return app.exec();
}

