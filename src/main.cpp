#include <iostream>
#include <QtDebug>

#include "application.h"
#include "my_utils.h"
using namespace Eigen;

int main(int argc, char *argv[])
{
  std::cout << "---------------------------------------------------------------------------\n";
	std::cout << "eduPhys v1.0\n\n";
	std::cout << "\tControls:\n";
  std::cout << "\tP\t\tpause/unpause simulation\n";
  std::cout << "\tN\t\tperfrom single step in paused mode\n";
  std::cout << "\tC\t\tsave current scene to file \n";
  std::cout << "\tV\t\tload scene from file\n";
	std::cout << "\tLMB\t\tselect entity\n";
	std::cout << "\n";
  std::cout << "\tNavigation:\n";
  std::cout << "\tW/A/S/D\t\tmove the camera\n";
  std::cout << "\tUp/Down\t\tcamera pitch\n";
  std::cout << "\tLeft/Right\tcamera yaw\n";

  std::cout << "---------------------------------------------------------------------------\n";

	srand(time(NULL));
  QApplication app(argc, argv);
  CApplication demo;
  demo.resize(800, 600);
	demo.move(700, 100);

	std::vector<IPhysEnt*>& objcts = demo.getCore()->m_objects;
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
#if 0
	//ODE comparision test for 1 box rotation and collision
  Box* s3 = new Box(0.0f, Vector3f(0.1f, 1.f, 1.f), true);
  s3->m_pos = Vector3f(-1.0f, 0.0f, 1.0f);
  s3->m_id = 3;
	demo.getCore()->m_objects.push_back(s3);

  Box* s4 = new Box(0.0f, Vector3f(0.1f, 1.f, 1.f), true);
  s4->m_pos = Vector3f(1.0f, 0.0f, 1.0f);
  s4->m_id = 40;
	demo.getCore()->m_objects.push_back(s4);

  Box* s9 = new Box(1.0f, Vector3f(0.1f, 0.1f, 0.1f), false);
  s9->m_pos = Vector3f(0.0f, 0.0f, 0.0f);
  s9->m_rot = quatFromPYRAngles(45, 45, 0);
	qDebug() << s9->m_rot;
  s9->m_id = 9;
	s9->m_v = Vector3f(-0.3f, 0.0f, 0.3f);
	s9->m_w = Vector3f(0.f, 0.0f, 2.0f);

	//s9->AddImpulse(Vector3f(1,0,0), Vector3f(-0.05, 0.05, 1));
	demo.getCore()->m_objects.push_back(s9);
#endif

#if 1
	//ground
  Box* s3 = new Box(0.0f, Vector3f(2.5f, 2.5f, 0.1f), true);
  s3->m_pos = Vector3f(0.0f, 0.0f, -0.05f);
  s3->m_id = 3;
	objcts.push_back(s3);
#endif

#if 0
	// box falls on ground
  Box* s9 = new Box(1.0f, Vector3f(0.3f, 0.2f, 0.1f), false);
  s9->m_pos = Vector3f(0.0f, 0.0f, 0.5f);
  //s9->m_rot = quatFromPYRAngles(45, 45, 0);
	qDebug() << s9->m_rot;
  s9->m_id = 9;
	//s9->m_v = Vector3f(-0.3f, 0.0f, 0.0f);
	//s9->AddImpulse(Vector3f(1,0,0), Vector3f(-0.05, 0.05, 1));
	
	s9->m_isGravity = true;
	objcts.push_back(s9);
#endif

#if 1
  Box* s9 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s9->m_pos = Vector3f(0.0f, 0.0f, 0.1f);
  s9->m_id = 9;
	s9->m_isGravity = true;
	objcts.push_back(s9);

  Box* s10 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s10->m_pos = Vector3f(0.2f, 0.0f, 0.1f);
  s10->m_id = 10;
	s10->m_isGravity = true;
	objcts.push_back(s10);

  Box* s11 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s11->m_pos = Vector3f(-0.2f, 0.0f, 0.1f);
  s11->m_id = 11;
	s11->m_isGravity = true;
	objcts.push_back(s11);

  Box* s12 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s12->m_pos = Vector3f(-0.4f, 0.0f, 0.1f);
  s12->m_id = 12;
	s12->m_isGravity = true;
	objcts.push_back(s12);

  Box* s13 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s13->m_pos = Vector3f(0.4f, 0.0f, 0.1f);
  s13->m_id = 13;
	s13->m_isGravity = true;
	objcts.push_back(s13);

  Box* s20 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s20->m_pos = Vector3f(0.1f, 0.0f, 0.3f);
  s20->m_id = 20;
	s20->m_isGravity = true;
	objcts.push_back(s20);

  Box* s21 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s21->m_pos = Vector3f(0.3f, 0.0f, 0.3f);
  s21->m_id = 21;
	s21->m_isGravity = true;
	objcts.push_back(s21);

  Box* s22 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s22->m_pos = Vector3f(-0.1f, 0.0f, 0.3f);
  s22->m_id = 22;
	s22->m_isGravity = true;
	objcts.push_back(s22);

  Box* s23 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s23->m_pos = Vector3f(-0.3f, 0.0f, 0.3f);
  s23->m_id = 23;
	s23->m_isGravity = true;
	objcts.push_back(s23);

  Box* s30 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s30->m_pos = Vector3f(0.0f, 0.0f, 0.5f);
  s30->m_id = 30;
	s30->m_isGravity = true;
	objcts.push_back(s30);

  Box* s31 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s31->m_pos = Vector3f(0.2f, 0.0f, 0.5f);
  s31->m_id = 31;
	s31->m_isGravity = true;
	objcts.push_back(s31);

  Box* s32 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s32->m_pos = Vector3f(-0.2f, 0.0f, 0.5f);
  s32->m_id = 32;
	s32->m_isGravity = true;
	objcts.push_back(s32);

  Box* s40 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s40->m_pos = Vector3f(0.1f, 0.0f, 0.7f);
  s40->m_id = 40;
	s40->m_isGravity = true;
	objcts.push_back(s40);

  Box* s41 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s41->m_pos = Vector3f(-0.1f, 0.0f, 0.7f);
  s41->m_id = 41;
	s41->m_isGravity = true;
	objcts.push_back(s41);

  Box* s50 = new Box(1.0f, Vector3f(0.2f, 0.2f, 0.2f), false);
  s50->m_pos = Vector3f(0.0f, 0.0f, 0.9f);
  s50->m_id = 50;
	s50->m_isGravity = true;
	objcts.push_back(s50);

  Box* s60 = new Box(0.001f, Vector3f(0.1f, 0.1f, 0.1f), false);
  s60->m_pos = Vector3f(0.0f, 3.0f, 0.3f);
	s60->m_v = Vector3f(-0.0f, -6.0f, 0.0f);
  s60->m_id = 60;
	//s60->m_isGravity = true;
	objcts.push_back(s60);
#endif

#if 0
	//2 box collision
  Box* s3 = new Box(0.0f, Vector3f(0.1f, 1.f, 1.f), true);
  s3->m_pos = Vector3f(-1.0f, 0.0f, 1.0f);
  s3->m_id = 3;
	demo.getCore()->m_objects.push_back(s3);

  Box* s4 = new Box(0.0f, Vector3f(0.1f, 1.f, 1.f), true);
  s4->m_pos = Vector3f(1.0f, 0.0f, 1.0f);
  s4->m_id = 40;
	demo.getCore()->m_objects.push_back(s4);

  Box* s9 = new Box(1.0f, Vector3f(0.1f, 0.1f, 0.1f), false);
  s9->m_pos = Vector3f(0.5f, 0.05f, 1.05f);
  //s9->m_rot = quatFromPYRAngles(45, 45, 0);
	qDebug() << s9->m_rot;
  s9->m_id = 9;
	s9->m_v = Vector3f(-0.3f, 0.0f, 0.0f);
	//s9->AddImpulse(Vector3f(1,0,0), Vector3f(-0.05, 0.05, 1));
	demo.getCore()->m_objects.push_back(s9);

  Box* s20 = new Box(1.0f, Vector3f(0.1f, 0.1f, 0.1f), false);
  s20->m_pos = Vector3f(-0.5f, 0.0f, 1.0f);
  //s20->m_rot = quatFromPYRAngles(45, 45, 0);
  s20->m_id = 20;
	s20->m_v = Vector3f(0.3f, 0.0f, 0.0f);
	demo.getCore()->m_objects.push_back(s20);
#endif

#if 0
	//big scene. cubes inside box
  Box* s3 = new Box(0.0f, Vector3f(0.2f, 6.f, 6.f), true);
  s3->m_pos = Vector3f(-3.1f, 0.0f, 0.0f);
  s3->m_id = 3;
	demo.getCore()->m_objects.push_back(s3);

  Box* s4 = new Box(0.0f, Vector3f(0.2f, 6.f, 6.f), true);
  s4->m_pos = Vector3f(3.1f, 0.0f, 0.0f);
  s4->m_id = 40;
	demo.getCore()->m_objects.push_back(s4);

  Box* s5 = new Box(0.0f, Vector3f(6.f,.2f,6.f), true);
  s5->m_pos = Vector3f(.0f, -3.1f, .0f);
  s5->m_id = 5;
	demo.getCore()->m_objects.push_back(s5);

  Box* s6 = new Box(0.0f, Vector3f(6.f,.2f,6.f), true);
  s6->m_pos = Vector3f(.0f, 3.1f, .0f);
  s6->m_id = 6;
	demo.getCore()->m_objects.push_back(s6);

  Box* s7 = new Box(0.0f, Vector3f(6.f,6.f,.2f), true);
  s7->m_pos = Vector3f(.0f,.0f,-3.1f);
  s7->m_id = 7;
	demo.getCore()->m_objects.push_back(s7);

  Box* s8 = new Box(0.0f, Vector3f(6.f,6.f,.2f), true);
  s8->m_pos = Vector3f(.0f,.0f,3.1f);
  s8->m_id = 8;
	demo.getCore()->m_objects.push_back(s8);


  Box* s9 = new Box(1.0f, Vector3f(1.f, 1.f, 1.f), false);
  s9->m_pos = Vector3f(0.0f, 0.0f, 1.0f);
  s9->m_rot = quatFromPYRAngles(45, 45, 0);
  s9->m_id = 9;
	s9->m_v = Vector3f(-1.f, 0.4f, 0.0f);
	demo.getCore()->m_objects.push_back(s9);

  Box* s20 = new Box(1.0f, Vector3f(0.5f, 0.5f, 0.5f), false);
  s20->m_pos = Vector3f(0.0f, 0.0f, -1.0f);
  s20->m_rot = quatFromPYRAngles(45, 45, 0);
  s20->m_id = 20;
	s20->m_v = Vector3f(-1.f, 0.4f, 0.2f);
	demo.getCore()->m_objects.push_back(s20);

  Box* s21 = new Box(1.0f, Vector3f(0.4f, 0.8f, 1.2f), false);
  s21->m_pos = Vector3f(0.0f, 1.0f, 0.0f);
  s21->m_rot = quatFromPYRAngles(45, 45, 0);
  s21->m_id = 21;
	s21->m_v = Vector3f(1.f, 0.4f, 0.2f);
	demo.getCore()->m_objects.push_back(s21);

  Box* s22 = new Box(1.0f, Vector3f(0.4f, 0.4f, 0.4f), false);
  s22->m_pos = Vector3f(0.0f, -1.0f, 0.0f);
  s22->m_rot = quatFromPYRAngles(45, 45, 0);
  s22->m_id = 22;
	s22->m_v = Vector3f(1.f, -0.4f, 0.2f);
	demo.getCore()->m_objects.push_back(s22);
#endif

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

	for (int i=0; i<objcts.size(); ++i)
		objcts[i]->UpdatedPosRot();
	
	demo.show();
	return app.exec();
}

