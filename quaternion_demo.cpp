// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "quaternion_demo.h"
#include "icosphere.h"
#include "sphere.h"
#include "core.h"

#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/LU>

#include <iostream>
#include <QEvent>
#include <QMouseEvent>
#include <QInputDialog>
#include <QGridLayout>
#include <QButtonGroup>
#include <QRadioButton>
#include <QDockWidget>
#include <QPushButton>
#include <QGroupBox>


#include <QtDebug>
#include <stdlib.h>
#include <time.h>

#include "my_eulerAngles.h"
#include "geometry.h"
#include "box.h"
#include "my_utils.h"
#include "rwi.h"
#include "debug_draw.h"


using namespace Eigen;

// generic linear interpolation method
template<typename T> T lerp(float t, const T& a, const T& b)
{
  return a*(1-t) + b*t;
}

// quaternion slerp
template<> Quaternionf lerp(float t, const Quaternionf& a, const Quaternionf& b)
{ return a.slerp(t,b); }

// linear interpolation of a frame using the type OrientationType
// to perform the interpolation of the orientations
template<typename OrientationType>
inline static Frame lerpFrame(float alpha, const Frame& a, const Frame& b)
{
  return Frame(lerp(alpha,a.position,b.position),
               Quaternionf(lerp(alpha,OrientationType(a.orientation),OrientationType(b.orientation))));
}

// Euler angles slerp
template<> EulerAngles<float> lerp(float t, const EulerAngles<float>& a, const EulerAngles<float>& b)
{
  EulerAngles<float> res;
  res.coeffs() = lerp(t, a.coeffs(), b.coeffs());
  return res;
}

RenderingWidget::RenderingWidget()
{
  mAnimate = false;
  mCurrentTrackingMode = TM_NO_TRACK;
  mNavMode = NavTurnAround;
  mLerpMode = LerpQuaternion;
  m_SolverStepMode = SolverContinuous;
  mRotationMode = RotationStable;
  mTrackball.setCamera(&mCamera);
  m_core.reset(new Core);

  m_performPauseStep == false;
  m_isSolverStopped = false;
  m_solverTimeFlow = SolverForwardTime;
	m_realTimeStart = clock() / float(CLOCKS_PER_SEC);
  m_lastTime = m_realTimeStart;
	m_realTime = 0.0f;
	m_physTime = 0.0f;
	
  // required to capture key press events
  setFocusPolicy(Qt::ClickFocus);
}

void RenderingWidget::grabFrame(void)
{
    // ask user for a time
    bool ok = false;
    double t = 0;
    if (!m_timeline.empty())
      t = (--m_timeline.end())->first + 1.;
    t = QInputDialog::getDouble(this, "Eigen's RenderingWidget", "time value: ",
      t, 0, 1e3, 1, &ok);
    if (ok)
    {
      Frame aux;
      aux.orientation = mCamera.viewMatrix().linear();
      aux.position = mCamera.viewMatrix().translation();
      m_timeline[t] = aux;
    }
}

void RenderingWidget::drawScene()
{
  float length = 0.2;
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitX(), Color(1,0,0,1));
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitY(), Color(0,1,0,1));
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitZ(), Color(0,0,1,1));

  // draw the fractal object
  float sqrt3 = /*internal::*/sqrt(3.);
  glLightfv(GL_LIGHT0, GL_AMBIENT, Vector4f(1.,1,1,1).data());
 // glLightfv(GL_LIGHT0, GL_DIFFUSE, Vector4f(0.5,1,0.5,1).data());
  glLightfv(GL_LIGHT0, GL_SPECULAR, Vector4f(1,1,1,1).data());
  //glLightfv(GL_LIGHT0, GL_POSITION, Vector4f(-sqrt3,-sqrt3,sqrt3,0).data());
  glLightfv(GL_LIGHT0, GL_POSITION, Vector4f(0,0,100,0).data());

  glLightfv(GL_LIGHT1, GL_AMBIENT, Vector4f(0.5,0.5,0.5,1).data());
  glLightfv(GL_LIGHT1, GL_DIFFUSE, Vector4f(0.6,0.6,0.6,0.6).data());
  glLightfv(GL_LIGHT1, GL_SPECULAR, Vector4f(1,1,1,1).data());
  glLightfv(GL_LIGHT1, GL_POSITION, Vector4f(-sqrt3,sqrt3,-sqrt3,0).data());

//  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, Vector4f(0.7, 0.7, 0.7, 1).data());
//  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vector4f(0.8, 0.75, 0.6, 1).data());
//  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Vector4f(1, 1, 1, 1).data());
//  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  
	glEnable(GL_LIGHT1);


	float currTime = clock() / float(CLOCKS_PER_SEC);
  float dt = (currTime - m_lastTime);
	m_realTime = currTime - m_realTimeStart;

	float reqStep = 0.001f;
  if (dt > reqStep)
	{
//		qDebug() << "step:" <<dt;
		m_lastTime = currTime;  
    if (!m_isSolverStopped || (m_isSolverStopped && m_performPauseStep))
    {
      float dir = (m_solverTimeFlow == SolverForwardTime) ? 1.0f : -1.0f;
      m_core.get()->Step(dir * dt);
      m_performPauseStep = false;
    }
		m_physTime+=dt;
  }
  m_core.get()->Draw();
	m_objMover.Update();	
	DebugManager()->Draw(m_isSolverStopped);	
	
	glDisable(GL_LIGHTING);
  update();

	glColor4f(1.0, 1.0, 1.0, 0.9);
	//Print camera info
	Vector3f camPos = mCamera.position();
	renderText(10,12, QString("cam pos: %1, %2, %3").arg(QString::number(camPos.x(),'f',2),QString::number(camPos.y(),'f',2),QString::number(camPos.z(),'f',2)));

	Quaternionf dir = mCamera.orientation();
	Vector3f d(1,0,0);
	d = dir * d;
	renderText(10,32, QString("cam dir: %1, %2, %3").arg(QString::number(d[0],'f',2), QString::number(d[1],'f',2), QString::number(d[2],'f',2)));
	
	Vector3f ypr = PYRFromQuat(dir); 
	renderText(10,52, QString("YPR: %1, %2, %3").arg(QString::number(ypr.x(),'f',2),QString::number(ypr.y(),'f',2),QString::number(ypr.z(),'f',2)));

	if (m_realTime > 0.01)
		renderText(10,72, QString("rT:%1, pT:%2, ratio:%3").arg(QString::number(m_realTime,'f',2),QString::number(m_physTime,'f',2),QString::number(m_physTime / m_realTime,'f',2)));

}

float getCurrTime()
{
	return clock() / float(CLOCKS_PER_SEC);
}

void RenderingWidget::keyPressEvent(QKeyEvent * e)
{
		const float camSpeed = 0.25f;
		const float camAngleSpeed = 10.0f * M_PI / 180.0f;
    switch(e->key())
    {
			//rotate camera
      case Qt::Key_Up:
			{
				Quaternionf o = mCamera.orientation();
				Quaternionf delta;
				delta = AngleAxisf(camAngleSpeed, mCamera.right());
				mCamera.setOrientation(delta*o);	
        break;
			}
      case Qt::Key_Down:
  		{
				Quaternionf o = mCamera.orientation();
				Quaternionf delta;
				delta = AngleAxisf(-camAngleSpeed, mCamera.right());
				mCamera.setOrientation(delta*o);	
        break;
			}
      case Qt::Key_Left:
  		{
				Quaternionf o = mCamera.orientation();
				Quaternionf delta;
				delta = AngleAxisf(camAngleSpeed, Vector3f::UnitZ());
				mCamera.setOrientation(delta*o);	
        break;
			}
      case Qt::Key_Right:
   		{
				Quaternionf o = mCamera.orientation();
				Quaternionf delta;
				delta = AngleAxisf(-camAngleSpeed,  Vector3f::UnitZ());
				mCamera.setOrientation(delta*o);	
        break;
			}
      // add a frame
      case Qt::Key_G:
        grabFrame();
        break;
      // clear the time line
      case Qt::Key_C:
        m_timeline.clear();
        break;
      // move the camera to initial pos
      case Qt::Key_R:
        resetCamera();
        break;
      //  arrows to flight with camera
      case Qt::Key_W:
			{
				Vector3f camPos = mCamera.position();
				Vector3f camDir = mCamera.direction();
				camDir.normalize();
				camPos += camSpeed * camDir;
				mCamera.setPosition(camPos);
        break;
			}
      case Qt::Key_S:
			{
  			Vector3f camPos = mCamera.position();
				Vector3f camDir = mCamera.direction();
				camDir.normalize();
				camPos -= camSpeed * camDir;
				mCamera.setPosition(camPos);
        break;
			}
      case Qt::Key_A:
			{
				Vector3f camPos = mCamera.position(); 
				camPos -= camSpeed * mCamera.right();
				mCamera.setPosition(camPos);
        break;
			}
      case Qt::Key_D:
 			{
				Vector3f camPos = mCamera.position(); 
				camPos += camSpeed * mCamera.right();
				mCamera.setPosition(camPos);
        break;
			}
      case Qt::Key_P:
        m_isSolverStopped = !m_isSolverStopped; 
        break;
      case Qt::Key_N:
        m_solverTimeFlow = SolverForwardTime;
        m_performPauseStep = true;
        break;
      case Qt::Key_B:
        m_solverTimeFlow = SolverBackwardTime;
        m_performPauseStep = true;
        break;
			case Qt::Key_Space:
			{
				static int sIndex = 100;
				IPhysEnt* s2 = new Sphere();
				s2->m_pos = Vector3f(rand()%10 - 5, rand()%10 -5, 1.3f);
				s2->m_id = sIndex++;
				s2->m_minv = 1;
				//s2->m_v = Vector3f(2.f, 0.f, 0.f);
				m_core.get()->m_objects.push_back(s2);
				s2->m_forces.push_back(g_Gravity);
        break;
			}
      default:
        break;
    }

    updateGL();
}

void RenderingWidget::mousePressEvent(QMouseEvent* e)
{
	m_lastMousePos = Vector2i(e->pos().x(), e->pos().y());
	m_lastMousePosTime = getCurrTime();	
	if (e->modifiers() & Qt::ControlModifier)
	{
		bool fly = (mNavMode==NavFly) || (e->modifiers()&Qt::ControlModifier);
		switch(e->button())
		{
			case Qt::LeftButton:
				if(fly)
				{
					mCurrentTrackingMode = TM_LOCAL_ROTATE;
					mTrackball.start(Trackball::Local);
				}
				else
				{
					mCurrentTrackingMode = TM_ROTATE_AROUND;
					mTrackball.start(Trackball::Around);
				}
				mTrackball.track(m_lastMousePos);
				break;
			case Qt::MidButton:
				if(fly)
					mCurrentTrackingMode = TM_FLY_Z;
				else
					mCurrentTrackingMode = TM_ZOOM;
				break;
			case Qt::RightButton:
					mCurrentTrackingMode = TM_FLY_PAN;
				break;
			default:
				break;
		}
	}
	else
	{
		//do RWI test, to find pointed object
		SRay r;
		r.m_org = mCamera.position();
		r.m_dist = 1000.0f;
		r.m_dir = mCamera.directionFromScreen(m_lastMousePos);
		SRayHit res;
		
		//qDebug() << "direction:" << r.m_dir;
		bool isHelperHit = m_objMover.RWI(r);
		if (!isHelperHit)
		{
			if (m_core->RWI(r, res))
			{
				m_pSelectedEnt = res.m_pEnt;	
	//			m_pSelectedEnt->m_v= Vector3f(0,0,0);
				m_pSelectedEnt->m_active = true; 
				m_pSelectedEnt->m_forces.clear();
				qDebug() << "Picked object:" << res.m_pEnt->m_id;
				m_objMover.OnSelect(m_pSelectedEnt);
			}
			else	
			{
				m_objMover.OnDeselect(0);
				m_pSelectedEnt = 0; 
			}
		}
	}
}

void RenderingWidget::mouseReleaseEvent(QMouseEvent*)
{
    mCurrentTrackingMode = TM_NO_TRACK;
    updateGL();
		m_lastMousePosTime = 0.0f;
		if (m_pSelectedEnt)
		{
			//m_pSelectedEnt->m_forces.push_back(g_Gravity);//TODO:restore all forces
			m_pSelectedEnt = 0;
		}	
}

void RenderingWidget::mouseMoveEvent(QMouseEvent* e)
{
    // tracking
    if(mCurrentTrackingMode != TM_NO_TRACK)
    {
        float dx =   float(e->x() - m_lastMousePos.x()) / float(mCamera.vpWidth());
        float dy = - float(e->y() - m_lastMousePos.y()) / float(mCamera.vpHeight());

        // speedup the transformations
        if(e->modifiers() & Qt::ShiftModifier)
        {
          dx *= 10.;
          dy *= 10.;
        }

        switch(mCurrentTrackingMode)
        {
          case TM_ROTATE_AROUND:
          case TM_LOCAL_ROTATE:
            if (mRotationMode==RotationStable)
            {
              // use the stable trackball implementation mapping
              // the 2D coordinates to 3D points on a sphere.
              mTrackball.track(Vector2i(e->pos().x(), e->pos().y()));
            }
            else
            {
              // standard approach mapping the x and y displacements as rotations
              // around the camera's X and Y axes.
              Quaternionf q = AngleAxisf( dx*M_PI, Vector3f::UnitY())
                            * AngleAxisf(-dy*M_PI, Vector3f::UnitX());
              if (mCurrentTrackingMode==TM_LOCAL_ROTATE)
                mCamera.localRotate(q);
              else
                mCamera.rotateAroundTarget(q);
            }
            break;
          case TM_ZOOM :
            mCamera.zoom(dy*100);
            break;
          case TM_FLY_Z :
            mCamera.localTranslate(Vector3f(0, 0, -dy*200));
            break;
          case TM_FLY_PAN :
            mCamera.localTranslate(Vector3f(dx*200, dy*200, 0));
            break;
          default:
            break;
        }

        updateGL();
    }
		else
		{
        float dx =   float(e->x() - m_lastMousePos.x()); 
        float dy = - float(e->y() - m_lastMousePos.y());
				
				m_objMover.OnMouseMove(Vector3f(e->x(),e->y(), 0));
				if (m_pSelectedEnt && m_pSelectedEnt->m_minv > 0)	
				{
					Vector4f pickedWrld(m_pSelectedEnt->m_pos[0],m_pSelectedEnt->m_pos[1],m_pSelectedEnt->m_pos[2],1);
					
					GLfloat tmp[16];
					glGetFloatv(GL_MODELVIEW_MATRIX, tmp);
					Eigen::Map<Eigen::Matrix4f> mv_m(tmp);	
					//std::cout << "Model view" << std::endl << mv_m << std::endl;

					GLfloat tmp2[16];
					glGetFloatv(GL_PROJECTION_MATRIX, tmp2);
					Eigen::Map<Eigen::Matrix4f> proj_m(tmp2);	
					//std::cout << "Projection" << std::endl << proj_m << std::endl;
				
					Vector4f pickedClip = proj_m * mv_m * pickedWrld;
					float pickedNormDepth = pickedClip[2] / pickedClip[3];	

					//TODO: provide getter for window width, height
					Vector4f newMouseNorm(e->x()*2/800.0f - 1.f, -1.f* (e->y()*2/600.0f - 1.f), pickedNormDepth, 1);
					
					Vector4f newMouseWrld = (proj_m * mv_m).inverse() * newMouseNorm;
					{
						Vector3f offsetWrld(newMouseWrld[0] / newMouseWrld[3]  - pickedWrld[0],newMouseWrld[1] / newMouseWrld[3]  - pickedWrld[1],newMouseWrld[2] / newMouseWrld[3]  - pickedWrld[2]);

						float time = getCurrTime() - m_lastMousePosTime;
						if (time < 0.01)
							time = 0.01;
						assert(time > 0);

						Vector3f speed = offsetWrld / time;	
						m_pSelectedEnt->m_v = speed;
						/*qDebug() << "move:" << pickedWrld << "->" << newMouseWrld << "dist: " << offsetWrld
						<< "t: " << time 
						<< "spd: " << speed;
						*/
					}
				}
		}

    m_lastMousePos = Vector2i(e->pos().x(), e->pos().y());
		m_lastMousePosTime = getCurrTime();	
}

void RenderingWidget::wheelEvent(QWheelEvent * event)
{
		//qDebug() << "wheel " << event->delta() << " " << event->orientation();
		if (!m_pSelectedEnt || m_pSelectedEnt->m_minv == 0)
			return;	
		
		Vector3f camDir = mCamera.direction();
		m_pSelectedEnt->m_v = 0.5f * (event->delta() > 0 ? 1 : -1) * camDir;
}

void RenderingWidget::paintGL()
{
  glEnable(GL_DEPTH_TEST);
 // glDisable(GL_CULL_FACE);
 // glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
  //glDisable(GL_COLOR_MATERIAL);
  //glDisable(GL_BLEND);
 // glDisable(GL_ALPHA_TEST);
 // glDisable(GL_TEXTURE_1D);
 // glDisable(GL_TEXTURE_2D);
  //glDisable(GL_TEXTURE_3D);

  // Clear buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  mCamera.activateGL();


  drawScene();

	//Vector3f a = PYRFromQuat(dir);
	//EulerAngles<float> ea(dir);
	//qDebug() << dir << " ==  " << quatFromPYR(a.x(), a.y(), a.z());
	//qDebug() << dir << " ==  " << (Quaternionf)ea;
}

void RenderingWidget::initializeGL()
{
  glClearColor(0.2, 0.2, 0.2, 0.);
  mCamera.setPosition(Vector3f(2.23f, 1.88f, 1.51f));
	mCamera.setOrientation(Quaternionf(-0.354753,-0.248882, -0.471565, -0.768007));

  mInitFrame.orientation = mCamera.orientation().inverse();
  mInitFrame.position = mCamera.viewMatrix().translation();
}

void RenderingWidget::resizeGL(int width, int height)
{
    mCamera.setViewport(width,height);
}

void RenderingWidget::setNavMode(int m)
{
  mNavMode = NavMode(m);
}

void RenderingWidget::setLerpMode(int m)
{
  mLerpMode = LerpMode(m);
}

void RenderingWidget::setStepMode(int m)
{
  m_SolverStepMode = SolverStepMode(m);
}

void RenderingWidget::setRotationMode(int m)
{
  mRotationMode = RotationMode(m);
}

void RenderingWidget::resetCamera()
{
  m_timeline.clear();
  Frame aux0 = mCamera.frame();
  aux0.orientation = aux0.orientation.inverse();
  aux0.position = mCamera.viewMatrix().translation();
  m_timeline[0] = aux0;

  Vector3f currentTarget = mCamera.target();
  mCamera.setTarget(Vector3f::Zero());

  // compute the rotation duration to move the camera to the target
  Frame aux1 = mCamera.frame();
  aux1.orientation = aux1.orientation.inverse();
  aux1.position = mCamera.viewMatrix().translation();
  float duration = aux0.orientation.angularDistance(aux1.orientation) * 0.9;
  if (duration<0.1) duration = 0.1;

  // put the camera at that time step:
  aux1 = aux0.lerp(duration/2,mInitFrame);
  // and make it look at the target again
  aux1.orientation = aux1.orientation.inverse();
  aux1.position = - (aux1.orientation * aux1.position);
  mCamera.setFrame(aux1);
  mCamera.setTarget(Vector3f::Zero());

  // add this camera keyframe
  aux1.orientation = aux1.orientation.inverse();
  aux1.position = mCamera.viewMatrix().translation();
  m_timeline[duration] = aux1;

  m_timeline[2] = mInitFrame;
  m_alpha = 0;
}

QWidget* RenderingWidget::createNavigationControlWidget()
{
  QWidget* panel = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout();

  {
    QPushButton* but = new QPushButton("reset");
    but->setToolTip("move the camera to initial position (with animation)");
    layout->addWidget(but);
    connect(but, SIGNAL(clicked()), this, SLOT(resetCamera()));
  }
  {
    // navigation mode
    QGroupBox* box = new QGroupBox("navigation mode");
    QVBoxLayout* boxLayout = new QVBoxLayout;
    QButtonGroup* group = new QButtonGroup(panel);
    QRadioButton* but;
    but = new QRadioButton("turn around");
    but->setToolTip("look around an object");
    group->addButton(but, NavTurnAround);
    boxLayout->addWidget(but);
    but = new QRadioButton("fly");
    but->setToolTip("free navigation like a spaceship\n(this mode can also be enabled pressing the \"shift\" key)");
    group->addButton(but, NavFly);
    boxLayout->addWidget(but);
    group->button(mNavMode)->setChecked(true);
    connect(group, SIGNAL(buttonClicked(int)), this, SLOT(setNavMode(int)));
    box->setLayout(boxLayout);
    layout->addWidget(box);
  }
  {
    // track ball, rotation mode
    QGroupBox* box = new QGroupBox("rotation mode");
    QVBoxLayout* boxLayout = new QVBoxLayout;
    QButtonGroup* group = new QButtonGroup(panel);
    QRadioButton* but;
    but = new QRadioButton("stable trackball");
    group->addButton(but, RotationStable);
    boxLayout->addWidget(but);
    but->setToolTip("use the stable trackball implementation mapping\nthe 2D coordinates to 3D points on a sphere");
    but = new QRadioButton("standard rotation");
    group->addButton(but, RotationStandard);
    boxLayout->addWidget(but);
    group->button(mRotationMode)->setChecked(true);
    connect(group, SIGNAL(buttonClicked(int)), this, SLOT(setRotationMode(int)));
    box->setLayout(boxLayout);
    layout->addWidget(box);
  }
  {
    // interpolation mode
    QGroupBox* box = new QGroupBox("spherical interpolation");
    QVBoxLayout* boxLayout = new QVBoxLayout;
    QButtonGroup* group = new QButtonGroup(panel);
    QRadioButton* but;
    but = new QRadioButton("quaternion slerp");
    group->addButton(but, LerpQuaternion);
    boxLayout->addWidget(but);
    but->setToolTip("use quaternion spherical interpolation\nto interpolate orientations");
    but = new QRadioButton("euler angles");
    group->addButton(but, LerpEulerAngles);
    boxLayout->addWidget(but);
    but->setToolTip("use Euler angles to interpolate orientations");
    group->button(mNavMode)->setChecked(true);
    connect(group, SIGNAL(buttonClicked(int)), this, SLOT(setLerpMode(int)));
    box->setLayout(boxLayout);
    layout->addWidget(box);
  }
  {
    // step mode
    QGroupBox* box = new QGroupBox("solver step mode");
    QVBoxLayout* boxLayout = new QVBoxLayout;
    QButtonGroup* group = new QButtonGroup(panel);
    QRadioButton* but;
    but = new QRadioButton("continuous");
    group->addButton(but, SolverContinuous);
    boxLayout->addWidget(but);
    but->setToolTip("continous simulation in time");
    but = new QRadioButton("discrete");
    group->addButton(but, SolverStepDiscrete);
    boxLayout->addWidget(but);
    but->setToolTip("press 'B' and 'N' buttons to choose simulation direction");
    group->button(mNavMode)->setChecked(true);
    connect(group, SIGNAL(buttonClicked(int)), this, SLOT(setStepMode(int)));
    box->setLayout(boxLayout);
    layout->addWidget(box);
  }


  layout->addItem(new QSpacerItem(0,0,QSizePolicy::Minimum,QSizePolicy::Expanding));
  panel->setLayout(layout);
  return panel;
}

QuaternionDemo::QuaternionDemo()
{
  mRenderingWidget = new RenderingWidget();

  //set phys initial world
  IPhysEnt* s1 = new Sphere();
  s1->m_pos = Vector3f(1.f, 1.f, 1.0f);
  //s1->m_v = Vector3f(-10.f, 0.f, 0.f);
  s1->m_id = 1;
  s1->m_minv = 1;
	//s1->AddImpulse(Vector3f(-1.f, 0.f, 0.f) * 200.f /*Vector3f(10,10,10)*/);
	//s1->m_forces.push_back(g_Gravity);
	 
  //mRenderingWidget->m_core.get()->m_objects.push_back(s1);

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

  IPhysEnt* s3 = new Box();
  s3->m_pos = Vector3f(0.f, 0.f, 0.0f);
  s3->m_id = 3;
  s3->m_minv = 0.1f;
//  s3->m_v = Vector3f(1.0f, 0.f, 0.f);

	mRenderingWidget->m_core.get()->m_objects.push_back(s3);
  //s3->AddImpulse(Vector3f(10.f, 0.f, 0.f) * 100.f, Vector3f(10,10,0));
	

  IPhysEnt* s4 = new Box();
  s4->m_pos = Vector3f(1.f, 0.f, 0.0f);
  s4->m_id = 4;
  s4->m_minv = 10.f;
	s4->m_rot = Quaternionf(0.8923991008325228,0.0990457605412876,-0.36964381061438606,0.2391176183943345); 
	//s4->m_rot = Quaternionf(0.9238795325112867,0, -0.3826834323650897,0); 
 	//s4->m_rot = Quaternionf(0.7071067811865476 ,0, -0.7071067811865476,0); 
	s4->m_rot.normalize();
	//mRenderingWidget->m_core.get()->m_objects.push_back(s4);

  setCentralWidget(mRenderingWidget);
/*
  QDockWidget* panel = new QDockWidget("navigation", this);
  panel->setAllowedAreas((QFlags<Qt::DockWidgetArea>)(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea));
  addDockWidget(Qt::RightDockWidgetArea, panel);
  panel->setWidget(mRenderingWidget->createNavigationControlWidget());
*/
}

void myMessageOutput(QtMsgType type, const char *msg)
{
	 //in this function, you can write the message to any stream!
	 switch (type) {
	 case QtDebugMsg:
			 fprintf(stderr, "[Debug]: %s\n", msg);
			 break;
	 case QtWarningMsg:
			 fprintf(stderr, "[Warning]: %s\n", msg);
			 break;
	 case QtCriticalMsg:
			 fprintf(stderr, "[Critical]: %s\n", msg);
			 break;
	 case QtFatalMsg:
			 fprintf(stderr, "[Fatal]: %s\n", msg);
			 abort();
	 }
}

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
	
	qInstallMsgHandler(myMessageOutput);	

	srand(time(NULL));
  QApplication app(argc, argv);
  QuaternionDemo demo;
  demo.resize(800, 600);
	demo.move(700, 100);
	demo.show();
	glutInit(&argc, argv); 
	 return app.exec();
}

#include "quaternion_demo.moc"

