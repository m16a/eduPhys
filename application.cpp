// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "application.h"

#include <stdlib.h>
#include <iostream>
#include <unistd.h>

#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/LU>

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

#include "rwi.h"
#include "phys_ent.h"
#include "my_utils.h"
#include "icosphere.h"
#include "debug_draw.h"
#include "my_eulerAngles.h"

using namespace Eigen;

// generic linear interpolation method
template<typename T> T lerp(float t, const T& a, const T& b)
{
  return a*(1-t) + b*t;
}

// quaternion slerp
template<> Quaternionf lerp(float t, const Quaternionf& a, const Quaternionf& b)
{ return a.slerp(t,b); }

// Euler angles slerp
template<> EulerAngles<float> lerp(float t, const EulerAngles<float>& a, const EulerAngles<float>& b)
{
  EulerAngles<float> res;
  res.coeffs() = lerp(t, a.coeffs(), b.coeffs());
  return res;
}

RenderingWidget::RenderingWidget()
{
  m_core.reset(new Core);

  m_performPauseStep = false;
  m_isSolverStopped = true;
  m_solverTimeFlow = SolverForwardTime;
	clock_gettime(CLOCK_REALTIME, &m_lastTime);
	m_realTime = 0.0f;
	m_physTime = 0.0f;
	
	m_pSelectedEnt = 0;
  // required to capture key press events
  setFocusPolicy(Qt::StrongFocus);
	m_frameNumber = 0;
	m_cameraMoveFlags = 0;
}

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

float timespecToFloat(const timespec& ts)
{
	return ts.tv_sec + ts.tv_nsec / 1000000000.0f;
}

void RenderingWidget::drawScene()
{
	timespec currTime;
	clock_gettime(CLOCK_REALTIME, &currTime);
  const float dt = timespecToFloat(diff(m_lastTime, currTime));
	m_lastTime = currTime;
	//qDebug() << "rT " << time1.tv_sec << ":" << time1.tv_nsec;
	//qDebug() << "dt " << time1.tv_sec << ":" << time1.tv_nsec;
	//m_lastTime = currTime;  

  const float length = 0.2;
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitX(), Color(1,0,0,1));
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitY(), Color(0,1,0,1));
  gpu.drawVector(Vector3f::Zero(), length*Vector3f::UnitZ(), Color(0,0,1,1));

  glLightfv(GL_LIGHT0, GL_AMBIENT, Vector4f(1.,1,1,1).data());
	glLightfv(GL_LIGHT0, GL_DIFFUSE, Vector4f(0.5,1,0.5,1).data());
  glLightfv(GL_LIGHT0, GL_SPECULAR, Vector4f(1,1,1,1).data());
  glLightfv(GL_LIGHT0, GL_POSITION, Vector4f(0,0,2.8,0.5).data());

  glLightfv(GL_LIGHT1, GL_AMBIENT, Vector4f(0.5,0.5,0.5,1).data());
  glLightfv(GL_LIGHT1, GL_DIFFUSE, Vector4f(0.6,0.6,0.6,0.6).data());
  glLightfv(GL_LIGHT1, GL_SPECULAR, Vector4f(1,0,0,0.5).data());
  glLightfv(GL_LIGHT1, GL_POSITION, Vector4f(2.8,0,0,0).data());

	/*
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, Vector4f(0.7, 0.7, 0.7, 1).data());
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Vector4f(0.8, 0.75, 0.6, 1).data());
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, Vector4f(1, 1, 1, 1).data());
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64);
	*/
  //glEnable(GL_LIGHTING);
  //glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	updateCameraPosDir(dt);

  m_core.get()->Draw();
	m_objMover.Update();	
	DebugManager()->Draw(m_isSolverStopped);	

	glDisable(GL_LIGHTING);
	updateCore(dt);
  update();
}

void RenderingWidget::updateCameraPosDir(float dt)
{
	static float lastUpdate = 0.0;
	if (lastUpdate < 0.03)
	{
		lastUpdate += dt;
		return;
	}
	else 
		lastUpdate = dt;

	const float camSpeed = 0.25f;
	const float camAngleSpeed = 3.0f * M_PI / 180.0f;

	//rotate camera
	{
		Quaternionf delta(Quaternionf::Identity());
		float tmp = 0.0f; 
		if (m_cameraMoveFlags & eCMM_PitchUp)
			tmp += camAngleSpeed;

		if (m_cameraMoveFlags & eCMM_PitchDown)
			tmp -= camAngleSpeed;

		if (tmp != 0.0f)
		{
			delta = AngleAxisf(tmp, mCamera.right());
			Quaternionf o = mCamera.orientation();
			mCamera.setOrientation(delta*o);	
		}
	}

	{
		Quaternionf delta(Quaternionf::Identity());
		float tmp = 0.0f; 
		if (m_cameraMoveFlags & eCMM_YawLeft)
			tmp += camAngleSpeed;

		if (m_cameraMoveFlags & eCMM_YawRight)
			tmp -= camAngleSpeed;

		if (tmp != 0.0f)
		{
			delta = AngleAxisf(tmp, Vector3f::UnitZ());
			Quaternionf o = mCamera.orientation();
			mCamera.setOrientation(delta*o);	
		}
	}

	{
		float tmp = 0.0f;
		if (m_cameraMoveFlags & eCMM_Up)
			tmp += camAngleSpeed;

		if (m_cameraMoveFlags & eCMM_Down)
			tmp -= camAngleSpeed;

		Vector3f camPos = mCamera.position();
		Vector3f camDir = mCamera.direction();
		camDir.normalize();
		camPos += tmp * camDir;
		mCamera.setPosition(camPos);
	}

	{
		float tmp = 0.0f;
		if (m_cameraMoveFlags & eCMM_Left)
			tmp += camAngleSpeed;

		if (m_cameraMoveFlags & eCMM_Right)
			tmp -= camAngleSpeed;

		Vector3f camPos = mCamera.position(); 
		camPos -= tmp * mCamera.right();
		mCamera.setPosition(camPos);
	}
}


void RenderingWidget::updateCore(float dt)
{
	const bool fixedStep = true;
	const float reqStep = 0.010f;

	static float physSimTime = 0.0f; 
	static float unperformedStep = 0.0f; 

	//Debug() << "fN:" << m_frameNumber << " sT:" << m_physTime << " rT:" << m_realTime << " dt:" << dt << " unperf" << unperformedStep; 
	if (!m_isSolverStopped || (m_isSolverStopped && m_performPauseStep))
	{
		if (unperformedStep > reqStep)
		{
			const float dir = (m_solverTimeFlow == SolverForwardTime) ? 1.0f : -1.0f;
			const float t = (fixedStep ? reqStep : dt);

			const float stepStartTime = clock() / float(CLOCKS_PER_SEC);
			m_core.get()->Step(dir * t);
			const float stepFinishTime = clock() / float(CLOCKS_PER_SEC);

			m_performPauseStep = false;
			physSimTime = stepFinishTime - stepStartTime ;
			if (fixedStep && physSimTime > reqStep)
				qWarning() << "Can't chase real time. reqStep:" << reqStep << "performedTime:" << physSimTime;
			m_physTime += fixedStep ? reqStep : dt;
			m_frameNumber++;
			unperformedStep -= reqStep;
		}
		unperformedStep += dt;
		m_realTime += dt;
	}
	else
	{
		//don't waste CPU on pause
		usleep(30000);//30ms
	}

	//drawDebugInfo(dt, std::max(physSimTime, reqStep));
	drawDebugInfo(dt, physSimTime);
}

void RenderingWidget::drawDebugInfo(float dt, float physSimTime) 
{
	glColor4f(1.0, 1.0, 1.0, 0.9);
	//Print camera info
	const Vector3f camPos(mCamera.position());
	renderText(10,12, QString("cam pos: %1, %2, %3").arg(QString::number(camPos.x(),'f',2),QString::number(camPos.y(),'f',2),QString::number(camPos.z(),'f',2)));

	const Vector3f d(mCamera.direction());
	renderText(10,32, QString("cam dir: %1, %2, %3").arg(QString::number(d[0],'f',2), QString::number(d[1],'f',2), QString::number(d[2],'f',2)));
	
	Quaternionf quat = mCamera.orientation();
	Vector3f ypr = PYRAnglesFromQuat(quat); 
	renderText(10,52, QString("YPR: %1, %2, %3").arg(QString::number(ypr.x(),'f',2),QString::number(ypr.y(),'f',2),QString::number(ypr.z(),'f',2)));

	//if (m_realTime > 0.01 && dt > 0.0001)
		renderText(10,72, QString("rT:%1, pT:%2, ratio:%3, fps:%4").arg(QString::number(m_realTime,'f',2), QString::number(m_physTime,'f',2), QString::number(m_physTime / m_realTime,'f',2), QString::number(1/physSimTime,'f',1)));

	renderText(10,92, QString("E_kin:%1").arg(QString::number(m_core.get()->CalcKineticEnergy(),'f',2)));

	if (m_pSelectedEnt)
	{
		renderText(500,12, QString("Slctd objct: %1").arg(m_pSelectedEnt->m_id));
		renderText(500,32, QString("v: %1").arg(QString::fromStdString(VecToStr(m_pSelectedEnt->m_v))));
		renderText(500,52, QString("w: %1").arg(QString::fromStdString(VecToStr(m_pSelectedEnt->m_w))));
		
		//draw axis of rotation
		DebugManager()->DrawVector(m_pSelectedEnt->m_pos, m_pSelectedEnt->m_w, 0.5);
		DebugManager()->DrawVector(m_pSelectedEnt->m_pos, -m_pSelectedEnt->m_w, 0.2);
	}
}

float getCurrTime()
{
	return clock() / float(CLOCKS_PER_SEC);
}

void RenderingWidget::keyPressEvent(QKeyEvent* e)
{
		const float camSpeed = 0.25f;
		const float camAngleSpeed = 10.0f * M_PI / 180.0f;
    switch(e->key())
    {
			//rotate camera
      case Qt::Key_Up:
			{
				m_cameraMoveFlags |= eCMM_PitchUp; 
        break;
			}
      case Qt::Key_Down:
  		{
				m_cameraMoveFlags |= eCMM_PitchDown; 
        break;
			}
      case Qt::Key_Left:
  		{
				m_cameraMoveFlags |= eCMM_YawLeft; 
        break;
			}
      case Qt::Key_Right:
   		{
				m_cameraMoveFlags |= eCMM_YawRight; 
        break;
			}
      //  arrows to flight with camera
      case Qt::Key_W:
			{
				m_cameraMoveFlags |= eCMM_Up; 
        break;
			}
      case Qt::Key_S:
			{
				m_cameraMoveFlags |= eCMM_Down; 
        break;
			}
      case Qt::Key_A:
			{
				m_cameraMoveFlags |= eCMM_Left; 
        break;
			}
      case Qt::Key_D:
 			{
				m_cameraMoveFlags |= eCMM_Right; 
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
        //m_solverTimeFlow = SolverBackwardTime;
        //m_performPauseStep = true;
        break;
			case Qt::Key_C:
				{
					m_core.get()->SerializeToFile("dump");
					break;
				}
			case Qt::Key_V:
				{
					m_core.get()->DeserializeFromFile("dump");
					break;
				}
			case Qt::Key_Space:
			{
        break;
			}
      default:
        break;
    }

    updateGL();
}

void RenderingWidget::keyReleaseEvent(QKeyEvent* e)
{
		const float camSpeed = 0.25f;
		const float camAngleSpeed = 10.0f * M_PI / 180.0f;
    switch(e->key())
    {
			//rotate camera
      case Qt::Key_Up:
			{
				m_cameraMoveFlags &= ~eCMM_PitchUp; 
        break;
			}
      case Qt::Key_Down:
  		{
				m_cameraMoveFlags &= ~eCMM_PitchDown; 
        break;
			}
      case Qt::Key_Left:
  		{
				m_cameraMoveFlags &= ~eCMM_YawLeft; 
        break;
			}
      case Qt::Key_Right:
   		{
				m_cameraMoveFlags &= ~eCMM_YawRight; 
        break;
			}
      //  arrows to flight with camera
      case Qt::Key_W:
			{
				m_cameraMoveFlags &= ~eCMM_Up; 
        break;
			}
      case Qt::Key_S:
			{
				m_cameraMoveFlags &= ~eCMM_Down; 
        break;
			}
      case Qt::Key_A:
			{
				m_cameraMoveFlags &= ~eCMM_Left; 
        break;
			}
      case Qt::Key_D:
 			{
				m_cameraMoveFlags &= ~eCMM_Right; 
        break;
			}
      case Qt::Key_P:
        break;
      case Qt::Key_N:
        break;
      case Qt::Key_B:
        break;
			case Qt::Key_C:
				{
					break;
				}
			case Qt::Key_V:
				{
					break;
				}
			case Qt::Key_Space:
			{
				/*
				static int sIndex = 100;
				IPhysEnt* s2 = new Sphere();
				s2->m_pos = Vector3f(rand()%10 - 5, rand()%10 -5, 1.3f);
				s2->m_id = sIndex++;
				s2->m_minv = 1;
				m_core.get()->m_objects.push_back(s2);
				s2->m_forces.push_back(g_Gravity);
				*/
        break;
			}
      default:
        break;
    }

}

void RenderingWidget::mousePressEvent(QMouseEvent* e)
{
	m_lastMousePos = Vector2i(e->pos().x(), e->pos().y());
	m_lastMousePosTime = getCurrTime();	

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

void RenderingWidget::mouseReleaseEvent(QMouseEvent*)
{
    updateGL();
		m_lastMousePosTime = 0.0f;
		if (m_pSelectedEnt)
		{
			//m_pSelectedEnt->m_forces.push_back(g_Gravity);//TODO:restore all forces
			//m_pSelectedEnt = 0;
		}	
}

void RenderingWidget::mouseMoveEvent(QMouseEvent* e)
{
	float dx =   float(e->x() - m_lastMousePos.x()); 
	float dy = - float(e->y() - m_lastMousePos.y());

	SRay r;
	r.m_org = mCamera.position();
	r.m_dist = 1000.0f;
	r.m_dir = mCamera.directionFromScreen(Vector2i(e->pos().x(), e->pos().y()));

	bool wasRotation = m_objMover.OnMouseMove(Vector3f(e->x(),e->y(), 0), r);
	if (!wasRotation && m_pSelectedEnt && m_pSelectedEnt->m_minv > 0)	
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

		Vector4f newMouseNorm(e->x()*2/float(mCamera.vpWidth()) - 1.f, -1.f* (e->y()*2/float(mCamera.vpHeight()) - 1.f), pickedNormDepth, 1);
		
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
}

void RenderingWidget::initializeGL()
{
	glClearColor(0.2, 0.2, 0.2, 0.);
	mCamera.setPosition(Vector3f(0.0f, -2.0f, 1.0f));
	mCamera.setOrientation(quatFromPYRAngles(90.0f, 0.0f, 0.0f));
}

void RenderingWidget::resizeGL(int width, int height)
{
    mCamera.setViewport(width,height);
}

void myMessageOutput(QtMsgType type, const char *msg)
{
	 switch (type) 
	 {
	 case QtDebugMsg:
			 fprintf(stdout, "[Debug]: %s\n", msg);
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

CApplication::CApplication()
{
  mRenderingWidget = new RenderingWidget();
	qInstallMsgHandler(myMessageOutput);	
  setCentralWidget(mRenderingWidget);
}

#include "application.moc"

