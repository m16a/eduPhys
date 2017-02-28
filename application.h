// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_QUATERNION_DEMO_H
#define EIGEN_QUATERNION_DEMO_H

#ifdef WIN32
#include <windows.h>
#endif
#include <map>
#include <QTimer>
#include <QTime>
#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include <QtGui/QMainWindow>
#include <memory>

#include "gpuhelper.h"
#include "camera.h"
#include "obj_mover.h"
#include "sphere.h"
#include "core.h"
#include "box.h"

class RenderingWidget : public QGLWidget
{
  Q_OBJECT

    enum SolverFlow{
      SolverForwardTime,
      SolverBackwardTime
    };

  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    RenderingWidget();
    ~RenderingWidget() { }

		Core* getCore() {return m_core.get();};
    std::auto_ptr<Core> m_core;
	
		ObjMover m_objMover;	
		IPhysEnt* m_pSelectedEnt;
		
    float m_lastTime; //seconds
		float m_realTime;
		float m_physTime;
		int m_frameNumber;


  protected slots:

    virtual void drawScene(void);

  protected:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();
    
    //--------------------------------------------------------------------------------
    virtual void mousePressEvent(QMouseEvent * e);
    virtual void mouseReleaseEvent(QMouseEvent * e);
    virtual void mouseMoveEvent(QMouseEvent * e);
    virtual void keyPressEvent(QKeyEvent * e);
		virtual void wheelEvent(QWheelEvent * event);
    //--------------------------------------------------------------------------------

	private:
		void drawDebugInfo(float dt, float physSimTime);
		void updateCore(float dt);

	private:
    Camera mCamera;
    SolverFlow m_solverTimeFlow;
    Vector2i m_lastMousePos;
		float m_lastMousePosTime;
    bool m_performPauseStep;

    QTimer m_timer;
    bool m_isSolverStopped;
    void setupCamera();
};

class CApplication : public QMainWindow
{
  Q_OBJECT
  public:
    CApplication ();
		Core* getCore() {return mRenderingWidget->getCore();};
  protected:
    RenderingWidget* mRenderingWidget;
};

#endif // EIGEN_QUATERNION_DEMO_H
