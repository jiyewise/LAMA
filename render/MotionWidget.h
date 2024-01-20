#ifndef __RENDER_MOTION_WIDGET_H__
#define __RENDER_MOTION_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>

#pragma push_macro("slots")
#undef slots
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "Camera.h"
#include "GLFunctions.h"
#include "DARTInterface.h"
#include "ReferenceManager.h"
#include "Functions.h"
// #include "Controller.h"
#include "Transform.h"
#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class MotionWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	MotionWidget();
	MotionWidget(std::string _motion, std::string _ppo);
	void togglePlay();

	dart::dynamics::SkeletonPtr mObjSkel;
public slots:
	void nextFrame();
	void prevFrame();
	void reset();

protected:
 	void initNetworkSetting(std::string _ppo);
 	void runPPO();

	void initializeGL() override;	
	void resizeGL(int w,int h) override;
	void paintGL() override;
	void initLights();

	void timerEvent(QTimerEvent* _event);
	void keyPressEvent(QKeyEvent* _event);

	void mousePressEvent(QMouseEvent* _event);
	void mouseMoveEvent(QMouseEvent* _event);
	void mouseReleaseEvent(QMouseEvent* _event);
	void wheelEvent(QWheelEvent* _event);

	void drawGround();
	void drawSkeletons();
	void setFrame(int _n);

	Camera* mCamera;
	int	mPrevX, mPrevY;
	Qt::MouseButton mButton;
	bool mIsDrag;
	bool mTrackCamera;
	
	bool mPlay;
	int mCurFrame;
	int mTotalFrame;

	// bool mRunPPO;

	std::vector<Eigen::VectorXd> mMotionBVH;
	// std::vector<Eigen::VectorXd> mMotionPPO;
	std::vector<double> mTiming;

	dart::dynamics::SkeletonPtr mSkelBVH;
	// dart::dynamics::SkeletonPtr mSkelPPO;

	// p::object mPPO;
	SIM::ReferenceManager* mReferenceManager;
	// SIM::Controller* mController;
};
#endif
