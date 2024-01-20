#ifndef __RENDER_POSTPROCESS_RESULT_WIDGET_H__
#define __RENDER_POSTPROCESS_RESULT_WIDGET_H__
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
#include "Transform.h"
#include "Environment.h"
// #include "MotionEvaluator.h"

#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class PostProcessRecordWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	PostProcessRecordWidget();
	PostProcessRecordWidget(std::string _dir, bool _optimize);
	void togglePlay();

    ENV::Environment* mEnv;
    ENV::Scene* mEnvScene;
	
    void drawContact();
	void calcFootSlipLoss();
	void saveStartSubTarget();

    std::vector<std::vector<bool>> mContacts; 
    std::map<std::string, int> mLeftFoot, mRightFoot;
	std::vector<std::vector<double>> mFootSlip;

	// penetration
	std::vector<std::pair<int,int>> mPenetrationPointCount;
	std::vector<Eigen::Vector3d> computePenetration(std::string _key, double _heightLimit);
	void calcPenetration();

	// parsing from directory
    void parse(std::string _dir);
	std::string mDir;

	// draw root grid
	void drawRootOccupancy();
	
	std::vector<Eigen::VectorXd> mMotionBVH;
	dart::dynamics::SkeletonPtr mSkelBVH;
	dart::dynamics::SkeletonPtr mSkelOptimized;

    std::vector<Eigen::VectorXd> mMotionOriginal;
    std::vector<Eigen::VectorXd> mMotionOptimized;
    std::vector<Eigen::VectorXd> mMotionPostProcessed;

    std::string mCurMode;

	// capture
	void Screenshot();
	std::string mSaveDir;
	std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>> mCameraList;
	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
	bool mIsCapture;
	bool mAutoCapture;

	// compare motion editing
	bool mShowOriginal;

public slots:
	void nextFrame();
	void prevFrame();
	void reset();

protected:
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

	SIM::ReferenceManager* mReferenceManager;

};
#endif