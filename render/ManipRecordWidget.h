#ifndef __RENDER_MANIP_RECORD_WIDGET_H__
#define __RENDER_MANIP_RECORD_WIDGET_H__
#include <vector>
#include <QOpenGLWidget>
#include <QTimerEvent>
#include <QKeyEvent>

#pragma push_macro("slots")
#undef slots
#include "Functions.h"

#include "Camera.h"
#include "GLFunctions.h"
// #include "RenderConfigParser.h"
#include "DARTInterface.h"
#include "Environment.h"
#pragma pop_macro("slots")

class ManipRecordWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	ManipRecordWidget();
	ManipRecordWidget(std::string _dir);
	void togglePlay();

	ENV::Environment* mEnv;
	ENV::Scene* mEnvScene; 
	
	std::vector<Eigen::VectorXd> mMotionRecord; 
	std::vector<Eigen::VectorXd> mManipObjectRecord;
	std::vector<Eigen::VectorXd> mOriginalMotionRecord;

	void parse(std::string _dir);
	std::string mDir;
	dart::dynamics::SkeletonPtr mSkelRecord;
	dart::dynamics::SkeletonPtr mSkelManipObject;
	

	// for snapshot
	std::map<std::string, Eigen::Vector4d> mColorMap;
	void setSnapshotInfo();
	std::vector<dart::dynamics::SkeletonPtr> mSkelList;
	std::vector<dart::dynamics::SkeletonPtr> mManipObjList;
	std::vector<Eigen::Vector4d> mColorGrad;
	int mSkelNum;
	int mObjNum;
	int mSnapshotNum;
	bool mIsSnapshot;
	bool mDrawFrame;

	// draw ic
	void getManipInfo();
	void drawContactTrajectory();
	// void drawIC();
	std::vector<Eigen::Vector3d> mManipContactPos;

	// capture
	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
	void Screenshot();
	bool mIsCapture;
	std::string mSaveDir;

	bool mDrawSceneMeshColor;

	bool mShowOriginal;

	std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>> mCameraList;

public slots:
	void nextFrame();
	void prevFrame();
	void reset();

protected:
 	// void initNetworkSetting(std::string _env, std::string _ppo);
 	// void runPPO();

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

	// std::vector<Eigen::VectorXd> mMotionBVH;
	// std::vector<Eigen::VectorXd> mMotionPPO;
	// std::vector<double> mTiming;

	// dart::dynamics::SkeletonPtr mSkelBVH;
	// dart::dynamics::SkeletonPtr mSkelPPO;

	// p::object mPPO;
	// ENV::ActionController* mController;
};
#endif