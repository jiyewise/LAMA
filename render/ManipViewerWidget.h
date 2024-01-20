#ifndef __RENDER_MANIP_VIEWER_WIDGET_H__
#define __RENDER_MANIP_VIEWER_WIDGET_H__
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
#include "ManipMotionEditor.h"
#pragma pop_macro("slots")

class ManipViewerWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	ManipViewerWidget();
	ManipViewerWidget(std::string _env, std::string _manipIC, std::string _dir);
	void togglePlay();

	ENV::Environment* mEnv;
	ENV::Scene* mEnvScene; 

	std::vector<Eigen::Vector3d> mManipContactPos;
	std::vector<std::pair<int, Eigen::VectorXd>> mMotionManipObject;

	// to be sent to autoencoder
	std::vector<ENV::Contact> mManip2ContactList; 
	ENV::ManipMotionEditor* mManipMotionEditor;

	void getManipInfo();
	void readMotion(std::string _dir);
	void drawContactTrajectory();

	std::vector<Eigen::VectorXd> mMotionEdited;
	std::vector<Eigen::VectorXd> mMotionRecord;

	bool mIsRecord;
	bool mIsEditMode;
	std::string mDir;
	// bool mDrawRef;
	dart::dynamics::SkeletonPtr mSkelEdit;
	dart::dynamics::SkeletonPtr mSkelRecord;

	void optimize(int _start, int _end);
	void optimizeIK(int _start, int _end);
	void saveMotion(std::string _file, int _start, int _end);

	std::map<std::string, Eigen::Vector4d> mColorMap;

	void setSnapshotInfo();
	void drawSnapshot();
	std::vector<dart::dynamics::SkeletonPtr> mSkelList;
	std::vector<dart::dynamics::SkeletonPtr> mManipObjList;
	std::vector<Eigen::Vector4d> mColorGrad;
	int mSkelNum;
	int mObjNum;
	int mSnapshotNum;
	bool mIsSnapshot;
	bool mDrawFrame;

	// void saveOriginalMotion(std::string _filename);
	// void saveTransitionRecord(std::string _filename);
	// void saveOptimizeMotion(std::string _filename);

	bool mDrawSceneMeshColor;
	std::vector<Eigen::VectorXd> mMotionManipRecord;


	// capture
	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
	void Screenshot();
	bool mIsCapture;
	std::string mSaveDir;

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