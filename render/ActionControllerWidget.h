#ifndef __RENDER_AC_WIDGET_H__
#define __RENDER_AC_WIDGET_H__
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
#include "ActionController.h"
#pragma pop_macro("slots")

class ActionControllerWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	ActionControllerWidget();
	ActionControllerWidget(std::string _env, std::string _ppo, bool _optimize);
	void togglePlay();

	ENV::Environment* mEnv;
	ENV::Scene* mEnvScene; 
	
	std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> mMotionTargetTrajectory;
	std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> mMotionCurrentTrajectory;

	std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> mTargetTrajectory;
	std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> mCurTrajectory;

	std::vector<Eigen::VectorXd> mMotionEdited;

	bool mIsEditMode;
	bool mDrawRef;
	dart::dynamics::SkeletonPtr mSkelEdit;

	void saveOriginalMotion(std::string _filename);
	void saveTransitionRecord(std::string _filename);
	void saveActionLog(std::string _filename);
	void saveOptimizeMotion(std::string _filename);

	std::vector<Eigen::VectorXd> mActionRecord;

	// variations
	bool mUseVariations;
	// random
    std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;

	// eff experiment: save initial and ic when success
	// print in env.xml form
	// print time
	void saveInputPair(std::string _filePath);
	void printInputPair();
	
	SIM::Pose2d mSuccessInitial;
	std::vector<ENV::InteractionCue> mSuccessTargetICList;
	std::string mSuccessInferenceTime;

	// ablation
	bool mRemoveOffset;

	// capture
	std::vector<unsigned char> mScreenshotTemp;
	std::vector<unsigned char> mScreenshotTemp2;
	void Screenshot();
	bool mIsCapture;
	std::string mSaveDir;
	std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>> mCameraList;

	bool mDrawSceneMeshColor;
	bool mDrawActionCue;

public slots:
	void nextFrame();
	void prevFrame();
	void reset();
	void optimize();

protected:
 	void initNetworkSetting(std::string _env, std::string _ppo);
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

	bool mRunPPO;

	std::vector<Eigen::VectorXd> mMotionBVH;
	std::vector<Eigen::VectorXd> mMotionPPO;
	std::vector<double> mTiming;

	dart::dynamics::SkeletonPtr mSkelBVH;
	dart::dynamics::SkeletonPtr mSkelPPO;

	p::object mPPO;
	ENV::ActionController* mController;
};
#endif