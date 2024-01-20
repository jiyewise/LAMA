#ifndef __RENDER_INTERACTION_GEN_WIDGET_H__
#define __RENDER_INTERACTION_GEN_WIDGET_H__
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
// #include "Transform.h"
#include "Environment.h"
#include "InteractionCue.h"
#pragma pop_macro("slots")

namespace p = boost::python;
namespace np = boost::python::numpy;

class InteractionGenWidget : public QOpenGLWidget
{
	Q_OBJECT

public:
	InteractionGenWidget();
	InteractionGenWidget(std::string _env_path);
	
	void updatePointList();
	void loadPosMap(); // pos map for generating interaction cue
	void setRootTransform(double _angleDelta, double _heightDelta);
	void setFoot();
	std::vector<std::string> getJointNames();
	void createInteractionContact(std::vector<int> _jointClicked, int _frame);
	ENV::InteractionCue createInteractionCue();
	void addInteractionCue(ENV::InteractionCue _ic) { mInteractionCueList.push_back(_ic); }
	void rotateBody(int _idx, bool _increase);
	void translateBody(int _idx, bool _increase);
	void moveSelectedPoint();
	bool isAction(std::string _curMode);
	void viewInputAndIC(bool _all);
	void modifyIC();
	void exportInput(std::string _fileName);

	ENV::Environment* mEnv;
	ENV::Scene* mEnvScene;
	std::vector<ENV::InteractionCue> mInteractionCueList;
	std::vector<ENV::Contact> mContactList;
	std::vector<Eigen::Vector3d> mRay; // should be size 2: point and dir
	std::vector<ENV::IntersectPoint> mIntersectPointList; 
	std::map<std::string, Eigen::VectorXd> mPosMap; // initial
	std::map<std::string, Eigen::VectorXd> mEditedPosMap; 
	// for ic visualization
	dart::dynamics::SkeletonPtr mInputSkel; 

	Eigen::Vector3d mIntersectPointMovedPos;
	Eigen::Vector3d mSceneOri;
	std::string mCurMode;

	double mOriDelta, mHeightDelta;
	double mUnit;
	int mSelectPointIdx, mSelectCueIdx;
	bool mDrawSkel;
	bool mDrawAllCues;
	
	void togglePlay();

public slots:
	std::vector<ENV::IntersectPoint> getIntersectPoints();
	void setSelectedPoint(int _selectIdx);
	void setSelectedCue(int _selectIdx);
	// void nextFrame();
	// void prevFrame();
	// void reset();

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

	void computeMouseRay(int _x, int _y);

	void drawMouseRay();
	void drawIntersectPoint();
	void drawInteractionCue(ENV::InteractionCue _ic);
	void drawInteractionCues();

	void drawGround();
	void drawSkeletons();
	void setFrame(std::string _type);

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
	dart::dynamics::SkeletonPtr mSkelBVH;

	SIM::ReferenceManager* mReferenceManager;
};
#endif
