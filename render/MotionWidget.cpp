#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "MotionWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
MotionWidget::
MotionWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false)
{
	startTimer(30);
}
MotionWidget::
MotionWidget(std::string _motion, std::string _ppo)
  :MotionWidget()
{
	mCurFrame = 0;
	mTotalFrame = 0;

	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
	// std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + std::string("skel_cmu") + std::string(".xml");
	std::string motionPath = std::string(LAMA_DIR) + std::string("/data/motion/") + _motion;

	if(!boost::filesystem::exists(skelPath)) {
		SIM::SkeletonBuilder::generateNewSkeleton(motionPath, skelPath);
	}
    mSkelBVH = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
    // mSkelPPO = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
  
    mReferenceManager = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(skelPath).first);
    mReferenceManager->loadMotionFromBVH(motionPath);
   
    mMotionBVH.clear();
    for(int i = 0; i < TERMINAL_ITERATION * mReferenceManager->getMotionLength(); i++) {
        Eigen::VectorXd p = mReferenceManager->getFrame(i).position;

		// p[4] += 5.0;
		// p.setZero();
		mMotionBVH.push_back(p);
		// mSkelBVH->setPositions(p);
		// Eigen::Vector3d leftFootPos = mSkelBVH->getBodyNode("LeftFoot")->getWorldTransform().translation();
		// Eigen::Vector3d rightFootPos = mSkelBVH->getBodyNode("RightFoot")->getWorldTransform().translation();
		// std::cout << "frame: " << i << " leftFoot " << leftFootPos.transpose() << " rightFoot " << rightFootPos.transpose() << std::endl;
    }

	mTotalFrame = mMotionBVH.size() - 1;

	// GUI::setSkeletonColor(mSkelBVH, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));
	GUI::setSkeletonColor(mSkelBVH, Eigen::Vector4d(83./255.,143./255.,217./255., 1.0));
	// GUI::setSkeletonColor(mSkelPPO, Eigen::Vector4d(235./255., 235./255., 235./255., 1.0));

	std::string objPath = std::string(LAMA_DIR) + std::string("/data/character/object/door") + std::string(".xml");
	mObjSkel = SIM::SkeletonBuilder::buildFromFile(objPath).first;
	setFocusPolicy( Qt::StrongFocus );

}

void
MotionWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
MotionWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
MotionWidget::
setFrame(int n)
{
    mSkelBVH->setPositions(mMotionBVH[n]);
	Eigen::VectorXd objPos = mObjSkel->getPositions();
	objPos(objPos.size()-1) = (M_PI/2.0)*((double)n/30.0) + (-0.24*M_PI);
	mObjSkel->setPositions(objPos);
}
void
MotionWidget::
drawSkeletons()
{
	GUI::drawSkeleton(mSkelBVH, 0);
	// GUI::drawSkeleton(mObjSkel, 0);

	// draw XYZ axis
	// Eigen::Isometry3d root = mSkelBVH->getBodyNode(0)->getWorldTransform();
	// Eigen::Vector3d color = Eigen::Vector3d::Zero();
	// for(int i = 0; i < 3; i++) {
	// 	color(i) = 1;
	// 	Eigen::VectorXd axis = root.linear().col(i);
	// 	glColor3f(color[0], color[1], color[2]);
	// 	glPushMatrix();
	// 	Eigen::VectorXd pos = root.translation() + axis;
	// 	glTranslatef(pos[0], pos[1], pos[2]);
	// 	GUI::drawSphere(0.05);
	// 	glPopMatrix();
	// 	color.setZero();
	// }

	// draw Pose2d dir
	// SIM::Pose2d p2d = SIM::Transform::to2d(root);

	// glColor3f(0, 0, 0);
	// glBegin(GL_LINES);
	// glVertex3f(root.translation()[0], 0, root.translation()[2]);
	// glVertex3f(root.translation()[0] + p2d.dir[0], 0, root.translation()[2] + p2d.dir[1]);
	// glEnd();

	// glPushMatrix();
	// glTranslatef(1.5, 0, 0);
	// GUI::drawSkeleton(mSkelBVH, 0);
	// glPopMatrix();

}	
void
MotionWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelBVH->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
MotionWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	mCamera->apply();

	drawGround();
	drawSkeletons();
	GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
}
void
MotionWidget::
initLights()
{

	float ambient[]           	 = {0.4, 0.4, 0.4, 1.0};
	float diffuse[]             = {0.4, 0.4, 0.4, 1.0};
	float frontShininess[] = {60.0};
	float frontSpecular[]  = {0.2, 0.2,  0.2,  1.0};
	float frontDiffuse[]   = {0.2, 0.2, 0.2, 1.0};
	float lmodelAmbient[]      = {0.2, 0.2,  0.2,  1.0};
	float lmodelTwoside[]      = {GL_TRUE};

	GLfloat position[] = {0.0, 1.0, 1.0, 0.0};
	GLfloat position1[] = {0.0, 1.0, -1.0, 0.0};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodelAmbient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodelTwoside);

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, frontShininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  frontSpecular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   frontDiffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_FOG);
	GLfloat fogColor[] = {200.0/256.0, 200.0/256.0, 200.0/256.0, 1};
	glFogfv(GL_FOG_COLOR, fogColor);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_DENSITY, 0.05);
	glFogf(GL_FOG_START, 20.0);
	glFogf(GL_FOG_END, 40.0);
}
void
MotionWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();

}
void
MotionWidget::
keyPressEvent(QKeyEvent* _event)
{
	if(_event->key() == Qt::Key_Escape){
		exit(0);
	}
	if(_event->key() == Qt::Key_Space){
		mPlay = !mPlay;
		if(mPlay)
			std::cout << "Play." << std::endl;
		else 
			std::cout << "Pause." << std::endl;
	}
}
void
MotionWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
MotionWidget::
mouseMoveEvent(QMouseEvent* _event)
{
	if(!mIsDrag)
	return;

	if (mButton == Qt::MidButton)
		mCamera->translate(_event->x(), _event->y(), mPrevX, mPrevY);
	else if(mButton == Qt::LeftButton)
		mCamera->rotate(_event->x(), _event->y(), mPrevX, mPrevY);

	mPrevX = _event->x();
	mPrevY = _event->y();
	update();
}
void
MotionWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
MotionWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
MotionWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
MotionWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
MotionWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
MotionWidget::
togglePlay() {
	mPlay = !mPlay;
}
