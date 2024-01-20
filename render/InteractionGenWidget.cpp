#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "InteractionGenWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
InteractionGenWidget::
InteractionGenWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false), mDrawSkel(false), mOriDelta(0), mHeightDelta(0),  mSelectPointIdx(-1), mSelectCueIdx(-1), mUnit(0.5)
{
	startTimer(30);
}
InteractionGenWidget::
InteractionGenWidget(std::string _env_path)
  :InteractionGenWidget()
{
	mCurFrame = 0;
	mTotalFrame = 0;

	mCurMode = std::string("view");

	std::string envPath = std::string(LAMA_DIR) + std::string("/env/env_config/") + _env_path + std::string(".xml");
	mEnv = new ENV::Environment(envPath);
	mEnvScene = mEnv->mScene;
	std::cout << "loading env done" << " " <<  envPath << std::endl;

	loadPosMap(); 
	std::cout << "loading pos map done" <<  std::endl;

	// mSceneOri.setZero();
	// mSceneOri[0] = -1.579079;

	setFocusPolicy( Qt::StrongFocus );
}

void
InteractionGenWidget::
loadPosMap()
{
	// load skeleton		
	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    mSkelBVH = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
	mInputSkel = mSkelBVH->cloneSkeleton();

    mReferenceManager = new SIM::ReferenceManager(SIM::SkeletonBuilder::buildFromFile(skelPath).first);

	std::map<std::string, std::string> motionNameMap;
	motionNameMap.insert(std::pair<std::string, std::string>(std::string("sit"), std::string("sit_idle_mxm.bvh")));
	motionNameMap.insert(std::pair<std::string, std::string>(std::string("initial"), std::string("stand_idle_mxm.bvh")));
	motionNameMap.insert(std::pair<std::string, std::string>(std::string("stop"), std::string("stand_idle_mxm.bvh")));

	for(auto const& m : motionNameMap){ 
		// load motion from reference manager
		std::string motionPath = std::string(LAMA_DIR) + std::string("/data/motion/") + m.second;
		mReferenceManager->loadMotionFromBVH(motionPath, false);
		// get first pos
		Eigen::VectorXd pos = mReferenceManager->getFrame(0).position;
		mPosMap.insert(std::pair<std::string, Eigen::VectorXd>(m.first, pos));
		mEditedPosMap.insert(std::pair<std::string, Eigen::VectorXd>(m.first, pos));
	}	
}

std::vector<std::string> 
InteractionGenWidget::
getJointNames()
{
	std::vector<std::string> names;
	for(int i = 0; i < mSkelBVH->getNumBodyNodes(); i++){
		auto bn = mSkelBVH->getBodyNode(i);
		names.push_back(bn->getName());
	}
	return names;
}

void 
InteractionGenWidget::
setRootTransform(double _angleDelta, double _heightDelta) 
{
	if(mIntersectPointList.size() == 0 || mSelectPointIdx == -1) {
		std::cout << "point should be selected first" << std::endl;
		return;
	}
	// normalize angle delta 
	if(_angleDelta > M_PI)
		_angleDelta -= 2*M_PI;
	else if(_angleDelta < -M_PI)
		_angleDelta += 2*M_PI;

	// get the interaction point 
	ENV::IntersectPoint p = mIntersectPointList[mSelectPointIdx];
	Eigen::Vector3d selectedPos = p.pos;
	double originalHeight = mPosMap[mCurMode](4);

	// get root rotation (for testing)
	SIM::Pose2d rootPos = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mPosMap[mCurMode].head<6>()));
	SIM::Pose2d rotatedPos = SIM::Transform::rotateDir(rootPos, _angleDelta);
	mEditedPosMap[mCurMode].head<6>() = dart::dynamics::FreeJoint::convertToPositions(SIM::Transform::to3d(rotatedPos));

	// set the root xz pos to the selected point
	mEditedPosMap[mCurMode].segment<3>(3) = selectedPos;

	mEditedPosMap[mCurMode](4) = (mCurMode == "sit") ? mEditedPosMap[mCurMode](4) + 0.1 : originalHeight;
	mEditedPosMap[mCurMode](4) += _heightDelta;
}

void
InteractionGenWidget::
setFoot()
{
	// remove foot penetration 
	if(mCurMode != "sit")
		return;
	for(int i = 0; i < 2; i++) {
		bool isLeft = i; // start with right foot
		std::string FootJoint = isLeft ? std::string("LeftFoot") : std::string("RightFoot");
		mSkelBVH->setPositions(mEditedPosMap[mCurMode]);
		Eigen::Vector3d target = mSkelBVH->getBodyNode(FootJoint)->getWorldTransform().translation();
		target(1) = 0.05;
		SIM::IKLimb(mSkelBVH, mEditedPosMap[mCurMode], target, isLeft);
	}
}



void
InteractionGenWidget::
createInteractionContact(std::vector<int> _jointClicked, int _frame)
{
	// if(mCurMode == "view" || mCurMode == "ray") {
	// 	std::cout << "Cannot create interaction cue in view or ray mode" << std::endl;
	// 	return;
	// }

	// // return if no interaction point is selected
	// if(mIntersectPointList.size() == 0 || mSelectPointIdx == -1) {
	// 	std::cout << "intersect point should be selected first" << std::endl; 
	// 	return;
	// }

	// ENV::IntersectPoint p = mIntersectPointList[mSelectPointIdx];

	// for(int bnIdx : _jointClicked) {
	// 	auto bn = mSkelBVH->getBodyNode(bnIdx);
	// 	std::string name = bn->getName();

	// 	ENV::Contact c;
	// 	if(mCurMode == "sit") {
	// 		Eigen::Isometry3d t = bn->getWorldTransform() * bn->getParentJoint()->getTransformFromChildBodyNode();
	// 		c = ENV::Contact(_frame, bnIdx, t.translation());
	// 	}

	// 	if(name.find("Foot") != std::string::npos || name.find("Toe") != std::string::npos) {
	// 		mContactList.push_back(c);
	// 		continue; // for foot, add object as ground (default)
	// 	}

	// 	c.objInfo.objIdx = p.objIdx;
	// 	c.objInfo.bnIdx = p.bnIdx;
	// 	c.objInfo.isChange = (mCurMode == "manip") ? true : false;
		
	// 	// for root, add orientation (expected use: only for sit mode)
	// 	if(bnIdx == 0) {
	// 		SIM::Pose2d rootPos = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mEditedPosMap[mCurMode].head<6>()));
	// 		c.contactRootPos = rootPos;
	// 	}
	// 	mContactList.push_back(c);
	// }

}


void 
InteractionGenWidget:: 
viewInputAndIC(bool _all)
{
	if(mCurMode == "view_cues")
		return;
	if(mSelectCueIdx < 0) {
		std::cout << "add cue first" << std::endl;
		return;
	}
	mDrawAllCues = _all;
	// mInputSkelList.clear();
	// int num = (_all) ? 1+mInteractionCueList.size() : 2;
	// for(int i = 0; i < num; i++) {
	// 	mInputSkelList.push_back(mSkelBVH->cloneSkeleton());
	// }
	mCurMode = "view_cues";
}

void 
InteractionGenWidget:: 
modifyIC()
{
	if(mSelectCueIdx < 0) {
		std::cout << "add cue first" << std::endl;
		return;
	}
	mInteractionCueList[mSelectCueIdx] = createInteractionCue();
}

ENV::InteractionCue
InteractionGenWidget::
createInteractionCue()
{
	// for initial IC is not created
	if(mCurMode == "initial" || !isAction(mCurMode)) {
		std::cout << "IC is saved only for action mode" << std::endl;
		return ENV::InteractionCue(""); 
	}
	ENV::InteractionCue iterCue = ENV::InteractionCue(mCurMode);

	// generate root & foot for actions (except stop) TODO change to mTargetJoints as in SitDatabase
	std::vector<int> targetJoints;
	targetJoints.push_back(0); // Root
	targetJoints.push_back(3); // LeftFoot
	targetJoints.push_back(7); // RightFoot

	for(int bnIdx : targetJoints) {
		if(bnIdx > 0 && mCurMode == std::string("stop")) 
			continue;
		auto bn = mSkelBVH->getBodyNode(bnIdx);
		Eigen::Isometry3d t = bn->getWorldTransform() * bn->getParentJoint()->getTransformFromChildBodyNode(); // for root, same as convertToTransform(head<6>())
		ENV::Contact c = ENV::Contact(bnIdx, t.translation());
		if(bnIdx == 0) {
			SIM::Pose2d rootPos = SIM::Transform::to2d(t);
			c.contactRootPos = rootPos;
		}
		iterCue.mContactList.push_back(c);
	}
	
	// print env info
	std::cout << "<Env name=\"env1\">" << std::endl;
	std::cout << "<Scene name=\"Scene1\">" << std::endl;
	
	// print obj info
	int numObjInScene = mEnvScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		Eigen::VectorXd pos = mEnvScene->mObjectList[i].mInitialPos;
		std::cout << "\t<Object name=\"object" << i << "\" file=\"" << mEnvScene->mObjectList[i].mFilePath << "\">" << std::endl;
		std::cout << "\t\t<ObjectPosition translation=\"" << pos.segment<3>(3).transpose() << "\" linear=\"" << pos.segment<3>(0).transpose() << "\"/>" << std::endl;
		std::cout << "\t</Object>" << std::endl;
	}
	std::cout << "</Scene>" << std::endl;

	// print initial
	SIM::Pose2d rootPos = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mEditedPosMap[std::string("initial")].head<6>()));
	std::cout << "<Initial pos=\"" << rootPos.pos.transpose() << "\" dir=\"" << rootPos.dir.transpose() << "\"/>" << std::endl;

	// print IC
	std::cout << "<Cue>" << std::endl;
	std::cout << "\t<ICList num=\"" << mInteractionCueList.size() << "\">" << std::endl;
	for(auto ic : mInteractionCueList) {
		std::cout << "\t<IC type=\"" << ic.mType << "\">" << std::endl;
		for(auto co: ic.mContactList) {
				std::cout << "\t\t<Contact jointIdx=\"" << co.jointIdx
						  << "\" contactPos =\"" << co.contactPos.transpose();
				if (co.jointIdx > 0)
					std::cout << "\"/>" << std::endl;
				else
					std::cout << "\" contactRootPos=\"" << co.contactRootPos.pos.transpose()
					<< "\" contactRootDir=\"" << co.contactRootPos.dir.transpose() << "\"/>" << std::endl;
		}
		std::cout << "\t</IC>" << std::endl;
	}
	std::cout << "\t</ICList>" << std::endl;
	std::cout << "</Cue>" << std::endl;
	std::cout << "</Env>" << std::endl;

	return iterCue;

}


void 
InteractionGenWidget:: 
exportInput(std::string _fileName)
{
	if(mSelectCueIdx < 0) {
		std::cout << "add cue first" << std::endl;
		return;
	}

	std::ofstream ofs(_fileName);
	// print env info
	ofs << "<Env name=\"env1\">" << std::endl;
	ofs << "<Scene name=\"Scene1\">" << std::endl;
	
	// print obj info
	int numObjInScene = mEnvScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		Eigen::VectorXd pos = mEnvScene->mObjectList[i].mInitialPos;
		ofs << "\t<Object name=\"object" << i << "\" file=\"" << mEnvScene->mObjectList[i].mFilePath << "\">" << std::endl;
		ofs << "\t\t<ObjectPosition translation=\"" << pos.segment<3>(3).transpose() << "\" linear=\"" << pos.segment<3>(0).transpose() << "\"/>" << std::endl;
		ofs << "\t</Object>" << std::endl;
	}
	ofs << "</Scene>" << std::endl;

	// print initial
	SIM::Pose2d rootPos = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mEditedPosMap[std::string("initial")].head<6>()));
	ofs << "<Initial pos=\"" << rootPos.pos.transpose() << "\" dir=\"" << rootPos.dir.transpose() << "\"/>" << std::endl;

	// print IC
	ofs << "<Cue>" << std::endl;

	ofs << "\t<ICList num=\"1\">" << std::endl;
	ENV::InteractionCue ic = mInteractionCueList[mSelectCueIdx];
	ofs << "\t<IC type=\"" << ic.mType << "\">" << std::endl;
	for(auto co: ic.mContactList) {
			ofs << "\t\t<Contact jointIdx=\"" << co.jointIdx
						<< "\" contactPos =\"" << co.contactPos.transpose();
			if (co.jointIdx > 0)
				ofs << "\"/>" << std::endl;
			else
				ofs << "\" contactRootPos=\"" << co.contactRootPos.pos.transpose()
				<< "\" contactRootDir=\"" << co.contactRootPos.dir.transpose() << "\"/>" << std::endl;
	}
	ofs << "\t</IC>" << std::endl;
	ofs << "\t</ICList>" << std::endl;
	ofs << "</Cue>" << std::endl;
	ofs << "</Env>" << std::endl;
	ofs.close();
	
}

void
InteractionGenWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
InteractionGenWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
InteractionGenWidget::
setFrame(std::string _type)
{
    // mSkelBVH->setPositions(mMotionBVH[n]);
	if (mEditedPosMap.find(_type) == mEditedPosMap.end()) {
		mDrawSkel = false;
		return;
	}
	mDrawSkel = true;
	mSkelBVH->setPositions(mEditedPosMap[_type]);
}
void
InteractionGenWidget::
drawSkeletons()
{
	// draw objects in the scene
	int numObjInScene = mEnvScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		if(i == 0) {
			Eigen::VectorXd pos = mEnvScene->mObjectList[i].mObjectSkel->getPositions();
			// pos.segment<3>(0) = mSceneOri;
			mEnvScene->mObjectList[i].mObjectSkel->setPositions(pos);
		}
		GUI::drawSkeleton(mEnvScene->mObjectList[i].mObjectSkel);
	}

	if(mDrawSkel)
		GUI::drawSkeleton(mSkelBVH, 0);
	


}	

void 
InteractionGenWidget::
drawMouseRay()
{
	if(mRay.size() != 2)
		return;
	
	Eigen::Vector3d point1 = mRay[0];
	Eigen::Vector3d point2 = mRay[1] * 10 + mRay[0];

	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(point1[0], point1[1], point1[2]);
	glVertex3f(point2[0], point2[1], point2[2]);
	glEnd();
	
}

void 
InteractionGenWidget::
drawIntersectPoint()
{
	if(mIntersectPointList.size() == 0 || mCurMode == "view_cues") {
		return; 
	}
	else {
		for(int i = 0; i < mIntersectPointList.size(); i++) {
			Eigen::Vector3d pos = mIntersectPointList[i].pos;
			Eigen::Vector3d color = (i == mSelectPointIdx) ? Eigen::Vector3d(1,0,0) : Eigen::Vector3d(0,0,1);
			double rad = (i == mSelectPointIdx) ? 0.015 : 0.01;
			
			glColor3f(color[0], color[1], color[2]);
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			GUI::drawSphere(rad);
			glPopMatrix();
		}
	}
	if(mCurMode == "manip") {
		glColor3f(0,1,0);
		glPushMatrix();
		glTranslatef(mIntersectPointMovedPos[0], mIntersectPointMovedPos[1], mIntersectPointMovedPos[2]);
		GUI::drawSphere(0.015);
		glPopMatrix();
	}
}


void 
InteractionGenWidget:: 
drawInteractionCue(ENV::InteractionCue _ic)
{
	for(int i = 0; i < _ic.mContactList.size(); i++) {
		ENV::Contact c = _ic.mContactList[i];
		glColor3f(1,0,0);
		glPushMatrix();
		glTranslatef(c.contactPos[0], c.contactPos[1], c.contactPos[2]);
		GUI::drawSphere(0.03);
		glPopMatrix();
		
		//draw orientation for root
		if(c.jointIdx == 0) {
			glBegin(GL_LINES);
			glVertex3f(c.contactPos[0], c.contactPos[1], c.contactPos[2]);
			glVertex3f(c.contactPos[0]+c.contactRootPos.dir[0]*0.1, c.contactPos[1], c.contactPos[2]+c.contactRootPos.dir[1]*0.1);
			glEnd();
		}
	}	
}

void
InteractionGenWidget::
drawInteractionCues()
{
	// only draw in view mode?
	if(mCurMode != "view_cues")
		return; 

	// draw initial
	mInputSkel->setPositions(mEditedPosMap["initial"]);
	GUI::drawSkeleton(mInputSkel);

	// draw IC		
	if(mDrawAllCues) {
		for(int i = 0; i < mInteractionCueList.size(); i++) {
			drawInteractionCue(mInteractionCueList[i]);
		}
	}
	else {
		drawInteractionCue(mInteractionCueList[mSelectCueIdx]);
	}

}

void
InteractionGenWidget::
drawGround()
{
	GUI::drawGround(0, 0, 0);
}
void
InteractionGenWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	mCamera->apply();

	drawGround();
	if(mCurMode == "ray")
		drawMouseRay();
	drawIntersectPoint();
	drawInteractionCues();
	
	GUI::drawStringOnScreen(0.2, 0.9, mCurMode, true, Eigen::Vector3d::Zero());
	GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
	
	drawSkeletons();
}
void
InteractionGenWidget::
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
InteractionGenWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurMode);
	update();

}

bool 
InteractionGenWidget::
isAction(std::string _curMode) {
	if (mPosMap.find(_curMode) == mPosMap.end()) {
		return false; // not found
	} 
	return true;
}

void
InteractionGenWidget::
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
	if(_event->key() == Qt::Key_A) {
		// mRayMode = !mRayMode;
		mCurMode = std::string("ray");
	}
	if(_event->key() == Qt::Key_S) {
		mCurMode = std::string("sit");
		mOriDelta = 0;
		mHeightDelta = 0;
		setRootTransform(mOriDelta, mHeightDelta);
	}
	if(_event->key() == Qt::Key_I) {
		mCurMode = std::string("initial");
		mOriDelta = 0;
		mHeightDelta = 0;
		std::cout << "ori and height delta: " << mOriDelta << " " << mHeightDelta << std::endl;
		setRootTransform(mOriDelta, mHeightDelta);
	}
	// if(_event->key() == Qt::Key_D) {
	// 	mCurMode = std::string("manip");
	// }
	if(_event->key() == Qt::Key_Return) {
		mCurMode = std::string("view");
	}
	if(_event->key() == Qt::Key_F) {
		mCurMode = std::string("view_cues");
	}
	
	// for rotation -> sit
	if(_event->key() == Qt::Key_1 && isAction(mCurMode)) {
		mOriDelta += 0.02;
		std::cout << "ori and height delta: " << mOriDelta << " " << mHeightDelta << std::endl;
		setRootTransform(mOriDelta, mHeightDelta);
	}
	if(_event->key() == Qt::Key_2 && isAction(mCurMode)) {
		mOriDelta -= 0.02;
		setRootTransform(mOriDelta, mHeightDelta);
	}
	if(_event->key() == Qt::Key_3 && isAction(mCurMode)) {
		setFoot();
	}
	if(_event->key() == Qt::Key_4 && isAction(mCurMode)) {
		mHeightDelta += 0.02;
		setRootTransform(mOriDelta, mHeightDelta);
	}
	if(_event->key() == Qt::Key_5 && isAction(mCurMode)) {
		mHeightDelta -= 0.02;
		setRootTransform(mOriDelta, mHeightDelta);
	}



}
void
InteractionGenWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
InteractionGenWidget::
mouseMoveEvent(QMouseEvent* _event)
{
	if(!mIsDrag)
		return;

	if(mCurMode == "ray")
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
InteractionGenWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	if((mCurMode == "ray") && mButton == Qt::LeftButton)
        computeMouseRay(_event->x(), _event->y());
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}

void 
InteractionGenWidget::
computeMouseRay(int _x, int _y)
{
	makeCurrent();
	
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLdouble winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );

	winX = (float)_x;
	winY = (float)viewport[3] - (float)_y;

	gluUnProject( winX, winY, 0.0, modelview, projection, viewport, &posX, &posY, &posZ);
	Eigen::Vector3d point = Eigen::Vector3d(posX, posY, posZ);

	gluUnProject( winX, winY, 0.99, modelview, projection, viewport, &posX, &posY, &posZ);

	Eigen::Vector3d dir = Eigen::Vector3d(posX, posY, posZ) - point;
	dir.normalize();

	mRay.clear();
	mRay.push_back(point);
	mRay.push_back(dir);

	// compute intersection with the scene
	mEnvScene->setIntersectInfo(point, dir);
	std::cout << "number of intersections : " << mEnvScene->mPointListScene.size() << std::endl;

	// if there's no intersection with the scene, calculate intersection with ground
	double t = -point(1) / dir(1);
	double x = dir(0) * t + point(0);
	double z = dir(2) * t + point(2);

	Eigen::Vector3d groundPoint = {x, 0.0, z};
	ENV::IntersectPoint groundIntersectPoint = ENV::IntersectPoint(groundPoint);
	mEnvScene->mPointListScene.push_back(groundIntersectPoint);

	// TODO: later move intersect point list to render related class
}


void
InteractionGenWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}

//////////////////////// slots ////////////////////////
void 
InteractionGenWidget::
updatePointList()
{
	mIntersectPointList = mEnvScene->mPointListScene;
	mSelectPointIdx = -1;
}

std::vector<ENV::IntersectPoint>
InteractionGenWidget::
getIntersectPoints()
{
	return mIntersectPointList;
}

void 
InteractionGenWidget::
setSelectedPoint(int _selectIdx)
{
	mSelectPointIdx = _selectIdx;
	mIntersectPointMovedPos = mIntersectPointList[mSelectPointIdx].pos;
}

void 
InteractionGenWidget::
setSelectedCue(int _selectIdx)
{
	mSelectCueIdx = _selectIdx;
}

// void
// InteractionGenWidget::
// nextFrame()
// { 
// 	if(!mPlay) {
// 		mCurFrame += 1;
// 		setFrame(mCurFrame);
// 	}
// }

// void
// InteractionGenWidget::
// prevFrame()
// {
// 	if(!mPlay && mCurFrame > 0) {
// 		mCurFrame -= 1;
// 		setFrame(mCurFrame);
// 	}
// }

// void
// InteractionGenWidget::
// reset()
// {
// 	mCurFrame = 0;
// 	setFrame(mCurFrame);
// }

void 
InteractionGenWidget::
togglePlay() {
	mPlay = !mPlay;
}