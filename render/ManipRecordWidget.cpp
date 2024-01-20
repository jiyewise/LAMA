#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "ManipRecordWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
// #include "SimConfigParser.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "dart/external/lodepng/lodepng.h"
ManipRecordWidget::
ManipRecordWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false), mIsSnapshot(false), mDrawFrame(true), mDrawSceneMeshColor(true), mIsCapture(false), mShowOriginal(false)
{
	startTimer(30);
}
ManipRecordWidget::
ManipRecordWidget(std::string _dir)
  :ManipRecordWidget()
{

	mScreenshotTemp.resize(4*1920*1080);
	mScreenshotTemp2.resize(4*1920*1080);

	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    mSkelRecord = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
	parse(_dir);

	mCurFrame = 0;
	mTotalFrame = mMotionRecord.size()-1;
	getManipInfo();

	GUI::setSkeletonColor(mSkelRecord, Eigen::Vector4d(83./255.,160./255.,237./255., 1.0));
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::setSkeletonColor(mEnv->mScene->mObjectList[i].mObjectSkel,Eigen::Vector4d(209./255.,209./255.,219./255., 1.0));
	}
	GUI::setSkeletonColor(mSkelManipObject, Eigen::Vector4d(220./255., 105./255., 81./255., 1.0));


	setFocusPolicy( Qt::StrongFocus );

}

void 
ManipRecordWidget:: 
parse(std::string _dir)
{

	mDir = std::string(LAMA_DIR) + std::string("/result/manip/") + _dir + std::string("/");
	std::string envPath = mDir + std::string("env.xml");
	std::string manipICPath = mDir + std::string("manip_ic.xml");
	std::string motionPath = mDir + std::string("optimized.txt");
	std::string objMotionPath = mDir + std::string("manip_result_obj.txt");

	std::string originalMotionPath = mDir + std::string("optimized.txt");

	mEnv = new ENV::Environment(envPath);
	mEnv->parseManipIC(manipICPath);

	mSkelManipObject = mEnv->mManipObject.mObjectSkel;
	// read human motion
    std::ifstream is(motionPath);
	char buffer[256];	
	is >> buffer;
	int length = atoi(buffer);
	int dof = mSkelRecord->getNumDofs();
	Eigen::VectorXd pos(dof);
	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < dof; j++) {
			is >> buffer;
			pos(j) = atof(buffer);
		}	
		mMotionRecord.push_back(pos);
	}
    is.close();

	// read original motion (before edited for manipulation)
    std::ifstream origIs(originalMotionPath);
	char origBuffer[256];	
	origIs >> origBuffer;
	length = atoi(origBuffer);
	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < dof; j++) {
			origIs >> origBuffer;
			pos(j) = atof(origBuffer);
		}	
		mOriginalMotionRecord.push_back(pos);
	}
    origIs.close();


	// read manip object motion
	std::ifstream objIs(objMotionPath);
	char objBuffer[256];	
	objIs >> objBuffer;
	length = atoi(objBuffer);
	dof = mSkelManipObject->getNumDofs();
	Eigen::VectorXd objPos(dof);
	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < dof; j++) {
			objIs >> objBuffer;
			objPos(j) = atof(objBuffer);
		}	
		mManipObjectRecord.push_back(objPos);
	}
    objIs.close();

}

// void
// ManipRecordWidget:: 
// getManipInfo()
// {
// 	ENV::Object manipObj = mEnv->mManipObject;
// 	ENV::ManipInteractionCue manipIC = mEnv->mManipIC;
// 	int bnIdx = manipIC.bnIdx;
// 	int vertex = manipIC.vertex;

// 	for(auto mc : manipIC.mContactList) {
// 		Eigen::VectorXd objPos = mc.objPos;
// 		manipObj.mObjectSkel->setPositions(objPos);
// 		manipObj.applyTransformAll();
// 		Eigen::Vector3d contactPos = manipObj.getVertexPos(bnIdx, vertex);
// 		contactPos[1] += 0.35;
// 		mManipContactPos.push_back(contactPos);
// 		mMotionManipObject.push_back(std::pair<int, Eigen::VectorXd>(mc.frame, objPos));
// 		// convert to contact
// 		ENV::Contact c = ENV::Contact(mc.frame, manipIC.jointIdx, contactPos);
// 		mManip2ContactList.push_back(c);
// 	}
// 	manipObj.mObjectSkel->setPositions(manipIC.mInitialObjPos);
// }

// void 
// ManipRecordWidget:: 
// readMotion(std::string _dir)
// {
// 	mDir = _dir + std::string("/");
//     std::string originalPath = LAMA_DIR + std::string("/result/") + _dir + std::string("/original.txt");
//     std::string editedPath = LAMA_DIR + std::string("/result/") + _dir + std::string("/optimized.txt");

//     // parse original motion
//     std::ifstream is(originalPath);
// 	char buffer[256];
	
// 	is >> buffer;
// 	int length = atoi(buffer);
	
// 	int dof = mSkelRecord->getNumDofs();
// 	Eigen::VectorXd pos(dof);
//     // std::cout << "original: " << length << std::endl;
// 	// for(int i = 0; i < length; i++)
// 	// {
// 	// 	for(int j = 0; j < dof; j++) {
// 	// 		is >> buffer;
// 	// 		pos(j) = atof(buffer);
// 	// 	}	
// 	// 	mMotionOriginal.push_back(pos);
// 	// }
//     // is.close();

//     // parse optimized motion
//     std::ifstream isOpt(editedPath);
// 	char buffer2[256];
// 	isOpt >> buffer2;
// 	length = atoi(buffer2);
//     std::cout << "edited: " << length << std::endl;

// 	for(int i = 0; i < length; i++)
// 	{
// 		for(int j = 0; j < dof; j++) {
// 			isOpt >> buffer2;
// 			pos(j) = atof(buffer2);
// 		}	
// 		mMotionRecord.push_back(pos);
// 	}
//     isOpt.close();
// 	mTotalFrame = mMotionRecord.size()-1;
// }



void
ManipRecordWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
ManipRecordWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}

void
ManipRecordWidget::
setFrame(int n)
{
	// if (n == 0) {
	// 	mEnv->mManipObject.mObjectSkel->setPositions(mEnv->mManipIC.mInitialObjPos);
	// }

	// int offset = n;
	// if(mIsEditMode) {
	// 	if(mIsSnapshot) {
	// 		// drawSnapshot();
	// 		return;
	// 	}
	// 	mSkelEdit->setPositions(mMotionEdited[n]);
	// 	int manipStartFrame = mManipMotionEditor->getStartManipFrame();
	// 	offset = n-manipStartFrame;
	// }

	// else if(mIsRecord) {
	// 	mSkelRecord->setPositions(mMotionRecord[n]);
	// 	return;
	// }

	// for(auto p: mMotionManipObject) {
	// 	if (offset == p.first)
	// 		mEnv->mManipObject.mObjectSkel->setPositions(p.second);
	// }
	
	Eigen::VectorXd pos;
	if(mShowOriginal) {
		if(n >= mOriginalMotionRecord.size())
			n = mOriginalMotionRecord.size()-1;
	}
	pos = (mShowOriginal) ? mOriginalMotionRecord[n] : mMotionRecord[n];
	mSkelRecord->setPositions(pos);
	Eigen::VectorXd objPos;
	objPos = (mShowOriginal) ? mManipObjectRecord[0] : mManipObjectRecord[n]; 
	mSkelManipObject->setPositions(objPos);

}

// void
// ManipRecordWidget:: 
// drawContactTrajectory() 
// {
// 	glColor3f(1./255.,183./255.,183./255.);
// 	for(int i = 0; i < mManipContactPos.size(); i++) {
// 		glPushMatrix();
// 		Eigen::Vector3d pos = mManipContactPos[i];
// 		glTranslatef(pos[0], pos[1], pos[2]);
// 		GUI::drawSphere(0.02);
// 		glPopMatrix();
// 	}
// }

// void 
// ManipRecordWidget::
// drawIC()
// {
// 	for(int i = 0; i < mEnv->mInteractionCueList.size(); i++) {
// 		if(mEnv->mInteractionCueList[i].mType == "sit" || mEnv->mInteractionCueList[i].mType == "stop") {
// 			glPushMatrix(); 
// 			glTranslatef(mEnv->mInteractionCueList[i].mContactList[0].contactPos[0], mEnv->mInteractionCueList[i].mContactList[0].contactPos[1], mEnv->mInteractionCueList[i].mContactList[0].contactPos[2]);
// 			GUI::drawSphere(0.05);
// 			glPopMatrix();
// 		}
// 		// draw orientation
// 		if(mEnv->mInteractionCueList[i].mContactList[0].jointIdx == 0) {
// 			Eigen::Vector2d rootDir = mEnv->mInteractionCueList[i].mContactList[0].contactRootPos.dir;
// 			Eigen::Vector3d pos = mEnv->mInteractionCueList[i].mContactList[0].contactPos;
// 			glLineWidth(3.0);
// 			glBegin(GL_LINES);
// 			glVertex3f(pos[0], pos[1], pos[2]);
// 			glVertex3f(pos[0]+0.2*rootDir[0], pos[1], pos[2]+0.2*rootDir[1]);
// 			glEnd();
// 		}
// 	}
// }


void
ManipRecordWidget:: 
getManipInfo()
{
	ENV::Object manipObj = mEnv->mManipObject;
	ENV::ManipInteractionCue manipIC = mEnv->mManipIC;
	int bnIdx = manipIC.bnIdx;
	int vertex = manipIC.vertex;
	for(auto mc : manipIC.mContactList) {
		Eigen::VectorXd objPos = mc.objPos;
		manipObj.mObjectSkel->setPositions(objPos);
		manipObj.applyTransformAll();
		Eigen::Vector3d contactPos = manipObj.getVertexPos(bnIdx, vertex);
		// contactPos[1] += 0.35;
		mManipContactPos.push_back(contactPos);
	}
}

void
ManipRecordWidget:: 
drawContactTrajectory() 
{
	// glColor3f(1,0,0);
	// glColor3f(0.2,0.85,0.21);
	// if(mDrawSceneMeshColor)
	// 	glColor3f(0.0,1,1);
	// if(!mDrawSceneMeshColor)
	for(int i = 0; i < mManipContactPos.size(); i++) {
		// glColor3f(20./255.,20./255.,20./255.);
		glColor3f(1./255.,183./255.,183./255.);
		// if((mCurFrame-81) != i) {
		// 	glPushMatrix();
		// 	Eigen::Vector3d pos = mManipContactPos[i];
		// 	glTranslatef(pos[0], pos[1], pos[2]);
		// 	GUI::drawSphere(0.02);
		// 	glPopMatrix();			
		// 	continue;
		// }
		glPushMatrix();
		Eigen::Vector3d pos = mManipContactPos[i];
		glTranslatef(pos[0], pos[1], pos[2]);
		GUI::drawSphere(0.03);
		glPopMatrix();
	}
}

void
ManipRecordWidget::
drawSkeletons()
{
	// GUI::drawSkeleton(mSkelBVH, 0);
	// draw scene
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::drawSkeleton(mEnv->mScene->mObjectList[i].mObjectSkel, 0, mDrawSceneMeshColor);
	}

	glColor3f(1./255.,183./255.,183./255.);
	if(mIsSnapshot) {
		for(int i = 0; i < mSkelList.size(); i++) {
			// if(i == mSkelList.size()-1) continue;
			// if(i == mSkelList.size()-2) continue;
			// if(i == mSkelList.size()-3) continue;
			if(i == mSkelList.size()-4) continue;
			if(i == mSkelList.size()-5) continue;
		GUI::drawSkeleton(mSkelList[i]);
		}
		for(int i = 0; i < mManipObjList.size(); i++) {
			if(i == mManipObjList.size()-1)	continue;
		GUI::drawSkeleton(mManipObjList[i]);
		}
		// return;
	}

	else {
		GUI::drawSkeleton(mSkelRecord);
		GUI::drawSkeleton(mSkelManipObject, 0, false);
	}

}	
void
ManipRecordWidget::
drawGround()
{
	// Eigen::Vector3d comRoot;
	// comRoot = mSkelPPO->getRootBodyNode()->getCOM();
	GUI::drawGround(0, 0, 0);
}
void
ManipRecordWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	for(auto tuple : mCameraList) {
		if(mDir.find(std::get<0>(tuple)) != std::string::npos) {
			Eigen::Vector3d lookAt = std::get<1>(tuple);
			Eigen::Vector3d eye = std::get<2>(tuple); 
			Eigen::Vector3d up = mCamera->getUp();
			mCamera->setCamera(lookAt, eye, up);
		}
	}

	mCamera->apply();

	drawSkeletons();

	if(mDrawFrame) {
		GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
		drawContactTrajectory();
		// drawIC();
	}

	if(mIsCapture)
		Screenshot();

}
void
ManipRecordWidget::
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
ManipRecordWidget::
timerEvent(QTimerEvent* _event)
{
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
		// std::cout << "mCurFrame: " << mCurFrame << std::endl;
	} 
	setFrame(mCurFrame);
	update();

}
void
ManipRecordWidget::
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
	if(_event->key() == Qt::Key_S) {
		mIsSnapshot = !mIsSnapshot;
	}
	if(_event->key() == Qt::Key_F) {
		mDrawFrame = !mDrawFrame;
	}
	if(_event->key() == Qt::Key_M) {
		mDrawSceneMeshColor = !mDrawSceneMeshColor;
	}
	if(_event->key() == Qt::Key_C) {
		mIsCapture = !mIsCapture;
	}
	if(_event->key() == Qt::Key_O) {
		mShowOriginal = !mShowOriginal;
	}

}
void
ManipRecordWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
ManipRecordWidget::
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
ManipRecordWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
ManipRecordWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}

// slot functions
void
ManipRecordWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
ManipRecordWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
ManipRecordWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
ManipRecordWidget::
togglePlay() {
	mPlay = !mPlay;
}

// capture
void 
ManipRecordWidget::
Screenshot() {
  std::string capdir = mDir + std::string("/capture");
  int length = capdir.length();
  static int count = 0;
  const char* directory = capdir.c_str();
  const char fileBase[8] = "Capture";
  char fileName[200];

  boost::filesystem::create_directories(directory);
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
  int tw = 1920;
  int th = 1080;

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return ;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return ;
  }
}

// void
// ManipRecordWidget::
// saveOriginalMotion(std::string _filename)
// {
// 	// save
// 	std::cout << "start saving original motion" << std::endl;
// 	std::ofstream ofs(_filename);
// 	ofs << mMotionPPO.size() << std::endl;
// 	int length = mMotionPPO.size();
// 	for(int i = 0; i < length; i++) {
// 		ofs << mMotionPPO[i].transpose() << std::endl;
// 	}
// 	ofs.close();
// }

// void 
// ManipRecordWidget::
// saveTransitionRecord(std::string _filename)
// {
// 	// save env file name
// 	// save transition record
// 	std::cout << "start transition record" << std::endl;
// 	std::ofstream ofs(_filename);
// 	std::vector<std::tuple<int,int,std::string,ENV::InteractionCue>> tRecord = mController->mMotionSynthesizer->mTransitionRecord;
// 	ofs << tRecord.size() << std::endl;
// 	for(auto t : tRecord) {
// 		ofs << std::get<0>(t) << " " << std::get<1>(t) << " " << std::get<2>(t) << " ic: " << std::get<3>(t).mType << std::endl;
// 	}
// 	// write transition cost
// 	ofs << "transitionCost" << std::endl;
// 	std::vector<std::map<std::string, double>> transitionRecordList = mController->getRecordTransitionCost();
// 	// TODO write
// 	ofs.close();
// }

// void 
// ManipRecordWidget::
// saveOptimizeMotion(std::string _filename)
// {
// 	std::vector<Eigen::VectorXd> toSave;
// 	if(mMotionEdited.size() == 0) {
// 		toSave = mMotionPPO;
// 		std::cout << "warning: original motion will be saved" << std::endl;
// 	}
// 	else {
// 		toSave = mMotionEdited;
// 	}
// 	// save edited 
// 	// std::string path = LAMA_DIR + std::string("/result/") + std::string("test_edited") + std::string(".txt");
// 	std::cout << "start saving optimized motion" << std::endl;
// 	std::ofstream ofs(_filename);
// 	ofs << toSave.size() << std::endl;
// 	int length = toSave.size();
// 	for(int i = 0; i < length; i++) {
// 		ofs << toSave[i].transpose() << std::endl;
// 	}
// 	ofs.close();
// }