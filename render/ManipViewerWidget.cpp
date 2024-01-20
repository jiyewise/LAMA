#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "ManipViewerWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
// #include "SimConfigParser.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "dart/external/lodepng/lodepng.h"
ManipViewerWidget::
ManipViewerWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false), mIsRecord(false), mIsEditMode(false), mIsSnapshot(false), mDrawFrame(true), mDrawSceneMeshColor(true), mIsCapture(false)
{
	startTimer(30);
}
ManipViewerWidget::
ManipViewerWidget(std::string _env, std::string _manipIC, std::string _dir)
  :ManipViewerWidget()
{

	mScreenshotTemp.resize(4*1690*975);
	mScreenshotTemp2.resize(4*1690*975);

	mCurFrame = 0;
	mTotalFrame = 120;

	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    std::string envPath = std::string(LAMA_DIR) + std::string("/env/env_config/") + _env + std::string(".xml");
	std::string manipICPath = std::string(LAMA_DIR) + std::string("/data/manip_ic/") + _manipIC + std::string(".xml");

	mEnv = new ENV::Environment(envPath);
	mEnv->parseManipIC(manipICPath);
	getManipInfo();

    mSkelRecord = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
	mSkelEdit = mSkelRecord->cloneSkeleton();
  

	GUI::setSkeletonColor(mSkelRecord, Eigen::Vector4d(83./255.,160./255.,237./255., 1.0));
	GUI::setSkeletonColor(mSkelEdit, Eigen::Vector4d(83./255.,160./255.,237./255., 1.0));
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::setSkeletonColor(mEnv->mScene->mObjectList[i].mObjectSkel,Eigen::Vector4d(209./255.,209./255.,219./255., 1.0));
	}
	GUI::setSkeletonColor(mEnv->mManipObject.mObjectSkel, Eigen::Vector4d(220./255., 105./255., 81./255., 1.0));


	if(_dir != "") {
		mIsRecord = true;
		readMotion(_dir);
		mManipMotionEditor = new ENV::ManipMotionEditor(mSkelEdit);
		std::string aePath = LAMA_DIR + std::string("/autoencoder/output/") + std::string("cnn3_sit_mod_normRoot2");
		mManipMotionEditor->loadAutoEncoder(aePath);
	}
	// if motion dir is not none:
	// first read motion from dir and set to original motion clip
	// load motion editor and autoencoder

	// mSkelNum = 10;
	// mObjNum = 6;

	mSnapshotNum = 12;

	mColorMap.clear();
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("RightHand", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("RightArm", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("RightForeArm", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("RightShoulder", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("LeftHand", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("LeftArm", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("LeftForeArm", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("LeftShoulder", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Hips", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Spine", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Spine1", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Spine2", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Neck", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));
	mColorMap.insert(std::pair<std::string, Eigen::Vector4d>("Head", Eigen::Vector4d(72./255.,191./255.,227./255., 1.0)));

	mColorGrad.clear();
	mColorGrad.push_back(Eigen::Vector4d(128./255.,254./255.,219./255., 1.0)); // aqua
	mColorGrad.push_back(Eigen::Vector4d(114./255.,239./255.,221./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(100./255.,223./255.,223./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(86./255.,207./255.,225./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(72./255.,191./255.,227./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(75./255.,178./255.,224./255., 1.0)); // manually added
	mColorGrad.push_back(Eigen::Vector4d(78./255.,168./255.,222./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(83./255.,143./255.,217./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(94./255.,95./255.,206./255., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(99./255.,77./255.,212./255., 1.0)); // manually added
	mColorGrad.push_back(Eigen::Vector4d(105./255.,48./255.,219./195., 1.0));
	mColorGrad.push_back(Eigen::Vector4d(115./255.,0./255.,219./184., 1.0)); // purple 
	mColorGrad.push_back(Eigen::Vector4d(125./255.,0./255.,222./184., 1.0)); // more purple
	
	// for(int i = 0; i <= mSkelNum; i++) {
	// 	mSkelList.push_back(mSkelEdit->cloneSkeleton());
	// }
	// for(int i = 0; i <= mObjNum; i++) {
	// 	mManipObjList.push_back(mEnv->mManipObject.mObjectSkel->cloneSkeleton());
	// }

	// int count = 0;
	// for(int i = mSkelList.size()-1; i >= 0; i--) {
	// 	GUI::setSkeletonColor(mSkelList[i], mColorGrad[count]);
	// 	count++;
	// }

	setFocusPolicy( Qt::StrongFocus );
	// initNetworkSetting(_env, _ppo);

}

void
ManipViewerWidget:: 
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
		// contactPos[1] += 0.3;
		mManipContactPos.push_back(contactPos);
		mMotionManipObject.push_back(std::pair<int, Eigen::VectorXd>(mc.frame, objPos));
		// convert to contact
		ENV::Contact c = ENV::Contact(mc.frame, manipIC.jointIdx, contactPos);
		mManip2ContactList.push_back(c);
	}
	manipObj.mObjectSkel->setPositions(manipIC.mInitialObjPos);
}

void 
ManipViewerWidget:: 
readMotion(std::string _dir)
{
	mDir = _dir + std::string("/");
    std::string originalPath = LAMA_DIR + std::string("/result_jiye/") + _dir + std::string("/original.txt");
    std::string editedPath = LAMA_DIR + std::string("/result_jiye/") + _dir + std::string("/optimized.txt");

    // parse original motion
    std::ifstream is(originalPath);
	char buffer[256];
	
	is >> buffer;
	int length = atoi(buffer);
	
	int dof = mSkelRecord->getNumDofs();
	Eigen::VectorXd pos(dof);
    // std::cout << "original: " << length << std::endl;
	// for(int i = 0; i < length; i++)
	// {
	// 	for(int j = 0; j < dof; j++) {
	// 		is >> buffer;
	// 		pos(j) = atof(buffer);
	// 	}	
	// 	mMotionOriginal.push_back(pos);
	// }
    // is.close();

    // parse optimized motion
    std::ifstream isOpt(editedPath);
	char buffer2[256];
	isOpt >> buffer2;
	length = atoi(buffer2);
    std::cout << "edited: " << length << std::endl;

	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < dof; j++) {
			isOpt >> buffer2;
			pos(j) = atof(buffer2);
		}	
		mMotionRecord.push_back(pos);
	}
    isOpt.close();
	mTotalFrame = mMotionRecord.size()-1;
}

void  
ManipViewerWidget::
optimize(int _start, int _end)
{
	// std::vector<Eigen::VectorXd> clip;
	// if (_start == _end)
	// 	clip.push_back(mMotionRecord[_clip]);
	// for(int i = _start; i < _end; i++) {
	// 	int idx = i;
	// 	if(idx >= mMotionRecord.size())
	// 		idx = mMotionRecord.size()-1; 
	// 	clip.push_back(mMotionRecord[idx]);
	// }

	// send contact list, cur frame into motion editor
	// bring edited motion
	if(!mIsRecord)
		return;

	mManipMotionEditor->setManipPointInMotion(mMotionRecord);
	mManipMotionEditor->optimizeAutoEncoder(_start, _end, mManip2ContactList, false); // later add index
	mManipMotionEditor->setEditedMotion(false); 
	mMotionEdited = mManipMotionEditor->getEditedMotion();
	mIsEditMode = true;
	mTotalFrame = mMotionEdited.size()-1;
	mCurFrame = 0;
	setSnapshotInfo();
	// save object pos
	int manipStartFrame = mManipMotionEditor->getStartManipFrame();
	std::cout << __func__ << " manip start frame: " << manipStartFrame << std::endl;

	for(int i = 0; i < manipStartFrame; i++) {
		mMotionManipRecord.push_back(mEnv->mManipIC.mInitialObjPos);
	}
	int startContact = mMotionManipObject[0].first;
	for(int i = manipStartFrame; i < mMotionEdited.size(); i++) {
		int offset = i - manipStartFrame;	
		if(offset < startContact) {
			mMotionManipRecord.push_back(mEnv->mManipIC.mInitialObjPos);
			continue;
		}
		for(auto p: mMotionManipObject) {
			if (offset == p.first) {
				mMotionManipRecord.push_back(p.second);
				// std::cout << "recorded: " << offset << " / " << p.second.transpose() << std::endl;
			}
		}
	}
	Eigen::VectorXd endPos = mMotionManipRecord.back();
	for(int i = mMotionManipRecord.size(); i < mMotionEdited.size(); i++) {
		mMotionManipRecord.push_back(endPos);
	}
}

void 
ManipViewerWidget::
optimizeIK(int _start, int _end)
{
	if(!mIsRecord)
		return;

	mManipMotionEditor->setManipPointInMotion(mMotionRecord);
	mManipMotionEditor->editWithIK(_start, _end, mManip2ContactList, false); // later add index
	// mManipMotionEditor->setEditedMotion(false); 
	mMotionEdited = mManipMotionEditor->getEditedMotion();
	mIsEditMode = true;
	mTotalFrame = mMotionEdited.size()-1;
	mCurFrame = 0;
	setSnapshotInfo();

	// save object pos
	int manipStartFrame = mManipMotionEditor->getStartManipFrame();
	std::cout << __func__ << " manip start frame: " << manipStartFrame << std::endl;

	for(int i = 0; i < manipStartFrame; i++) {
		mMotionManipRecord.push_back(mEnv->mManipIC.mInitialObjPos);
	}
	int startContact = mMotionManipObject[0].first;
	for(int i = manipStartFrame; i < mMotionEdited.size(); i++) {
		int offset = i - manipStartFrame;	
		if(offset < startContact) {
			mMotionManipRecord.push_back(mEnv->mManipIC.mInitialObjPos);
			continue;
		}
		for(auto p: mMotionManipObject) {
			if (offset == p.first) {
				mMotionManipRecord.push_back(p.second);
				std::cout << "recorded: " << offset << " / " << p.second.transpose() << std::endl;
			}
		}
	}
	Eigen::VectorXd endPos = mMotionManipRecord.back();
	for(int i = mMotionManipRecord.size(); i < mMotionEdited.size(); i++) {
		mMotionManipRecord.push_back(endPos);
	}
}

void 
ManipViewerWidget:: 
setSnapshotInfo()
{
	int manipStartFrame = mManipMotionEditor->getStartManipFrame();
	for(int i = 0; i <= mSnapshotNum; i++) {
		double weight = i/(double)mSnapshotNum; 
		weight = pow(weight, 0.77);
		int frame = mTotalFrame*weight;
		mSkelList.push_back(mSkelEdit->cloneSkeleton());
		mSkelList.back()->setPositions(mMotionEdited[frame]);
		if(frame > manipStartFrame) {
			int offset = frame - manipStartFrame;
			for(auto p: mMotionManipObject) {
				if (offset == p.first) {
					mManipObjList.push_back(mEnv->mManipObject.mObjectSkel->cloneSkeleton());
					mManipObjList.back()->setPositions(p.second);
				}
			}
		}
		if(frame == mTotalFrame) {
			mManipObjList.push_back(mEnv->mManipObject.mObjectSkel->cloneSkeleton());
			mManipObjList.back()->setPositions(mMotionManipObject.back().second);			
		}
	}
	// color human skeleton
	int count = 0;
	for(int i = mSkelList.size()-1; i >= 0; i--) {
		GUI::setSkeletonColor(mSkelList[i], mColorGrad[count]);
		count++;
	}
}

void
ManipViewerWidget::
saveMotion(std::string _dir, int _start, int _end)
{
	std::cout << "start saving manip motion" << std::endl;
	mSaveDir = _dir;
	std::string filePath = _dir + "manip_result.txt";
	std::string objFilePath = _dir + "manip_result_obj.txt";
	if(_end >= mMotionEdited.size())
		_end = mMotionEdited.size()-1;
	if(_start < 0)
		_start = 0;
	int size = _end - _start + 1;
	
	std::ofstream ofs(filePath);
	ofs << size << std::endl;
	for(int i = _start; i <= _end; i++) {
		ofs << mMotionEdited[i].transpose() << std::endl;
	}
	ofs.close();

	std::ofstream objOfs(objFilePath);
	objOfs << size << std::endl;
	for(int i = _start; i <= _end; i++) {
		if(i > mMotionManipRecord.size()-1)
			i = mMotionManipRecord.size()-1;
		objOfs << mMotionManipRecord[i].transpose() << std::endl;
	}
	objOfs.close();
	
}

void
ManipViewerWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
ManipViewerWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}

void 
ManipViewerWidget:: 
drawSnapshot()
{
	mSkelList[0]->setPositions(mMotionEdited[0]);
	int num = mSkelNum-3;
	for(int i = 1; i < num; i++) {
		int frame = i*(mMotionEdited.size() / mSkelNum);
		frame += 5;
		if (frame >= mMotionEdited.size())
			frame = mMotionEdited.size()-1;
		Eigen::VectorXd pos = mMotionEdited[frame];
		mSkelList[i]->setPositions(pos);
	}
	mSkelList[mSkelList.size()-5]->setPositions(mMotionEdited[mMotionEdited.size()-47]);
	mSkelList[mSkelList.size()-4]->setPositions(mMotionEdited[mMotionEdited.size()-33]);
	mSkelList[mSkelList.size()-3]->setPositions(mMotionEdited[mMotionEdited.size()-30]);
	mSkelList[mSkelList.size()-2]->setPositions(mMotionEdited[mMotionEdited.size()-27]);
	mSkelList[mSkelList.size()-1]->setPositions(mMotionEdited[mMotionEdited.size()-23]);

	// draw object
	mManipObjList[0]->setPositions(mMotionManipObject[0].second);
	for(int i = 1; i < mObjNum; i++) {
		int frame = i*(mMotionManipObject.size() / mObjNum);
		frame += 5;
		if (frame >= mMotionManipObject.size())
			frame = mMotionManipObject.size()-1;
		Eigen::VectorXd pos = mMotionManipObject[frame].second;
		mManipObjList[i]->setPositions(pos);
	}
	mManipObjList.back()->setPositions(mMotionManipObject.back().second);
}

void
ManipViewerWidget::
setFrame(int n)
{
	if (n == 0) {
		mEnv->mManipObject.mObjectSkel->setPositions(mEnv->mManipIC.mInitialObjPos);
	}

	int offset = n;
	if(mIsEditMode) {
		if(mIsSnapshot) {
			// drawSnapshot();
			return;
		}
		mSkelEdit->setPositions(mMotionEdited[n]);
		int manipStartFrame = mManipMotionEditor->getStartManipFrame();
		offset = n-manipStartFrame;
	}

	else if(mIsRecord) {
		mSkelRecord->setPositions(mMotionRecord[n]);
		return;
	}

	for(auto p: mMotionManipObject) {
		if (offset == p.first)
			mEnv->mManipObject.mObjectSkel->setPositions(p.second);
	}
	// if(mIsEditMode) {
	// 	mSkelEdit->setPositions(mMotionEdited[n]);
	// } else {
	// 	mTargetTrajectory = mMotionTargetTrajectory[n];
	// 	mCurTrajectory = mMotionCurrentTrajectory[n];	
	// }
	// mSkelPPO->setPositions(mMotionPPO[n]);
}

void
ManipViewerWidget:: 
drawContactTrajectory() 
{
	// glColor3f(1,0,0);
	// glColor3f(0.2,0.85,0.21);
	// if(mDrawSceneMeshColor)
	// 	glColor3f(0.0,1,1);
	// if(!mDrawSceneMeshColor)
		glColor3f(1./255.,183./255.,183./255.);
	for(int i = 0; i < mManipContactPos.size(); i++) {
		glPushMatrix();
		Eigen::Vector3d pos = mManipContactPos[i];
		glTranslatef(pos[0], pos[1], pos[2]);
		GUI::drawSphere(0.02);
		glPopMatrix();
	}
}

void
ManipViewerWidget::
drawSkeletons()
{
	// GUI::drawSkeleton(mSkelBVH, 0);

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
	else if(mIsEditMode) {
		// GUI::drawSkeleton(mSkelEdit, mColorMap, 0);
		GUI::drawSkeleton(mSkelEdit, 0);
		// if(mDrawRef) GUI::drawSkeleton(mSkelPPO, 0);
	}
	else if(mIsRecord){
		GUI::drawSkeleton(mSkelRecord, 0);	
	}

	// draw scene
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::drawSkeleton(mEnv->mScene->mObjectList[i].mObjectSkel, 0, mDrawSceneMeshColor);
	}

	GUI::drawSkeleton(mEnv->mManipObject.mObjectSkel, 0, false);

}	
void
ManipViewerWidget::
drawGround()
{
	// Eigen::Vector3d comRoot;
	// comRoot = mSkelPPO->getRootBodyNode()->getCOM();
	GUI::drawGround(0, 0, 0);
}
void
ManipViewerWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	mCamera->apply();

	// drawGround();
		// draw manip object
	drawSkeletons();
	// if(!mIsSnapshot)
	// 	drawContactTrajectory();

	// if(mRunPPO) 
	// 	GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mTiming[mCurFrame])+" / "+std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
	// else 
	if(mDrawFrame) {
		GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
		glColor3f(1./255.,183./255.,183./255.);
		drawContactTrajectory();

	for(int i = 0; i < mEnv->mInteractionCueList.size(); i++) {
		if(mEnv->mInteractionCueList[i].mType == "sit" || mEnv->mInteractionCueList[i].mType == "stop") {
			glColor3f(250./255.,132./255.,132./255.);
			glPushMatrix(); 
			glTranslatef(mEnv->mInteractionCueList[i].mContactList[0].contactPos[0], mEnv->mInteractionCueList[i].mContactList[0].contactPos[1], mEnv->mInteractionCueList[i].mContactList[0].contactPos[2]);
			GUI::drawSphere(0.05);
			glPopMatrix();
		}
		// draw orientation
		if(mEnv->mInteractionCueList[i].mContactList[0].jointIdx == 0) {
			Eigen::Vector2d rootDir = mEnv->mInteractionCueList[i].mContactList[0].contactRootPos.dir;
			Eigen::Vector3d pos = mEnv->mInteractionCueList[i].mContactList[0].contactPos;
			glLineWidth(3.0);
			glBegin(GL_LINES);
			glVertex3f(pos[0], pos[1], pos[2]);
			glVertex3f(pos[0]+0.2*rootDir[0], pos[1], pos[2]+0.2*rootDir[1]);
			glEnd();
		}
	}
	}

	if(mIsCapture)
		Screenshot();

	// draw occupancy grid and path
	// GUI::drawGrid(mEnv->mScene->mGrid, mEnv->mScene->mBBox, 0.2);

}
void
ManipViewerWidget::
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
ManipViewerWidget::
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
ManipViewerWidget::
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
}
void
ManipViewerWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
ManipViewerWidget::
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
ManipViewerWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
ManipViewerWidget::
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
ManipViewerWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
ManipViewerWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
ManipViewerWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
ManipViewerWidget::
togglePlay() {
	mPlay = !mPlay;
}

// capture
void 
ManipViewerWidget::
Screenshot() {
  static int count = 0;
  const char directory[29] = "/home/jiye/Pictures/capture";
  const char fileBase[8] = "Capture";
  char fileName[45];

  boost::filesystem::create_directories(directory);
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
  int tw = 1690;
  int th = 975;

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
// ManipViewerWidget::
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
// ManipViewerWidget::
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
// ManipViewerWidget::
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