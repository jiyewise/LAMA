#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include "ActionControllerWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
// #include "SimConfigParser.h"
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <chrono>
#include "dart/external/lodepng/lodepng.h"

ActionControllerWidget::
ActionControllerWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false), mRunPPO(false), mIsEditMode(false), mDrawRef(false), mRemoveOffset(false), mIsCapture(false), mDrawSceneMeshColor(true), mDrawActionCue(true),
  mRD(), mMT(mRD()), mUniform(0.0, 1.0), mUseVariations(false)
{
	startTimer(30);
}
ActionControllerWidget::
ActionControllerWidget(std::string _env, std::string _ppo, bool _optimize)
  :ActionControllerWidget()
{

	mScreenshotTemp.resize(4*1920*1080);
	mScreenshotTemp2.resize(4*1920*1080);

	mCurFrame = 0;
	mTotalFrame = 0;

	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    std::string envPath = std::string(LAMA_DIR) + std::string("/env/env_config/") + _env + std::string(".xml");
	mEnv = new ENV::Environment(envPath);

    mSkelPPO = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
	mSkelEdit = mSkelPPO->cloneSkeleton();
  
	mActionRecord.clear();

	// GUI::setSkeletonColor(mSkelBVH, Eigen::Vector4d(235./255., 73./255., 73./255., 1.0));
	GUI::setSkeletonColor(mSkelPPO, Eigen::Vector4d(83./255.,143./255.,217./255., 1.0));
	GUI::setSkeletonColor(mSkelEdit, Eigen::Vector4d(83./255.,143./255.,217./255., 1.0));

	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::setSkeletonColor(mEnv->mScene->mObjectList[i].mObjectSkel,Eigen::Vector4d(209./255.,209./255.,219./255., 1.0));
	}

	setFocusPolicy( Qt::StrongFocus );
	initNetworkSetting(_env, _ppo);
	if(_optimize) {
		mMotionEdited = mController->editMotion(0, true); 
		mIsEditMode = true;
	}
	if(mIsCapture)
		Screenshot();

}
void
ActionControllerWidget::
initNetworkSetting(std::string _env, std::string _ppo) {

    Py_Initialize();
    np::initialize();

    try {
    	if(_ppo != "") {
    		mRunPPO = true;
    		mController = new ENV::ActionController(mEnv, 0, true);

			p::object sys_module = p::import("sys");
			p::str module_dir = (std::string(LAMA_DIR)+"/network").c_str();
			sys_module.attr("path").attr("insert")(1, module_dir);

    		p::object ppo_main = p::import("ppo");
			mPPO = ppo_main.attr("PPO")();
			std::string path = std::string(LAMA_DIR)+ std::string("/network/output/") + _ppo;
			mPPO.attr("initRun")(path,
								 mController->getNumState(), 
								 mController->getNumAction());

			if(mUseVariations) {
				std::string icListPath = std::string(LAMA_DIR) + std::string("/env/env_config/jiyedata/MatPartial/icList.xml");
				mController->setTargetICList(icListPath);
				if(mController->mInitialPosList.size() == 0)
					mController->setInitialPosList();
			}

			runPPO();
			if(mUseVariations) {
				// run until desired motion is made
				mActionRecord.clear();
				mMotionPPO.clear();
				mMotionTargetTrajectory.clear();
				mMotionCurrentTrajectory.clear();
				while(true) {
					runPPO();
					if(mController->getTerminalReason() == 5) {
						printInputPair();
						break;
					}
				}
			}
			mController->loadAutoEncoder();
    	}
    
    } catch (const p::error_already_set&) {
        PyErr_Print();
    }    

}

void
ActionControllerWidget::
runPPO() {
	if(!mUseVariations)
		mController->reset();

	int count = 0;
	auto start = std::chrono::high_resolution_clock::now();

	if(mUseVariations) {
		// for variations, generate variations
		int numICIdx = mController->mTargetICList.size();
		int initialIdx = mController->mInitialPosList.size();

		int randomInitialPosIdx = std::floor(mUniform(mMT) * mController->mInitialPosList.size());
		int randomICIdx = std::floor(mUniform(mMT) * mController->mTargetICList.size());

		std::cout << "num pos idx: " << randomInitialPosIdx << " ic Idx: " << randomICIdx << std::endl;

		// randomly select idx and angle
		double randomAngle = ((mUniform(mMT))*M_PI*2)-M_PI;
		mController->reset(randomICIdx, randomInitialPosIdx, randomAngle);
		std::cout << "done reset" << std::endl;

		// save

		mSuccessInitial = mController->mInitialTargetRecord.back().first;
		mSuccessTargetICList.clear();
		mSuccessTargetICList.push_back(mController->mInitialTargetRecord.back().second);
	}

	while(!mController->isTerminalState()) {
		Eigen::VectorXd state = mController->getState();

		p::object a = mPPO.attr("run")(SIM::toNumPyArray(state));
		np::ndarray na = np::from_object(a);
		Eigen::VectorXd action = SIM::toEigenVector(na, mController->getNumAction());
		// remove action offset (ablation)
		if(mRemoveOffset) {
			Eigen::VectorXd offset(12);
			offset.setZero();
			action.tail(12) = offset;
		}
		mActionRecord.push_back(action);
		mController->setAction(action);
		mController->step();

		count += 1;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	// Print the duration in microseconds
	std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;
	std::cout << "count: " << count << std::endl;

	if(mUseVariations) {
		mSuccessInferenceTime = std::to_string(duration.count());
	}
	std::vector<Eigen::VectorXd> pos;

	for(int i = 0; i <= count; i++) {
		Eigen::VectorXd position = mController->getPosition(i);
		pos.push_back(position);
		// std::cout << "root: " << (position.head<6>()).transpose() << std::endl;
		mMotionTargetTrajectory.push_back(mController->getTargetTrajectory(i));
		mMotionCurrentTrajectory.push_back(mController->getCurrentTrajectory(i));
	}

	mTotalFrame = pos.size()-1;
	std::cout << "mTotalFrame: " << mTotalFrame << std::endl;
	mMotionPPO = pos;

}

void 
ActionControllerWidget:: 
printInputPair()
{
	// print env info
	std::cout << "<Env name=\"env1\">" << std::endl;
	std::cout << "<Scene name=\"Scene1\">" << std::endl;
	
	// print obj info
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		Eigen::VectorXd pos = mEnv->mScene->mObjectList[i].mInitialPos;
		std::cout << "\t<Object name=\"object" << i << "\" file=\"" << mEnv->mScene->mObjectList[i].mFilePath << "\">" << std::endl;
		std::cout << "\t\t<ObjectPosition translation=\"" << pos.segment<3>(3).transpose() << "\" linear=\"" << pos.segment<3>(0).transpose() << "\"/>" << std::endl;
		std::cout << "\t</Object>" << std::endl;
	}
	std::cout << "</Scene>" << std::endl;

	// print initial
	std::cout << "<Initial pos=\"" << mSuccessInitial.pos.transpose() << "\" dir=\"" << mSuccessInitial.dir.transpose() << "\"/>" << std::endl;

	// print IC
	std::cout << "<Cue>" << std::endl;
	std::cout << "\t<ICList num=\"" << mSuccessTargetICList.size() << "\">" << std::endl;
	for(auto ic : mSuccessTargetICList) {
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
}

void
ActionControllerWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
ActionControllerWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
ActionControllerWidget::
setFrame(int n)
{
    // mSkelBVH->setPositions(mMotionBVH[n]);
    // if(mRunPPO)

	if(mIsEditMode) {
		mSkelEdit->setPositions(mMotionEdited[n]);
	} else {
		mTargetTrajectory = mMotionTargetTrajectory[n];
		mCurTrajectory = mMotionCurrentTrajectory[n];	
	}
	mSkelPPO->setPositions(mMotionPPO[n]);

}
void
ActionControllerWidget::
drawSkeletons()
{
	// GUI::drawSkeleton(mSkelBVH, 0);
	// if(mRunPPO)

	if(mIsEditMode) {
		GUI::drawSkeleton(mSkelEdit, 0);
		if(mDrawRef) GUI::drawSkeleton(mSkelPPO, 0);
	}
	else {
		GUI::drawSkeleton(mSkelPPO, 0);	
		GUI::drawTrajectory(mTargetTrajectory, Eigen::Vector3d(0, 0, 1));
		// GUI::drawTrajectory(mCurTrajectory, Eigen::Vector3d(1, 0, 0));
	}
	// draw scene
	int numObjInScene = mController->mEnvScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::drawSkeleton(mController->mEnvScene->mObjectList[i].mObjectSkel, 0, mDrawSceneMeshColor);
	}
}	
void
ActionControllerWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelPPO->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
ActionControllerWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	mCamera->apply();

	// drawGround();
	drawSkeletons();
	
	glColor3f(0.98, 0.51, 0.51);
	if(mDrawActionCue) {
		for(int i = 0; i < mEnv->mInteractionCueList.size(); i++) {
		if(mEnv->mInteractionCueList[i].mType == "sit" || mEnv->mInteractionCueList[i].mType == "stop") {
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
	// GUI::drawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());

	if(mIsCapture)
		Screenshot();

	// draw occupancy grid and path
	// GUI::drawGrid(mEnv->mScene->mGrid, mEnv->mScene->mBBox, 0.2);

}
void
ActionControllerWidget::
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
ActionControllerWidget::
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
ActionControllerWidget::
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
	if(_event->key() == Qt::Key_C) {
		mIsCapture = !mIsCapture;
	}
	if(_event->key() == Qt::Key_M) {
		mDrawSceneMeshColor = !mDrawSceneMeshColor;
	}
	if(_event->key() == Qt::Key_A) {
		mDrawActionCue = !mDrawActionCue;
	}


}
void
ActionControllerWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
ActionControllerWidget::
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
ActionControllerWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
ActionControllerWidget::
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
ActionControllerWidget::
optimize()
{
	mMotionEdited = mController->editMotion(0, true); // later add index
	mIsEditMode = true;
}

void
ActionControllerWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
ActionControllerWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
ActionControllerWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
ActionControllerWidget::
togglePlay() {
	mPlay = !mPlay;
}

void
ActionControllerWidget::
saveOriginalMotion(std::string _filename)
{
	// save
	std::cout << "start saving original motion" << std::endl;
	std::ofstream ofs(_filename);
	ofs << mMotionPPO.size() << std::endl;
	int length = mMotionPPO.size();
	for(int i = 0; i < length; i++) {
		ofs << mMotionPPO[i].transpose() << std::endl;
	}
	ofs.close();
}

void 
ActionControllerWidget::
saveTransitionRecord(std::string _filename)
{
	// save env file name
	// save transition record
	std::cout << "start transition record" << std::endl;
	std::ofstream ofs(_filename);
	std::vector<std::tuple<int,int,std::string,ENV::InteractionCue>> tRecord = mController->mMotionSynthesizer->mTransitionRecord;
	ofs << tRecord.size() << std::endl;
	for(auto t : tRecord) {
		ofs << std::get<0>(t) << " " << std::get<1>(t) << " " << std::get<2>(t) << " ic: " << std::get<3>(t).mType << std::endl;
	}
	// write transition cost
	std::vector<std::pair<int, std::map<std::string, double>>> transitionRecordList = mController->getRecordTransitionCost();

	ofs << "transitionCost " << transitionRecordList.size() << std::endl;
	for(auto p : transitionRecordList) {
		ofs << "frame " << p.first << " transition " << p.second["transition_record"] << " velocity " << p.second["velocity_record"] << std::endl;
	}
	// std::vector<std::map<std::string, double>> transitionRecordList = mController->getRecordTransitionCost();
	// TODO write
	ofs.close();	
}

void
ActionControllerWidget::
saveActionLog(std::string _filename)
{
	// save
	std::cout << "start saving action log" << std::endl;
	std::ofstream ofs(_filename);
	int length = mActionRecord.size();
	ofs << length << " " <<  mActionRecord[0].rows() << std::endl;
	for(int i = 0; i < length; i++) {
		ofs << mActionRecord[i].transpose() << std::endl;
	}
	ofs.close();
}

void 
ActionControllerWidget::
saveOptimizeMotion(std::string _filename)
{
	std::vector<Eigen::VectorXd> toSave;
	if(mMotionEdited.size() == 0) {
		toSave = mMotionPPO;
		std::cout << "warning: original motion will be saved" << std::endl;
	}
	else {
		toSave = mMotionEdited;
	}
	// save edited 
	// std::string path = LAMA_DIR + std::string("/result/") + std::string("test_edited") + std::string(".txt");
	std::cout << "start saving optimized motion" << std::endl;
	std::ofstream ofs(_filename);
	ofs << toSave.size() << std::endl;
	int length = toSave.size();
	for(int i = 0; i < length; i++) {
		ofs << toSave[i].transpose() << std::endl;
	}
	ofs.close();
}

// capture
void 
ActionControllerWidget::
Screenshot() {
   static int count = 0;
  const char directory[29] = "/home/jiye/Pictures/capture";
  const char fileBase[8] = "Capture";
  char fileName[45];

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
