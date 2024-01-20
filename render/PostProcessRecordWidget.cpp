#include <GL/glu.h>
#include <iostream>
#include <Eigen/Geometry>
#include <QSlider>
#include <chrono>
#include <ctime>
#include <vector>
#include <numeric>
#include <boost/filesystem.hpp>
#include "PostProcessRecordWidget.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
#include "dart/external/lodepng/lodepng.h"
PostProcessRecordWidget::
PostProcessRecordWidget()
  :mCamera(new Camera(1000, 650)), mCurFrame(0), mPlay(false), mTrackCamera(false), mIsCapture(false), mShowOriginal(false), mAutoCapture(false)
{
	startTimer(30);
}
PostProcessRecordWidget::
PostProcessRecordWidget(std::string _dir, bool _optimize)
  :PostProcessRecordWidget()
{

	// auto capture
	mPlay = mAutoCapture;

	mScreenshotTemp.resize(4*1920*1080);
	mScreenshotTemp2.resize(4*1920*1080);

	mCurFrame = 0;
	mTotalFrame = 0;
	mDir = _dir + std::string("/");
	mSaveDir = LAMA_DIR + std::string("/result/") + mDir;

	std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    mSkelBVH = SIM::SkeletonBuilder::buildFromFile(skelPath).first;  
    mSkelOptimized = SIM::SkeletonBuilder::buildFromFile(skelPath).first;  
	mReferenceManager = new SIM::ReferenceManager(mSkelBVH->cloneSkeleton());

	if(_optimize)
		std::cout << "-------------------Dir: " << _dir << "-----------------" << std::endl;
    // parse env and motion
    parse(_dir);
    mCurMode = std::string("postprocess");

    calcFootSlipLoss();
	calcPenetration();

	if(_optimize)
		exit(0);

	mTotalFrame = mMotionBVH.size() - 1;
	int numObjInScene = mEnv->mScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::setSkeletonColor(mEnv->mScene->mObjectList[i].mObjectSkel,Eigen::Vector4d(209./255.,209./255.,219./255., 1.0));
	}
	GUI::setSkeletonColor(mSkelBVH, Eigen::Vector4d(83./255.,143./255.,217./255., 1.0));
	setFocusPolicy( Qt::StrongFocus );
}

void 
PostProcessRecordWidget:: 
parse(std::string _dir)
{
    mDir = _dir + std::string("/");
    std::string envPath = LAMA_DIR + std::string("/result/") + _dir + std::string("/env.xml");
    std::string originalPath = LAMA_DIR + std::string("/result/") + _dir + std::string("/original.txt");
    std::string editedPath = LAMA_DIR + std::string("/result/") + _dir + std::string("/optimized.txt");

    mEnv = new ENV::Environment(envPath);
    mEnvScene = mEnv->mScene;

    // parse original motion
    std::ifstream is(originalPath);
	char buffer[256];
	
	is >> buffer;
	int length = atoi(buffer);
	
	int dof = mSkelBVH->getNumDofs();
	Eigen::VectorXd pos(dof);
    std::cout << "original: " << length << std::endl;
	for(int i = 0; i < length; i++)
	{
		for(int j = 0; j < dof; j++) {
			is >> buffer;
			pos(j) = atof(buffer);
		}	
		mMotionOriginal.push_back(pos);
	}
    is.close();

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
		mMotionOptimized.push_back(pos);
	}
    isOpt.close();
	mMotionBVH = mMotionOptimized;
	mContacts = mReferenceManager->extractContact(mMotionOptimized);
	mLeftFoot = mReferenceManager->getLeftFoot();
    mRightFoot = mReferenceManager->getRightFoot();

}

void 
PostProcessRecordWidget:: 
calcFootSlipLoss()
{
    int motionLength = mMotionOptimized.size();
    for(int i = 0; i <= 1; i++) {
        std::map<std::string, int> footJoints = i ? mLeftFoot : mRightFoot;
        std::vector<bool> contacts = mContacts[i];
        int frameStart = 0;
        std::vector<double> footSlip;
        for(int i = 1; i < motionLength; i++) 
		{
			bool prevCon = contacts[i-1];
			bool curCon = contacts[i];

			if(prevCon && !curCon) {
				int mid = (frameStart + i)/2;
				int start = std::min(mid, frameStart);
				int end = std::max(mid, i);
				if(start != end) {
                    mSkelBVH->setPositions(mMotionOptimized[start]);
                    Eigen::Vector3d startFootPos = mSkelBVH->getBodyNode(footJoints["Foot"])->getWorldTransform().translation();
                    for(int j = start+1; j < end; j++) {
                        mSkelBVH->setPositions(mMotionOptimized[j]);
                        Eigen::Vector3d footPos = mSkelBVH->getBodyNode(footJoints["Foot"])->getWorldTransform().translation();                        
                        footSlip.push_back((footPos-startFootPos).norm());
                    }
				}
			} else if(!prevCon && curCon) {
				frameStart = i;
			}
		}
        mFootSlip.push_back(footSlip);
    }

    std::vector<double> avgFootSlipLoss;
    for(int i = 0; i <=1; i++) {
        std::vector<double> v = mFootSlip[i];
        double avg = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        avgFootSlipLoss.push_back(avg);
    }   

    // save in file
    std::string filePath = LAMA_DIR + std::string("/result/") + mDir + std::string("footslip.txt");
    std::ofstream ofs(filePath); 
    ofs << "Total: " << (avgFootSlipLoss[0] + avgFootSlipLoss[1])*0.5 << " Right Avg: " << avgFootSlipLoss[0] << " Left Avg: " << avgFootSlipLoss[1] << std::endl;
    for(int i = 0; i <= 1; i++) {
        std::vector<double> fs = mFootSlip[i];
        for(int j = 0; j < fs.size(); j++) {
            ofs << fs[j] << " ";
        }
        ofs << std::endl;
    }
    ofs.close();
    std::cout << "Total: " << (avgFootSlipLoss[0] + avgFootSlipLoss[1])*0.5 << " Right Avg: " << avgFootSlipLoss[0] << " Left Avg: " << avgFootSlipLoss[1] << std::endl;

}

std::vector<Eigen::Vector3d> 
PostProcessRecordWidget:: 
computePenetration(std::string _key, double _heightLimit)
{
	std::vector<Eigen::Vector3d> penetratedPoints;
    std::vector<dart::dynamics::BodyNode*> bnList;
    for(int i = 0; i < mSkelBVH->getNumBodyNodes(); i++) {
		auto bn = mSkelBVH->getBodyNode(i);
		std::string name = bn->getName();
		if (name.find(_key) != std::string::npos) {
            bnList.push_back(bn);
		}
	}
    penetratedPoints = mEnvScene->checkPenetrationByBodyNodeList(bnList, _heightLimit);
    return penetratedPoints;
}

void 
PostProcessRecordWidget:: 
calcPenetration()
{

	std::vector<Eigen::Vector3d> penetratedPoints;
	std::vector<Eigen::Vector3d> penetratedPointsArm;

	int legCount = 0;
	int armCount = 0;
	int bothCount = 0;
	for(int i = 0; i < mMotionOptimized.size(); i++) {
        mSkelBVH->setPositions(mMotionOptimized[i]);
        penetratedPoints = computePenetration("Leg", 0.1);
        penetratedPointsArm = computePenetration("Arm", 0.1);

		int legPoint = penetratedPoints.size();
		int armPoint = penetratedPointsArm.size();
		if(legPoint >= 10)
			legCount += 1;
		if(armPoint >= 7)
			armCount += 1;
		if(legPoint >= 10 && armPoint >= 7)
			bothCount += 1;
		mPenetrationPointCount.push_back(std::pair<int, int>(legPoint, armPoint));
    }

	std::string filePath = LAMA_DIR + std::string("/result/") + mDir + std::string("penetration.txt");
    std::ofstream ofs(filePath); 
    ofs << "Frame, Leg, Arm, Total" << std::endl;
	ofs << mMotionOptimized.size() << " " << legCount << " " << armCount << " " << (legCount+armCount)-bothCount << std::endl;
    for(int i = 0; i <= mMotionOptimized.size(); i++) {
        std::pair<int,int> points = mPenetrationPointCount[i];
		ofs << i << " " << points.first << " " << points.second << std::endl;
    }
    ofs.close();
	std::cout << mMotionOptimized.size() << " " << legCount << " " << armCount << " " << (legCount+armCount)-bothCount << std::endl;

}

void 
PostProcessRecordWidget::
drawRootOccupancy()
{

	Eigen::Vector3d rootPos = mMotionBVH[mCurFrame].segment<3>(3);
	std::pair<int, int> gridCoord = mEnvScene->getGridCoord(Eigen::Vector2d(rootPos[0], rootPos[2]));
	int numLines = mEnvScene->mOccupancyRange + 1;
	int offset = mEnvScene->mOccupancyRange/2;
	glColor3f(0,0,0);
	glLineWidth(2);
	for(int i = 0; i < numLines; i++) {
		double xStart = gridCoord.first - offset;
		double xEnd = gridCoord.first + offset;
		double zStart = gridCoord.second - offset + i;
		double x_start = mEnvScene->mBBox(0,0) + xStart*mEnvScene->mGridSizeX;
		double x_end = mEnvScene->mBBox(0,0) + (xEnd+1)*mEnvScene->mGridSizeX;
		double z = mEnvScene->mBBox(0,2) + zStart*mEnvScene->mGridSizeZ;
		glBegin(GL_LINES);
		glVertex3f(x_start, 0.02, z);
		glVertex3f(x_end, 0.02, z);
		glEnd();
	}
	for(int i = 0; i < numLines; i++) {
		double zStart = gridCoord.second - offset;
		double zEnd = gridCoord.second + offset;
		double xStart = gridCoord.first - offset + i;
		double z_start = mEnvScene->mBBox(0,2) + zStart*mEnvScene->mGridSizeZ;
		double z_end = mEnvScene->mBBox(0,2) + (zEnd+1)*mEnvScene->mGridSizeZ;
		double x = mEnvScene->mBBox(0,0) + xStart*mEnvScene->mGridSizeX;
		glBegin(GL_LINES);
		glVertex3f(x, 0.02, z_start);
		glVertex3f(x, 0.02, z_end);
		glEnd();
	}

	// draw occupied grid
	double x_base, z_base;
	for(int i = 0; i < mEnvScene->mOccupancyRange; i++) {
		for(int j = 0; j < mEnvScene->mOccupancyRange; j++) { 
			int newX = gridCoord.first - offset + i;
			int newZ = gridCoord.second - offset + j;
			x_base = mEnvScene->mBBox(0,0) + newX*mEnvScene->mGridSizeX;
			z_base = mEnvScene->mBBox(0,2) + newZ*mEnvScene->mGridSizeZ;
			if(mEnvScene->mGrid(newX, newZ) > 0.5 || ((newX == gridCoord.first)&&(newZ == gridCoord.second))) {
				if(((newX == gridCoord.first)&&(newZ == gridCoord.second)))
					glColor3f(0,0,1);
				else 
					glColor3f(0,0,0);
				glBegin(GL_QUADS);
					glVertex3f(x_base, 0.02, z_base);
					glVertex3f(x_base, 0.02, z_base + mEnvScene->mGridSizeZ);
					glVertex3f(x_base + mEnvScene->mGridSizeX, 0.02, z_base + mEnvScene->mGridSizeZ);
					glVertex3f(x_base + mEnvScene->mGridSizeX, 0.02, z_base);
				glEnd();
			}
		} 
	}
}



void
PostProcessRecordWidget::
initializeGL()
{
	glClearColor(1,1,1,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	mCamera->apply();
}
void
PostProcessRecordWidget::
resizeGL(int w,int h)
{
	glViewport(0, 0, w, h);
	mCamera->setSize(w, h);
	mCamera->apply();
}
void
PostProcessRecordWidget::
setFrame(int n)
{
    Eigen::VectorXd pos, pos2;
	pos = (mShowOriginal) ? mMotionOriginal[n] : mMotionOptimized[n];
    mSkelBVH->setPositions(pos);
}

void 
PostProcessRecordWidget::
drawContact()
{
    glColor3f(0,1,0);
    for(int i = 0; i < 2; i++) {
        std::map<std::string, int> footJoints = i ? mLeftFoot : mRightFoot;
        std::vector<bool> contact = mContacts[i];
        bool isContact = contact[mCurFrame];
        if(isContact) {
            Eigen::Vector3d footPos = mSkelBVH->getBodyNode(footJoints["Foot"])->getWorldTransform().translation();
            glPushMatrix();
            glTranslatef(footPos[0], footPos[1], footPos[2]);
            GUI::drawSphere(0.07);
            glPopMatrix();
        }
    }
}

void
PostProcessRecordWidget::
drawSkeletons()
{
    GUI::drawSkeleton(mSkelBVH, 0);
	int numObjInScene = mEnvScene->mObjectList.size();
	for(int i = 0; i < numObjInScene; i++) {
		GUI::drawSkeleton(mEnvScene->mObjectList[i].mObjectSkel, 0, true);
	}

}	
void
PostProcessRecordWidget::
drawGround()
{
	Eigen::Vector3d comRoot;
	comRoot = mSkelBVH->getRootBodyNode()->getCOM();
	GUI::drawGround((int)comRoot[0], (int)comRoot[2], 0);
}
void
PostProcessRecordWidget::
paintGL()
{	
	glClearColor(1.0, 1.0, 1.0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	initLights();
	glEnable(GL_LIGHTING);

	for(auto tuple : mCameraList) {
		if(mSaveDir.find(std::get<0>(tuple)) != std::string::npos) {
			Eigen::Vector3d lookAt = std::get<1>(tuple);
			Eigen::Vector3d eye = std::get<2>(tuple); 
			Eigen::Vector3d up = mCamera->getUp();
			mCamera->setCamera(lookAt, eye, up);
		}
	}
	mCamera->apply();

	drawSkeletons();
	if(mAutoCapture && mPlay) 
		Screenshot();
	
	if(mIsCapture)
		Screenshot();
}
void
PostProcessRecordWidget::
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
PostProcessRecordWidget::
timerEvent(QTimerEvent* _event)
{
	if(mAutoCapture && mCurFrame >= mTotalFrame) {
		exit(0);
	}
	if(mPlay && mCurFrame < mTotalFrame) {
		mCurFrame += 1;
	} 
	setFrame(mCurFrame);
	update();

}
void
PostProcessRecordWidget::
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
	if(_event->key() == Qt::Key_O) {
		mShowOriginal = !mShowOriginal;
	}

}
void
PostProcessRecordWidget::
mousePressEvent(QMouseEvent* _event)
{
	mIsDrag = true;
	mButton = _event->button();
	mPrevX = _event->x();
	mPrevY = _event->y();
}
void
PostProcessRecordWidget::
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
PostProcessRecordWidget::
mouseReleaseEvent(QMouseEvent* _event)
{
	mIsDrag = false;
	mButton = Qt::NoButton;
	update();
}
void
PostProcessRecordWidget::
wheelEvent(QWheelEvent* _event)
{
	if(_event->angleDelta().y() > 0)
		mCamera->pan(0, -5, 0, 0);
	else
		mCamera->pan(0, 5, 0, 0);
	update();
}
void
PostProcessRecordWidget::
nextFrame()
{ 
	if(!mPlay) {
		mCurFrame += 1;
		setFrame(mCurFrame);
	}
}
void
PostProcessRecordWidget::
prevFrame()
{
	if(!mPlay && mCurFrame > 0) {
		mCurFrame -= 1;
		setFrame(mCurFrame);
	}
}
void
PostProcessRecordWidget::
reset()
{
	mCurFrame = 0;
	setFrame(mCurFrame);
}
void 
PostProcessRecordWidget::
togglePlay() {
	mPlay = !mPlay;
}

void 
PostProcessRecordWidget::
Screenshot() {
  std::string capdir = mSaveDir + std::string("/capture");
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
