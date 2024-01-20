#include "ManipMotionEditor.h"
#include "Configurations.h"
#include "Functions.h"
#include <fstream>

namespace ENV
{

// constructor
ManipMotionEditor:: 
ManipMotionEditor(dart::dynamics::SkeletonPtr _skel): mSeqLength(120)
{
    mSkel = _skel;
    mReferenceManager = new SIM::ReferenceManager(_skel);
}


void
ManipMotionEditor:: 
loadAutoEncoder(std::string _dir)
{
    mAutoEncoderDir = _dir;
    // Py_Initialize();
    // np::initialize();
    try {
        p::object sys_module = p::import("sys");
        p::str module_dir = (std::string(LAMA_DIR) + "/autoencoder").c_str();
        sys_module.attr("path").attr("insert")(1, module_dir);

        p::object autoencoder_optimizer = p::import("autoencoder_norm_optimize");
        mAutoEncoderOptimize = autoencoder_optimizer.attr("AutoEncoder")(LAMA_DIR + std::string("/autoencoder/"));
    } catch (const p::error_already_set&) {
        PyErr_Print();
    }
}

void 
ManipMotionEditor:: 
setManipPointInMotion(std::vector<Eigen::VectorXd> _motionRecord)
{
    mOriginalMotionRecord = _motionRecord;
}

std::pair<np::ndarray, np::ndarray> 
ManipMotionEditor::
convertToAutoEncoderInput(std::vector<Contact> _contactList, bool _fromEditedMotion)
{
    std::vector<Eigen::VectorXd> motion;
    std::vector<Eigen::VectorXd> targetList;

    Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(mMotionToEdit[0].head<6>());
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();

    SIM::Pose2d T0Local = SIM::Transform::to2d(T0);
	SIM::Pose2d T1Local = SIM::Transform::to2d(T1);

	Eigen::Isometry3d T01 = SIM::Transform::getGlobalTransform(T0Local, T1Local);
	Eigen::Isometry3d T0new = T01 * T0;

    // should normalize this motion too!
    for(int i = 0; i < mMotionToEdit.size(); i++) {
        Eigen::VectorXd pos = mMotionToEdit[i];
        Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;
        
		pos.head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
        Eigen::Vector3d rootPos = pos.segment<3>(3);
        pos.segment<3>(3) = pos.segment<3>(0);
        pos.segment<3>(0) = rootPos;
        motion.push_back(pos);
    }

    // extract target from interaction cue and normalize
    Eigen::Vector3d ori = Eigen::Vector3d::Zero();
    for(Contact c : _contactList) {
        Eigen::Isometry3d targetT = Eigen::Isometry3d::Identity();
        targetT.translation() = c.contactPos;
        targetT = T01 * targetT;
        targetT.translation()[1] = c.contactPos[1];
        int frame = c.frame;   
        Eigen::VectorXd target(1+1+3+3);        
        target << frame, c.jointIdx, targetT.translation(), ori;
        targetList.push_back(target);
        // print
        std::cout << target.transpose() << std::endl;
    }

    return std::pair<np::ndarray,np::ndarray>(SIM::toNumPyArray(motion), SIM::toNumPyArray(targetList));
}

void 
ManipMotionEditor::
optimizeAutoEncoder(int _manipStart, int _manipEnd, std::vector<Contact> _contactList, bool _fromEditedMotion)
{

    if(mOriginalMotionRecord.size() == 0) {
        std::cout << "original motion record is empty. first set original motion record first" << std::endl;
        return;
    }

    std::vector<Eigen::VectorXd> baseMotionRecord = (_fromEditedMotion) ? mManipEditedMotionRecord : mOriginalMotionRecord;
    if(_manipStart >= baseMotionRecord.size())  {
        _manipStart = baseMotionRecord.size()-1;
        _manipEnd = _manipStart;
    }
    mStartManipFrame = _manipStart;
    mEndManipFrame = _manipEnd;

    // Eigen::VectorXd lastFrame = baseMotionRecord[_manipFrame];
    mMotionToEdit.clear();
    // for(int i = 0; i < mSeqLength; i++) {
    //     mMotionToEdit.push_back(lastFrame);
    // }

    int startOffset = _contactList[0].frame-1;
    mEndEditFrame = _contactList.back().frame;

    if(startOffset < 0)
        startOffset = 0;

    // for(int i = 0; i < startOffset; i++) {
    //     mMotionToEdit.push_back(baseMotionRecord[mStartManipFrame]);
    // }
    for(int i = mStartManipFrame; i < mEndManipFrame; i++) {
        mMotionToEdit.push_back(baseMotionRecord[i]);
    }
    mEditedMotionLength = mMotionToEdit.size();
    for(int i = mEditedMotionLength; i < mSeqLength; i++) {
        mMotionToEdit.push_back(baseMotionRecord[mEndManipFrame]);
    }

    // get input 
    std::pair<np::ndarray, np::ndarray> input = convertToAutoEncoderInput(_contactList, _fromEditedMotion);
    bool useRootVel = false;
    int epoch = 500;
    try {
        mAutoEncoderOptimize.attr("init_optimize")(mAutoEncoderDir, input.first, input.second);
        mAutoEncoderOptimize.attr("latent_optimize")(useRootVel, epoch);
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
}

void 
ManipMotionEditor::
setEditedMotion(bool _fromEditedMotion) 
{
    std::vector<Eigen::VectorXd> baseMotionRecord = (_fromEditedMotion) ? mManipEditedMotionRecord : mOriginalMotionRecord;
    std::vector<Eigen::VectorXd> editedMotion;
    std::vector<Eigen::VectorXd> result;
    
    bool isEditClip = false;
    // if(mEndManipFrame < (baseMotionRecord.size()-6)) {
    //     isEditMid = true;
    // }
    if(mEditedMotionLength > mEndEditFrame) 
        isEditClip = true;

    try {
        p::object e = mAutoEncoderOptimize.attr("get_logmap")(0);
        np::ndarray ne = np::from_object(e);
        editedMotion = SIM::toEigenVectorVector(ne);
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }

    // swap
    for(int i = 0; i < mSeqLength; i++) {
        Eigen::VectorXd pos = editedMotion[i];
        Eigen::Vector3d rootPos = pos.segment<3>(3);
        pos.segment<3>(3) = pos.segment<3>(0);
        pos.segment<3>(0) = rootPos;
        editedMotion[i] = pos;
    }

    // cleanup - remove
    // editedMotion = smooth(editedMotion);
    // editedMotion = postProcess(editedMotion);

    // align root to the original motion !!
    Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(editedMotion[0].head<6>());
    Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(baseMotionRecord[mStartManipFrame-1].head<6>());

    SIM::Pose2d T0Local = SIM::Transform::to2d(T0);
	SIM::Pose2d T1Local = SIM::Transform::to2d(T1);

	Eigen::Isometry3d T01 = SIM::Transform::getGlobalTransform(T0Local, T1Local);
	Eigen::Isometry3d T0new = T01 * T0;

    std::cout << "start manip frame: " << mStartManipFrame << std::endl;
    for(int i = 0; i < mStartManipFrame; i++) {
        result.push_back(baseMotionRecord[i]);
    }
    int startFrame = result.size();

    // int offset = 0;
    // if(!isEditMid) {
    //     offset = mEndEditFrame;
    // }
    // int length = mEditedMotionLength + offset;
    // std::cout << "offset: " << offset << " mEditedMotionLength: " << mEditedMotionLength << std::endl;
    int length = (isEditClip) ? mEditedMotionLength : mEndEditFrame;

    for(int i = 0; i < length; i++) {
        Eigen::VectorXd pos = editedMotion[i];
        Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;
		pos.head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);

        // mEditedMotionRecord[i+start] = pos;
        result.push_back(pos);
        // if(i == 0) {
        //     // blend at start
        //     for(int j = 8; j > 0; j--) {
        //         // double weight = 1 - j / (double)(8+1);
        //         double id = j/(double)(8+1);
        //         double weight = 0.5*cos(M_PI*id)+0.5;
        //         Eigen::VectorXd oldPos = result[mStartManipFrame-1+i-j];
        //         result[mStartManipFrame+i-j] = SIM::weightedSumPos(oldPos, pos, weight);
        //     }
        // }
    }

    int endFrame = result.size();

    for(int i = mEndManipFrame; i < baseMotionRecord.size(); i++) {
        result.push_back(baseMotionRecord[i]);
    }

    // blend


    // blend before
    if(startFrame > 10) {
        for(int j = 10; j > 0; j--) {
            // double weight = 1 - j / (double)(10+1);
            double id = j/(double)(10+1);
            double weight = 0.5*cos(M_PI*id)+0.5;
            Eigen::VectorXd oldPos = result[mStartManipFrame-j];
            // std::cout << "j : " << j << " weight: " << weight << std::endl;
            result[startFrame-j] = SIM::weightedSumPos(oldPos, result[startFrame], weight, true);
        }
    }

    // blend after
    // std::cout << "end frame: " << endFrame << std::endl;
    for(int j = 1; j <= 10; j++) {
        double id = j/(double)(10+1);
        double weight = 0.5*cos(M_PI*id)+0.5;
        Eigen::VectorXd target = result[endFrame-1];
        int curIdx = ((endFrame+j-1) < baseMotionRecord.size()) ? (endFrame+j-1) : (baseMotionRecord.size()-1);
        Eigen::VectorXd oldPos = result[curIdx];
        // std::cout << "j : " << j << " weight: " << weight << std::endl;
        result[curIdx] = SIM::weightedSumPos(oldPos,target,weight, true); 
    }

        // for(int j = 8; j > 0; j--) {
        //     // double weight = 1 - j / (double)(8+1);
        //     double id = j/(double)(8+1);
        //     double weight = 0.5*cos(M_PI*id)+0.5;
        //     Eigen::VectorXd oldPos = result[length-j];
        //     std::cout << "j : " << j << " weight: " << weight << std::endl;
        //     result[length-j] = SIM::weightedSumPos(oldPos, result[length], weight);
        // }

    // if(isEditMid) {
    //     // blend and push
    //     if(mEndManipFrame < (baseMotionRecord.size()-8)) {
    //         for(int i = 8; i > 0; i--) {
    //             // double weight = 1-i/(double)(8+1);
    //             double id = i/(double)(8+1);
    //             double weight = 0.5*cos(M_PI*id)+0.5;
    //             Eigen::VectorXd oldPos = result.back();
    //             result.push_back(SIM::weightedSumPos(oldPos, baseMotionRecord[mEndManipFrame], weight));
    //         }
    //     }
    //     for(int i = mEndManipFrame; i < baseMotionRecord.size(); i++) {
    //         result.push_back(baseMotionRecord[i]);
    //     }
    // }

    // blend before and after


    mManipEditedMotionRecord = result;
    // std::cout << __func__ << "ik edited motion size: " << mManipEditedMotionRecord << std::endl;
}


std::vector<Eigen::VectorXd>
ManipMotionEditor:: 
postProcess(std::vector<Eigen::VectorXd> _original)
{
    // change vector of eigen::vectorXd into frames
    int motionLength = _original.size();
    int dof = _original[0].rows();
    std::vector<SIM::Frame> motion;
    std::vector<Eigen::VectorXd> cleaned;

    for(int i = 0; i < motionLength; i++) {
        Eigen::VectorXd pos = _original[i];
        Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);
        if(i >= 2) {
            vel = mReferenceManager->computeVelocity(false, pos, _original[i-1], _original[i-2]);
        } else if(i == 1) {
            vel = mReferenceManager->computeVelocity(false, pos, _original[0]);
            motion[0].velocity = vel;
        }
        motion.push_back(SIM::Frame(pos, vel));
    }
    std::vector<std::vector<bool>> contact = mReferenceManager->extractContact(_original);
    mReferenceManager->footCleanup(motion, contact);
    for(int i = 0; i < motionLength; i++) {
        cleaned.push_back(motion[i].position);
    }
    return cleaned;
}

std::vector<Eigen::VectorXd> 
ManipMotionEditor:: 
smooth(std::vector<Eigen::VectorXd> _original)
{
    std::vector<Eigen::VectorXd> smoothed;
    Eigen::VectorXd smoothedPos; 
    for(int i = 1; i < _original.size(); i++) {
        smoothedPos = SIM::weightedSumPos(_original[i-1], _original[i], 0.5, true);
        smoothed.push_back(smoothedPos);
    }
    smoothed.push_back(_original.back());
    return smoothed;
}


void 
ManipMotionEditor::
editWithIK(int _manipStart, int _manipEnd, std::vector<Contact> _contactList, bool _fromEditedMotion)
{

    if(mOriginalMotionRecord.size() == 0) {
        std::cout << "original motion record is empty. first set original motion record first" << std::endl;
        return;
    }

    std::vector<Eigen::VectorXd> baseMotionRecord = (_fromEditedMotion) ? mManipEditedMotionRecord : mOriginalMotionRecord;
    if(_manipStart >= baseMotionRecord.size())  {
        _manipStart = baseMotionRecord.size()-1;
        _manipEnd = _manipStart;
    }
    mStartManipFrame = _manipStart;
    mEndManipFrame = _manipEnd;

    // Eigen::VectorXd lastFrame = baseMotionRecord[_manipFrame];
    mMotionToEdit.clear();
    // for(int i = 0; i < mSeqLength; i++) {
    //     mMotionToEdit.push_back(lastFrame);
    // }

    bool isEditMid = false;
    if(mEndManipFrame < (baseMotionRecord.size()-6)) {
        isEditMid = true;
    }

    int startOffset = _contactList[0].frame-1;
    mEndEditFrame = _contactList.back().frame;

    if(startOffset < 0)
        startOffset = 0;

    // for(int i = 0; i < startOffset; i++) {
    //     mMotionToEdit.push_back(baseMotionRecord[mStartManipFrame]);
    // }
    for(int i = mStartManipFrame; i < mEndManipFrame; i++) {
        mMotionToEdit.push_back(baseMotionRecord[i]);
    }
    mEditedMotionLength = mMotionToEdit.size();
    for(int i = mEditedMotionLength; i < mSeqLength; i++) {
        mMotionToEdit.push_back(baseMotionRecord[mEndManipFrame]);
    }

    std::map<int, std::vector<std::pair<std::string, Eigen::Vector3d>>> motionConstraint = convertToIKConstraint(_contactList);
    std::vector<Eigen::VectorXd> editedMotion;

    for (auto& pair : motionConstraint) {
        int key = pair.first;
        std::cout << "frame to edit: " << key << std::endl;
        // Do something with the key
    }


    std::vector<std::string> optimBodies;
	optimBodies.push_back("Hips");
	optimBodies.push_back("Spine");
	optimBodies.push_back("Spine1");
	optimBodies.push_back("Spine2");
	optimBodies.push_back("RightShoulder");
	optimBodies.push_back("RightArm");
	optimBodies.push_back("RightForeArm");
	optimBodies.push_back("RightHand");

    // jacobian ik
    for(int i = 0; i < mMotionToEdit.size(); i++) {
        Eigen::VectorXd pos = mMotionToEdit[i];
        mSkel->setPositions(pos);
        Eigen::VectorXd newPos = SIM::solveMCIK(mSkel, motionConstraint[i]);
        editedMotion.push_back(newPos);
    }

    // set to original motion 
    bool isEditClip = false;
    if(mEditedMotionLength > mEndEditFrame) 
        isEditClip = true;

    std::vector<Eigen::VectorXd> result;
    for(int i = 0; i < mStartManipFrame; i++) {
        result.push_back(baseMotionRecord[i]);
    }

    // int offset = 0;
    // if(!isEditMid) {
    //     offset = mEndEditFrame;
    // }
    // int length = mEditedMotionLength + offset;

    int startFrame = result.size();
    int length = (isEditClip) ? mEditedMotionLength : mEndEditFrame;

    // int length = (isEditMid) ? mEditedMotionLength : mEndEditFrame;

    for(int i = 0; i < length; i++) {
        Eigen::VectorXd pos = editedMotion[i];
        result.push_back(pos);
    }
    std::cout << "edited length: " << length << std::endl;

    int endFrame = result.size();
    for(int i = mEndManipFrame; i < baseMotionRecord.size(); i++) {
        result.push_back(baseMotionRecord[i]);
    }

    // blend before
    if(startFrame > 10) {
        for(int j = 10; j > 0; j--) {
            // double weight = 1 - j / (double)(8+1);
            double id = j/(double)(10+1);
            double weight = 0.5*cos(M_PI*id)+0.5;
            Eigen::VectorXd oldPos = result[mStartManipFrame-j];
            // std::cout << "j : " << j << " weight: " << weight << std::endl;
            result[startFrame-j] = SIM::weightedSumPos(oldPos, result[startFrame], weight, true);
        }
    }

    // std::cout << "end frame: " << endFrame << std::endl;
    for(int j = 1; j <= 10; j++) {
        double id = j/(double)(10+1);
        double weight = 0.5*cos(M_PI*id)+0.5;
        Eigen::VectorXd target = result[endFrame-1];
        int curIdx = ((endFrame+j-1) < baseMotionRecord.size()) ? (endFrame+j-1) : (baseMotionRecord.size()-1);
        Eigen::VectorXd oldPos = result[curIdx];
        // std::cout << "j : " << j << " weight: " << weight << std::endl;
        result[curIdx] = SIM::weightedSumPos(oldPos,target,weight, true); 
    }
    
    // if(isEditMid) {
    //     // blend and push
    //     if(mEndManipFrame < (baseMotionRecord.size()-8)) {
    //         for(int i = 8; i > 0; i--) {
    //             // double weight = 1-i/(double)(8+1);
    //             double id = i/(double)(8+1);
    //             double weight = 0.5*cos(M_PI*id)+0.5;
    //             Eigen::VectorXd oldPos = result.back();
    //             result.push_back(SIM::weightedSumPos(oldPos, baseMotionRecord[mEndManipFrame], weight));
    //         }
    //     }
    //     for(int i = mEndManipFrame; i < baseMotionRecord.size(); i++) {
    //         result.push_back(baseMotionRecord[i]);
    //     }
    // }

    mManipEditedMotionRecord = result;    

    // get input 
    // std::pair<np::ndarray, np::ndarray> input = convertToAutoEncoderInput(_contactList, _fromEditedMotion);
    // bool useRootVel = false;
    // int epoch = 300;
    // try {
    //     mAutoEncoderOptimize.attr("init_optimize")(mAutoEncoderDir, input.first, input.second);
    //     mAutoEncoderOptimize.attr("latent_optimize")(useRootVel, epoch);
    // } catch (const p::error_already_set&) {
    //         PyErr_Print();
    // }
}

std::map<int, std::vector<std::pair<std::string, Eigen::Vector3d>>>
ManipMotionEditor:: 
convertToIKConstraint(std::vector<Contact> _contactList)
{
    int startContactJoint, startFrame, endContactJoint, endFrame;
    Eigen::Vector3d startContactPos, endContactPos;
    Contact cStart = _contactList[0];
    Contact cEnd = _contactList.back();

    // assume that joint is fixed
    std::string jointName = mSkel->getBodyNode(_contactList[0].jointIdx)->getName();

    std::map<int, std::vector<std::pair<std::string, Eigen::Vector3d>>> motionConstraint;
    int totalLength = mMotionToEdit.size();
    for(Contact c : _contactList) {
        std::string jointName = mSkel->getBodyNode(c.jointIdx)->getName();
        Eigen::Vector3d pos = c.contactPos;
        std::vector<std::pair<std::string, Eigen::Vector3d>> constraints;
        constraints.push_back(std::pair<std::string, Eigen::Vector3d>(jointName, pos));
        motionConstraint.insert(std::pair<int,std::vector<std::pair<std::string, Eigen::Vector3d>>>(c.frame, constraints));
    }

    // fill in empty constraints 
    // TODO make it generalizable (later)
    int start = 0;
    int end = cStart.frame;

    Eigen::VectorXd initialPos = mMotionToEdit[0];
    mSkel->setPositions(initialPos);
    Eigen::Vector3d initialContactPos = mSkel->getBodyNode(jointName)->getWorldTransform().translation();
    for(int i = start; i < end; i++) {
        int dist = i - start;
        double weight = (dist)/(double)(end-start);
        Eigen::Vector3d interpolatedPos = (1-weight) * initialContactPos + weight*cStart.contactPos;
        std::vector<std::pair<std::string, Eigen::Vector3d>> constraints;
        constraints.push_back(std::pair<std::string, Eigen::Vector3d>(jointName, interpolatedPos));
        motionConstraint.insert(std::pair<int,std::vector<std::pair<std::string, Eigen::Vector3d>>>(i, constraints));
    }

    start = cEnd.frame+1;
    end = mMotionToEdit.size();
    initialContactPos = cEnd.contactPos;
    for(int i = start; i < end; i++) {
        int dist = i - start;
        double weight = (dist)/(double)(end-start);
        Eigen::Vector3d interpolatedPos = (1-weight) * initialContactPos + weight*cEnd.contactPos;
        std::vector<std::pair<std::string, Eigen::Vector3d>> constraints;
        constraints.push_back(std::pair<std::string, Eigen::Vector3d>(jointName, interpolatedPos));
        motionConstraint.insert(std::pair<int,std::vector<std::pair<std::string, Eigen::Vector3d>>>(i, constraints));
    }
    
    return motionConstraint;
}

}