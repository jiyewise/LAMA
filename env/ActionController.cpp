#include "ActionController.h"
#include "Configurations.h"
#include <Eigen/QR>
#include <fstream>
#include <numeric>
#include <algorithm>
#include "EnvConfigurations.h"

namespace ENV
{
ActionController::
ActionController(Environment* _env, int _id, bool _render)
: mCurrentStage(0), mIsEdit(true), mEditBodiesWeight(EDIT_BODY_WEIGHT), 
  mIsParametric(false), mTargetRewardWeight(1.0), mPenetrationRewardWeight(1.0), mRD(), mMT(mRD()), mUniform(0.0, 1.0) // TODO add body weight in config
{
    mRender = _render;
	mEnv = _env;
	mEnvScene = mEnv->mScene;

    std::string skelPath = std::string(LAMA_DIR) + std::string("/data/character/") + CHARACTER_TYPE + std::string(".xml");
    mCharacter = SIM::SkeletonBuilder::buildFromFile(skelPath).first;
    loadSynthesizer();

    if(mIsParametric) {
        // hardcoding
        // std::string icListPath = std::string(LAMA_DIR) + std::string("/env/env_config/jiyedata/MatPartial/icList.xml");
        // std::string icListPath = std::string(LAMA_DIR) + std::string("/env/env_config/jiyedata/N3OpenArea/icList.xml");
        std::string icListPath = std::string(LAMA_DIR) + std::string("/env/env_config/jiyedata/MPH1Library/icList.xml");
        setTargetICList(icListPath);
        setInitialPosList();
    }

    mActionTypeLabels.clear();
    mActionTypeLabels.push_back("walk");
    mActionTypeLabels.push_back("sit");
    mActionTypeLabels.push_back("stop");

    int actionType = mActionTypeLabels.size();
    int walkTarget = 2; // x and z 
    int actionDim = actionType + walkTarget;

    mEditBodies.clear();
    if (mIsEdit) {
		mEditBodies.push_back("RightArm");		
		mEditBodies.push_back("LeftArm");	
		mEditBodies.push_back("RightForeArm");		
		mEditBodies.push_back("LeftForeArm");	                                                                                                                                                                                                                                                                                                                                                                                   
        actionDim += mEditBodies.size() * 3;
    }

    mActions = Eigen::VectorXd::Zero(actionDim); 
    
	mEndEffectors.clear();
	mEndEffectors.push_back(RightToe);
	mEndEffectors.push_back(LeftToe);
	mEndEffectors.push_back(RightHand);
	mEndEffectors.push_back(LeftHand);
	mEndEffectors.push_back(Head);

    mTotalDof = mCharacter->getNumDofs();

    int controlDim = 2; // current stage(1), current action type(1)
    if(mIsParametric)
        controlDim = 2 + 4 + 4; // include target ic pos and dir, target ic relative
    int sceneDim = 2 + pow((mEnvScene->mOccupancyRange),2); // current location in grid coord, near occupancy
    int posDim = (mCharacter->getNumBodyNodes() - 1) * 6;
    int velDim = mTotalDof;
    int eeDim = mEndEffectors.size() * 3;
    int rootDim = 2; // upvec angle and height

    int stateDim = controlDim + sceneDim + posDim + velDim + eeDim + rootDim;

    mStates = Eigen::VectorXd::Zero(stateDim);
    mNumState = mStates.size();
    mNumAction = mActions.size();

    mRewardLabels.clear();
    mRewardLabels.push_back("total");
    mRewardLabels.push_back("target");
    mRewardLabels.push_back("distance");
    mRewardLabels.push_back("penetration");
    if (mIsEdit)
        mRewardLabels.push_back("edit");

    mCurrentFrame = 0;
} 

void 
ActionController::
loadSynthesizer()
{
    std::map<std::string, std::string> databaseMap;
    databaseMap.insert(std::pair<std::string, std::string>(std::string("walk"), std::string(WALK_DATABASE)));
    databaseMap.insert(std::pair<std::string, std::string>(std::string("sit"), std::string(SIT_DATABASE)));

    mMotionSynthesizer = new ENV::MotionSynthesizer(mCharacter, databaseMap);

    // mTargetIC = mEnv->mInteractionCueList;
    for(int i = 0; i < mEnv->mInteractionCueList.size(); i++) {
        mTargetIC.push_back(mEnv->mInteractionCueList[i]);
    }
}

void 
ActionController::
setAction(Eigen::VectorXd _action)
{
	mActions = _action;
}

// parametric
void 
ActionController::
setTargetICList(std::string _icListPath) 
{
    std::vector<InteractionCue> icList = mEnv->parseICList(_icListPath);
    for(auto ic : icList) {
        std::vector<InteractionCue> targetIC;
        targetIC.push_back(ic);
        mTargetICList.push_back(targetIC);
    }
}

void 
ActionController::
setInitialPosList() 
{
    mInitialPosList = mEnv->mScene->uniformSampleInitial();
}


void 
ActionController:: 
reset()
{
    mCurrentStage = 0;
    mCurrentFrame = 0;
    mIsTerminal = false;
    mTerminalReason = 0;
    mPenetrationRewardRecord.clear();

    SIM::Pose2d initial = mEnv->mInitial;

    // for augmented optimization
    if(mIsParametric) {
        // get random pos TODO fix
        int randomInitialPosIdx = std::floor(mUniform(mMT) * mInitialPosList.size());
        if(randomInitialPosIdx >= mInitialPosList.size())
            randomInitialPosIdx = mInitialPosList.size() - 1;
        Eigen::Vector2d randomPos = mInitialPosList[randomInitialPosIdx];

        // set random dir
        double randomAngle = ((mUniform(mMT))*M_PI*2)-M_PI;
        Eigen::Vector2d randomDir = SIM::Transform::rotateDir(SIM::Pose2d(), randomAngle).dir;
        initial.pos = randomPos;
        initial.dir = randomDir;

        // randomly select mTargetIC
        int randomICIdx = std::floor(mUniform(mMT) * mTargetICList.size());
        if(randomICIdx >= mTargetICList.size())
            randomICIdx = mTargetICList.size() - 1;
        mTargetIC = mTargetICList[randomICIdx];

        // save to record (assume one target ic in mTargetIC)
        mInitialTargetRecord.push_back(std::pair<SIM::Pose2d, InteractionCue>(initial, mTargetIC[0]));
    }

    mMotionSynthesizer->initialize(true, initial); 

    auto result = mMotionSynthesizer->getCurrentMotion();
    mCharacter->setPositions(result.first);
    if (mRender) {
        mRecordPosition.clear();
        mRecordActionFeature.clear();
        mRecordTargetTrajectory.clear();
        mRecordCurrentTrajectory.clear();

        mRecordPosition.push_back(mCharacter->getPositions());
        mRecordActionFeature.push_back(mMotionSynthesizer->mCurActionFeature);
        mRecordCurrentTrajectory.push_back(mMotionSynthesizer->getWorldTrajectory(result.second));
        mRecordTargetTrajectory.push_back(mRecordCurrentTrajectory.back());
    }
    updateState();
}

// parametric (custom setting) inference
void
ActionController:: 
reset(SIM::Pose2d _custom_initial)
{
    mCurrentStage = 0;
    mCurrentFrame = 0;
    mIsTerminal = false;
    mTerminalReason = 0;
    mPenetrationRewardRecord.clear();
    mMotionSynthesizer->initialize(true, _custom_initial); 

    // save to record (assume one target ic in mTargetIC)
    mInitialTargetRecord.push_back(std::pair<SIM::Pose2d, InteractionCue>(_custom_initial, mTargetIC[0]));

    auto result = mMotionSynthesizer->getCurrentMotion();
    mCharacter->setPositions(result.first);

    if (mRender) {
        mRecordPosition.clear();
        mRecordActionFeature.clear();
        mRecordTargetTrajectory.clear();
        mRecordCurrentTrajectory.clear();

        mRecordPosition.push_back(mCharacter->getPositions());
        mRecordActionFeature.push_back(mMotionSynthesizer->mCurActionFeature);
        mRecordCurrentTrajectory.push_back(mMotionSynthesizer->getWorldTrajectory(result.second));
        mRecordTargetTrajectory.push_back(mRecordCurrentTrajectory.back());
    }
    updateState();
}

void 
ActionController:: 
reset(int _icIdx, int _initialIdx, double _dirAngle)
{
    mCurrentStage = 0;
    mCurrentFrame = 0;
    mIsTerminal = false;
    mTerminalReason = 0;
    mPenetrationRewardRecord.clear();

    SIM::Pose2d initial = mEnv->mInitial;
    if(mIsParametric) {
        // get random pos TODO fix
        Eigen::Vector2d randomPos = mInitialPosList[_initialIdx];
        // set random dir
        Eigen::Vector2d randomDir = SIM::Transform::rotateDir(SIM::Pose2d(), _dirAngle).dir;
        initial.pos = randomPos;
        initial.dir = randomDir;
        // randomly select mTargetIC
        mTargetIC = mTargetICList[_icIdx];

    }
    // save to record (assume one target ic in mTargetIC)
    mInitialTargetRecord.push_back(std::pair<SIM::Pose2d, InteractionCue>(initial, mTargetIC[0]));

    mMotionSynthesizer->initialize(true, initial); 

    auto result = mMotionSynthesizer->getCurrentMotion();
    mCharacter->setPositions(result.first);
    if (mRender) {
        mRecordPosition.clear();
        mRecordActionFeature.clear();
        mRecordTargetTrajectory.clear();
        mRecordCurrentTrajectory.clear();

        mRecordPosition.push_back(mCharacter->getPositions());
        mRecordActionFeature.push_back(mMotionSynthesizer->mCurActionFeature);
        mRecordCurrentTrajectory.push_back(mMotionSynthesizer->getWorldTrajectory(result.second));
        mRecordTargetTrajectory.push_back(mRecordCurrentTrajectory.back());
    }
    updateState();
}

ActionFeature
ActionController:: 
generateActionFeatureFromAction()
{
    // generate action type and feature from action
    // int actionType = 2;
    // int walkTarget = 2; // x and z -> TODO: local or global?
    // int actionDim = actionType + walkTarget;
    
    int numAction = mActionTypeLabels.size();
    // Eigen::VectorXd prob = mActions.block(0, 0, numAction, 1);
    Eigen::VectorXd prob = mActions.block(0, 0, numAction, 1).cwiseAbs();

    // if(mRender)
    //     std::cout << "prob: " << prob.transpose() << std::endl;

    // softmax
    double max = prob.maxCoeff();

    for(int i = 0; i < numAction; i++) {
        prob[i] = exp(prob[i]-max);
    }

    double sum = prob.sum();
    for(int i = 0; i < numAction; i++) {
        prob[i] = (exp(prob[i] - max) / sum);
        // prob[i] /= sum;
    }

    // cdf
    Eigen::VectorXd probCDF(numAction);
    double cdfSum = 0;
    for(int i = 0; i < numAction; i++) {
        cdfSum += prob[i];
        probCDF[i] = cdfSum;
    }

    // select action 
    double random = dart::math::Random::uniform(0.0, 1.0);
    int selectedAction = probCDF.size()-1;
    for(int i = 0; i < numAction; i++) {
        if (random <= probCDF[i]) {
            selectedAction = i;
            break;
        }
    }

    // initially it should be walk - optional 
    if (mCurrentFrame < INITIAL_WALK_FRAME_COUNT)
        selectedAction = 0;

    // get xz 
    Eigen::VectorXd walkTarget = mActions.block(numAction, 0, 2, 1);
    std::vector<SIM::Pose2d> targetTrajectory = mMotionSynthesizer->generateTargetTrajectory(Eigen::Vector3d(walkTarget[0], 0, walkTarget[1]), true);

    // set pos offset
    // Eigen::VectorXd posOffset(mTotalDof);
    // posOffset.setZero();
    // if (mIsEdit) {
    //     int start = numAction+2;
    //     for(int i = start; i < mActions.rows(); i++) {
    //         mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
    //     }
    //     for(int i = 0; i < mEditBodies.size(); i++) {
    //         int idx = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getIndexInSkeleton(0);
	// 		int dof = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getNumDofs();
	// 		posOffset.block(idx, 0, dof, 1) += mActions.block(start+i*3, 0, dof, 1);
    //     }
    // }

    ActionFeature currentActionFeature = ActionFeature();
    currentActionFeature.targetTrajectory = targetTrajectory;
    currentActionFeature.type = mActionTypeLabels[selectedAction];
    return currentActionFeature;
}

void 
ActionController::
step()
{
    // if (mRender)
    //     std::cout << "-------------------------------------" << std::endl;

    ActionFeature actionFeature = generateActionFeatureFromAction();
    InteractionCue currentTargetIC = mTargetIC[mCurrentStage];
    std::pair<bool,std::map<std::string, double>> transitionRecord = mMotionSynthesizer->step(actionFeature, currentTargetIC);
    auto result = mMotionSynthesizer->getCurrentMotion();
    
    // if edit -> add offset to positions
    Eigen::VectorXd pos = result.first;
    mOriginalPos = result.first;
    if(mIsEdit) {
        int start = mActionTypeLabels.size()+2;
        for(int i = start; i < mActions.rows(); i++) {
            mActions[i] = dart::math::clip(mActions[i]*0.2, -0.7*M_PI, 0.7*M_PI);
        }
        for(int i = 0; i < mEditBodies.size(); i++) {
            int idx = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getIndexInSkeleton(0);
			int dof = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getNumDofs();
			pos.block(idx, 0, dof, 1) += mActions.block(start+i*3, 0, dof, 1) * mEditBodiesWeight;
        }
    }
    
    mCharacter->setPositions(pos);
    mIsActionComplete = mMotionSynthesizer->checkActionComplete();

    // get reward and decide terminal conditions
    updateReward(transitionRecord.second);
	updateTerminalInfo();
	updateState();

    if (mIsActionComplete && !mIsTerminal) {
        mCurrentStage += 1;
        mIsActionComplete = false;
    }

    if(mRender || mIsEdit) {
        if(transitionRecord.first)
            mRecordTransitionCost.push_back(std::pair<int, std::map<std::string, double>>(mCurrentFrame, transitionRecord.second));
        mRecordPosition.push_back(pos);
        if(mRender) {
            std::cout << "current step: " << mCurrentFrame << " action type: " << mMotionSynthesizer->mCurActionFeature.type << std::endl;
            std::cout << "total reward: " << mRewardParts[0] << " target: " << mRewardParts[1] 
                    << " root: " << mRewardParts[2] << " penetration: " << mRewardParts[3] << std::endl;
            std::cout << "terminal: " << mIsTerminal << " terminal reason: " << mTerminalReason << std::endl;
            std::cout << "-------------------------------------" << std::endl;

            mRecordActionFeature.push_back(mMotionSynthesizer->mCurActionFeature);
            mRecordCurrentTrajectory.push_back(mMotionSynthesizer->getWorldTrajectory(result.second));
            mRecordTargetTrajectory.push_back(mMotionSynthesizer->getWorldTrajectory(actionFeature.targetTrajectory));
        }
    }
    
    mCurrentFrame += 1;

}

double 
ActionController:: 
getTargetReward()
{
    InteractionCue currentTargetIC = mTargetIC[mCurrentStage];
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    double sigTarget;
    double rewardTarget = 0;

    double sigAmplify = mEnv->getICDistance(mCurrentStage);
    sigAmplify = dart::math::clip(sigAmplify,(double) TARGET_REWARD_AMPLIFY_MIN, (double) TARGET_REWARD_AMPLIFY_MAX);

    // walk
    if (currentTargetIC.mType == "stop") {
        SIM::Pose2d curRootPos = SIM::Transform::to2d(mCharacter->getRootBodyNode()->getWorldTransform());
        SIM::Pose2d targetRootPos;
        for(auto c : currentTargetIC.mContactList) {
            if (c.jointIdx == 0) targetRootPos = c.contactRootPos;
        }
        Eigen::Vector2d rootDiff = (targetRootPos.pos - curRootPos.pos).cwiseAbs();
        target[0] = rootDiff[0];
        target[2] = rootDiff[1];
        target /= (double) 2;
        sigTarget = 0.3;
        rewardTarget = SIM::expOfSquared(target, sigTarget)*sigAmplify*5;
    }
    // sit
    else if (currentTargetIC.mType == "sit") {
        for(Contact c : currentTargetIC.mContactList) {
            Eigen::Vector3d curTargetPos = mCharacter->getBodyNode(c.jointIdx)->getWorldTransform().translation();
            target += (c.contactPos - curTargetPos).cwiseAbs();
        }
        target /= ((double) currentTargetIC.mContactList.size()*4);
	    sigTarget = 0.3;
        rewardTarget = SIM::expOfSquared(target, sigTarget)*sigAmplify;
    }

    // adjust weight 
    return rewardTarget * mTargetRewardWeight;
}

double
ActionController::
getEditReward(Eigen::VectorXd _original, Eigen::VectorXd _edited, Eigen::VectorXd _prev)
{
	Eigen::VectorXd posDiff = mCharacter->getPositionDifferences(_edited, _original);
	Eigen::VectorXd partPosDiff(mEditBodies.size()*3), regPosDiff(mEditBodies.size()*3);
	for(int i = 0; i < mEditBodies.size(); i++) {
		int idx = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getNumDofs();
		partPosDiff.segment<3>(i*3) = posDiff.segment<3>(idx);
	}
	posDiff = mCharacter->getPositionDifferences(_prev, _edited);
	for(int i = 0; i < mEditBodies.size(); i++) {
		int idx = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getIndexInSkeleton(0);
		int dof = mCharacter->getBodyNode(mEditBodies[i])->getParentJoint()->getNumDofs();
		regPosDiff.segment<3>(i*3) = posDiff.segment<3>(idx);
	}

	// calc reward
	double reward = 0.4*SIM::expOfSquared(partPosDiff, 0.2) + 0.6*SIM::expOfSquared(regPosDiff, 0.2);
	return reward;
}

// double
// ActionController::
// getContactReward(Eigen::VectorXd _original, Eigen::VectorXd _edited)
// {
// 	double diff = 0.0;
//     std::vector<int> originalContact;
//     // first get original contact
//     Eigen::VectorXd p = mCharacter->getPositions();
//     mCharacter->setPositions(_original);
// 	for(int i = 0; i < 2; i++) {
// 		double h;
// 		if(i == 0)
// 			h = mCharacter->getBodyNode(RightFoot)->getWorldTransform().translation()[1] + mCharacter->getBodyNode(RightToe)->getWorldTransform().translation()[1];
// 		else 
// 			h = mCharacter->getBodyNode(LeftFoot)->getWorldTransform().translation()[1] + mCharacter->getBodyNode(LeftToe)->getWorldTransform().translation()[1];
//         h *= 0.5;
//         int isContact = (h < 0.08) ? 1 : 0; 
//         originalContact.push_back(isContact);
// 	}

//     mCharacter->setPositions(_edited);

// 	for(int i = 0; i < 2; i++) {
// 		double h;
// 		if(i == 0)
// 			h = mCharacter->getBodyNode(RightFoot)->getWorldTransform().translation()[1] + mCharacter->getBodyNode(RightToe)->getWorldTransform().translation()[1];
// 		else 
// 			h = mCharacter->getBodyNode(LeftFoot)->getWorldTransform().translation()[1] + mCharacter->getBodyNode(LeftToe)->getWorldTransform().translation()[1];
// 		h *= 0.5;
// 		int contact = originalContact[i];
// 		if(contact == 1 && h > 0.08)
// 			diff += pow(std::max(0.0, h - 0.1)*10, 2);
// 		else if(contact == 0 && h < 0.08)
// 			diff += pow(std::max(0.0, 0.1 - h)*10, 2);
// 	}

// 	double reward = exp(-diff);
//     // backup
//     mCharacter->setPositions(p);
// 	return reward;
// }

void
ActionController::
updateReward(std::map<std::string, double> _transRecord)
{
    InteractionCue currentTargetIC = mTargetIC[mCurrentStage];
    std::string curActionType = mMotionSynthesizer->mCurActionFeature.type;

    double rewardDistance;
    double rewardTarget = 0;
    double rewardPenetration, rewardPenetrationAvg;

    Eigen::Vector3d rootDist = Eigen::Vector3d::Zero();

    // root distance
    Eigen::Vector3d root = mCharacter->getRootBodyNode()->getWorldTransform().translation();
    for(Contact c : currentTargetIC.mContactList) {
        rootDist += (c.contactPos - root).cwiseAbs();
        Eigen::Vector3d curTargetPos = mCharacter->getBodyNode(c.jointIdx)->getWorldTransform().translation();
    }

    rootDist /= ((double) currentTargetIC.mContactList.size()*6);
    rootDist[1] = 0;

	double sigRoot = 0.3; 
    rewardDistance = SIM::expOfSquared(rootDist, sigRoot);
    // if(mRender)
    //     std::cout << "currentTargetIC type: " << currentTargetIC.mType << std::endl;
    if (currentTargetIC.mType == curActionType)
        rewardTarget = getTargetReward();
    
    // transition 
    double transitionReward = exp(-0.17*_transRecord["transition"]);
    double velReward = exp(-0.10*_transRecord["velocity"]);

    // penetration

    // only for penetration ablation
    // rewardPenetration = 1;

    // only for trans and vel ablation
    // transitionReward = 1;
    // velReward = 1;

    std::vector<Eigen::Vector3d> penetratedPoints = computePenetration("Leg");
    std::vector<Eigen::Vector3d> penetratedPointsArm = computePenetration("Arm");
    double numPoints = PENE_LEG_WEIGHT*(double) penetratedPoints.size() + (1-PENE_LEG_WEIGHT)*(double) penetratedPointsArm.size();
    rewardPenetration = exp(-PENE_EXP_WEIGHT*mPenetrationRewardWeight*numPoints);
    mPenetrationRewardRecord.push_back(rewardPenetration);
    rewardPenetrationAvg = checkPenetrationRewardAvg(PENE_FRAME_COUNT);
    
    // root vel
    double rootVelNorm = mMotionSynthesizer->checkRootVel(ROOT_VEL_FRAME_COUNT);
    double rootVelReward = 1;
    if (curActionType == "walk" && rootVelNorm < ROOT_VEL_THRESHOLD) {
        rootVelReward = pow((rootVelNorm*100),3)/100.0;
    }

    // edit
    double editReward = 1;        
    if (mIsEdit) {
        Eigen::VectorXd curPos = mCharacter->getPositions();
		Eigen::VectorXd prevPos = (mRecordPosition.size() == 0) ? curPos : mRecordPosition.back();
        editReward = getEditReward(mOriginalPos, curPos, prevPos);
    }
    
    double rewardTotal = 0.4*(rewardDistance*rewardPenetrationAvg*rootVelReward) + 0.2*editReward + 0.4*rewardTarget*transitionReward*velReward;
    
    mRewardParts.clear();
    mRewardParts.push_back(rewardTotal);
    mRewardParts.push_back(rewardTarget);
    mRewardParts.push_back(rewardDistance);
    mRewardParts.push_back(rewardPenetrationAvg);

    if (mIsEdit)
        mRewardParts.push_back(editReward);

    if (mRender) {
        std::cout << "penetration reward (avg): " << rewardPenetrationAvg << std::endl;
        if (_transRecord.size() > 0) {
            std::cout << "transition reward: " << transitionReward << std::endl;
            std::cout << "velocity reward: " << velReward << std::endl;
        }
    }
}

double
ActionController:: 
checkPenetrationRewardAvg(int _interval) {
    double peneNorm = 0; 
    if(_interval >= mPenetrationRewardRecord.size()) {
        return mPenetrationRewardRecord.back();
    }
    for(int i = 0; i < (_interval); i++) {
        double pene = mPenetrationRewardRecord[mPenetrationRewardRecord.size()-(i+1)];
        peneNorm += pene;
    }
    peneNorm /= (double) _interval;
    // if(mRender)
    //     std::cout << "penetration avg: " << peneNorm << std::endl;
    return peneNorm;
}

std::vector<Eigen::Vector3d> 
ActionController::
computePenetration(std::string _key, double _heightLimit)
{
    // bool isArm = false;
    // if(_key == std::string("Arm"));
    //     isArm = true;
    std::vector<Eigen::Vector3d> penetratedPoints;
    std::vector<dart::dynamics::BodyNode*> bnList;
    for(int i = 0; i < mCharacter->getNumBodyNodes(); i++) {
		auto bn = mCharacter->getBodyNode(i);
		std::string name = bn->getName();
		if (name.find(_key) != std::string::npos) {
            bnList.push_back(bn);
		}
	}
    penetratedPoints = mEnvScene->checkPenetrationByBodyNodeList(bnList, _heightLimit);
    return penetratedPoints;
}

void 
ActionController::
updateTerminalInfo()
{
    std::string curActionType = mMotionSynthesizer->mCurActionFeature.type;
    Eigen::Vector3d rootPos = mCharacter->getRootBodyNode()->getWorldTransform().translation();
    bool isRootOccupied = mEnvScene->checkRootOccupied(Eigen::Vector2d(rootPos[0], rootPos[2]), false); // true is only for var motion quality checking
    double rootVelNorm = 1;
    if(mCurrentFrame < 60) // turn this off only for mp3d data -> add in config
        rootVelNorm = mMotionSynthesizer->checkRootVel(50);

    if (curActionType == "walk" && isRootOccupied) {
        mIsTerminal = true;
        mTerminalReason = 1;
    }
    if (curActionType == "walk" && rootVelNorm < TERMINAL_ROOT_VEL) {
        mIsTerminal = true;
        mTerminalReason = 2;
    }
    else if (checkPenetrationRewardAvg(PENE_FRAME_COUNT) < (TERMINAL_PENETRATION_THRESHOLD)) {
        mIsTerminal = true;
        mTerminalReason = 3;
    }
    // else if (mPenetrationRewardRecord.back() < TERMINAL_PENETRATION_THRESHOLD) {
    //     mIsTerminal = true;
    //     mTerminalReason = 4;
    // }
    else if (mIsActionComplete && mCurrentStage == mTargetIC.size()-1) {
        mIsTerminal = true;
        mTerminalReason = 5;
    }        
}

void 
ActionController::
updateState()
{
    // int controlDim = 2; // current stage(1), current action type(1)
    // int sceneDim = 2 + pow((mScene->mOccupancyRange),2) // current location in grid coord, near occupancy
    // int posDim = (mCharacter->getNumBodyNodes() - 1) * 6;
    // int velDim = mTotalDof;
    // int eeDim = mEndEffectors.size() * 3;

    InteractionCue currentTargetIC = mTargetIC[mCurrentStage];
    SIM::Pose2d icRoot = currentTargetIC.mContactList[0].contactRootPos;
    SIM::Pose2d curRoot = SIM::Transform::to2d(mCharacter->getRootBodyNode()->getWorldTransform());
    SIM::Pose2d relativeTarget = SIM::Transform::getLocalTransform(curRoot, icRoot);

	if (mIsTerminal && mTerminalReason != 2){
		mStates = Eigen::VectorXd::Zero(mNumState);
		return;
	}	

    std::string curActionType = mMotionSynthesizer->mCurActionFeature.type;
    int actionType = 0;
    if (curActionType == "sit") 
        actionType = 1; 
    if (curActionType == "stop") 
        actionType = 2; 

    Eigen::Vector3d rootPos = mCharacter->getRootBodyNode()->getWorldTransform().translation();
    double rootHeight = rootPos[1];
    std::pair<int,int> gridCoord = mEnvScene->getGridCoord(Eigen::Vector2d(rootPos[0], rootPos[2]));
    Eigen::VectorXd occupied = mEnvScene->getGridOccupancy(Eigen::Vector2d(rootPos[0], rootPos[2]));

	Eigen::VectorXd p,v;
	v = mCharacter->getVelocities();

    int posDim = (mCharacter->getNumBodyNodes() - 1) * 6;
	p.resize(posDim);

	for(int i = 1; i < mCharacter->getNumBodyNodes(); i++){
		Eigen::Isometry3d transform = mCharacter->getBodyNode(i)->getRelativeTransform();
		p.segment<6>(6*(i-1)) << transform.linear()(0,0), transform.linear()(0,1), transform.linear()(0,2),
								 transform.linear()(1,0), transform.linear()(1,1), transform.linear()(1,2);
	}

	dart::dynamics::BodyNode* root = mCharacter->getRootBodyNode();
	Eigen::Isometry3d curRootInv = root->getWorldTransform().inverse();
	Eigen::VectorXd ee;
	ee.resize(mEndEffectors.size() * 3);
	for(int i = 0; i < mEndEffectors.size(); i++)
	{
		Eigen::Isometry3d transform = curRootInv * mCharacter->getBodyNode(mEndEffectors[i])->getWorldTransform();
		ee.segment<3>(3*i) << transform.translation();
	}

	Eigen::Vector3d upvec = root->getTransform().linear()*Eigen::Vector3d::UnitY();
	double upvecAngle = atan2(std::sqrt(upvec[0]*upvec[0]+upvec[2]*upvec[2]),upvec[1]);

    if(mIsParametric) {
        mStates << (double) mCurrentStage, (double) actionType, icRoot.pos, icRoot.dir, relativeTarget.pos, relativeTarget.dir, (double) gridCoord.first, (double) gridCoord.second, occupied, p, v, ee, upvecAngle, rootHeight;
    }
    else {
        mStates << (double) mCurrentStage, (double) actionType, (double) gridCoord.first, (double) gridCoord.second, occupied, p, v, ee, upvecAngle, rootHeight;
    }

}

// editing related
void 
ActionController::
loadAutoEncoder()
{
    mMotionSynthesizer->loadAutoEncoder(LAMA_DIR + std::string("/autoencoder/output/") + std::string(AUTOENCODER_NETWORK_NAME));
}

std::vector<Eigen::VectorXd> 
ActionController::
editMotion(int _recordIdx, bool _postProcess)
{
    // Py_Initialize();
    // np::initialize();
    mMotionSynthesizer->optimizeAutoEncoder(_recordIdx);
    mMotionSynthesizer->setEditedMotion(_recordIdx);

    std::vector<Eigen::VectorXd> editedMotion = mMotionSynthesizer->mEditedMotionRecord;
    std::vector<SIM::Frame> postProcessedEditMotion;
    if (_postProcess) {
        int motionLength = editedMotion.size();
        for(int i = 0; i < motionLength; i++) {
            // convert to frames
            Eigen::VectorXd pos = editedMotion[i];
            int dof = pos.rows();
            Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);
            if (i > 2) {
                vel = mMotionSynthesizer->mReferenceManager->computeVelocity(false, pos, postProcessedEditMotion[i-1].position, postProcessedEditMotion[i-2].position);
            } else if (i == 1) {
                vel = mMotionSynthesizer->mReferenceManager->computeVelocity(false, pos, postProcessedEditMotion[0].position);
                postProcessedEditMotion[0].velocity = vel;
            }
            postProcessedEditMotion.push_back(SIM::Frame(pos, vel));
        }
        mMotionSynthesizer->mReferenceManager->footCleanup(postProcessedEditMotion);
        for(int i = 0; i < motionLength; i++) {
            editedMotion[i] = postProcessedEditMotion[i].position;
        }
    }

    return editedMotion;
}


}