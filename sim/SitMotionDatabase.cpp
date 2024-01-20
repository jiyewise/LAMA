#include "SitMotionDatabase.h"
#include "BVHParser.h"
#include "Configurations.h"
#include "Functions.h"
#include <fstream>

namespace SIM
{

SitMotionDatabase::
SitMotionDatabase(dart::dynamics::SkeletonPtr _skel): mRD(), mMT(mRD()), mUniform(0.0, 1.0), mCurIdx(0)
{
	mSkel = _skel;
	for(int i = 0 ; i < mSkel->getNumBodyNodes(); i++) {
		std::string name = mSkel->getBodyNode(i)->getName();
		if(name.find("Foot") != std::string::npos || name.find("Hand") != std::string::npos || name.find("Head") != std::string::npos ) {
			mContactName.push_back(name);
			mContactJoints.push_back(i);
		}
		// target
		if(name.find("Foot") != std::string::npos || name.find("Hip") != std::string::npos) {
			mTargetName.push_back(name);
			mTargetJoints.push_back(i);
		}
	}
	mSearchInterval = 10;
	mLookahead = 3;
	mBlendInterval = 5;
	mFeatureSearchEnd = 50;
	mReferenceManager = new ReferenceManager(_skel);

	mTotalFrames = 0;
}

void
SitMotionDatabase::
loadMotionFromBVH(std::string _bvh)
{
	mReferenceManager->loadMotionFromBVH(_bvh, false);
	std::vector<Frame> motion = mReferenceManager->getMotionGen();

	int motionLength = motion.size();
	int bvhIdx = mSitMotionClipList.size();

	if(motionLength > 85) {
		int offset = motionLength - 85;
		std::vector<Frame> motionClipped;
		for(int i = offset; i < motionLength; i++ ) {
			motionClipped.push_back(motion[i]);
		}
		// std::vector<Frame> motionClipped(motion.begin()+offset, motion.end());
		mSitMotionClipList.push_back(motionClipped);
		generateFeature(bvhIdx, _bvh);		
		mTotalFrames += motionClipped.size();
		return;
	}
	mSitMotionClipList.push_back(motion);
	generateFeature(bvhIdx, _bvh);
	mTotalFrames += motion.size();
}

void 
SitMotionDatabase:: 
generateFeature(int _bvhIdx, std::string _bvh)
{	

	int motionLength = mSitMotionClipList[_bvhIdx].size();
	std::vector<Eigen::Vector3d> prevContactPos;

	for(int i = 0; i < (motionLength - mFeatureSearchEnd); i++) {
		Frame f = mSitMotionClipList[_bvhIdx][i];
		SitMotion sitMotion = SitMotion(f.position, f.velocity);
		sitMotion.originalBVHId = _bvhIdx;
		sitMotion.originalFrameNo = i;
		sitMotion.name = _bvh;

		// upvec
		mSkel->setPositions(f.position);
		Eigen::Isometry3d root = dart::dynamics::FreeJoint::convertToTransform(f.position.head<6>());
		sitMotion.upvec = root.linear()*Eigen::Vector3d::UnitY();
		Pose2d T = Transform::to2d(root);

		// contact info
		for(int j = 0; j < mContactJoints.size(); j++) {
			Eigen::Vector3d translation = mSkel->getBodyNode(mContactJoints[j])->getWorldTransform().translation();
			if(translation(1) < CONTACT_HEIGHT)
				sitMotion.contacts.push_back(1);
			else if(translation(1) > CONTACT_HEIGHT && translation(1) < CONTACT_HEIGHT + 0.02)
				sitMotion.contacts.push_back(0.5);
			else
				sitMotion.contacts.push_back(0);

			Eigen::Vector3d velocity;
			if(i == 0) {
				velocity.setZero();
			} else {
				velocity = (translation - prevContactPos[j]) / NORMALIZED_TIME_STEP;
			}

			Pose2d Tcontact = Pose2d(Transform::to2dPos(translation), Transform::to2dPos(velocity));
			Pose2d transLocal = Transform::getLocalTransform(T, Tcontact);
			
			Eigen::Vector3d transLocal3d = Transform::to3dPos(transLocal.pos);
			transLocal3d(1)= translation(1);
			sitMotion.contactPos.push_back(transLocal3d);

			double velNorm = sqrt(velocity(0) * velocity(0) + velocity(2) * velocity(2));
			Eigen::Vector3d velLocal3d = Transform::to3dPos(transLocal.dir) * velNorm;
			velLocal3d(1) = velocity(1);
			sitMotion.contactVel.push_back(velLocal3d);

			if(i == 0) {
				prevContactPos.push_back(translation);
			} else {
				prevContactPos[j] = translation;
			}
		}

		// target diff
		// Eigen::VectorXd targetPos = (mSitMotionClipList[_bvhIdx])[motionLength-1].position;
		// Eigen::Isometry3d targetRoot = dart::dynamics::FreeJoint::convertToTransform(targetPos.head<6>()); 
		// Pose2d target = Transform::to2d(targetRoot);
		// Pose2d diff = Transform::getLocalTransform(Transform::to2d(root), target);
		// sitMotion.targetPos = diff;

		// target info (root and foot)
		Eigen::VectorXd targetPos = (mSitMotionClipList[_bvhIdx])[motionLength-1].position;
		mSkel->setPositions(targetPos);
		for(int j = 0; j < mTargetJoints.size(); j++) {
			auto bn = mSkel->getBodyNode(mTargetJoints[j]);
			// Eigen::Isometry3d globalPos = mSkel->getBodyNode(mTargetJoints[j])->getWorldTransform();
			Eigen::Isometry3d globalPos = bn->getWorldTransform() * bn->getParentJoint()->getTransformFromChildBodyNode();
			Pose2d targetLocal = Transform::getLocalTransform(Transform::to2d(root), Transform::to2d(globalPos));
			sitMotion.targetPos.push_back(targetLocal);
		}

		// push into mRawMotions
		mFeaturedMotions.push_back(sitMotion);
	}

}


std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>>
SitMotionDatabase::
step(std::vector<SIM::Pose2d> _localTarget, Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	bool success = true;
	std::vector<Eigen::VectorXd> positions;
	std::vector<std::vector<Pose2d>> traj;
	std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>> result = searchNextMotion(_localTarget, _current, _prev);
	if(std::get<1>(result) == false) {
		return std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>>(false, positions, std::get<2>(result));
	}
	std::vector<Frame> next = std::get<0>(result);
	next = mReferenceManager->align(_current, _prev, next, true);
	for(int i = 0; i < next.size(); i++) {
		positions.push_back(next[i].position);
	}
	// std::cout << "sequence length of found clip: " << positions.size() << std::endl;
	return std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>>(success, positions, std::get<2>(result));
}

SitMotion 
SitMotionDatabase::
convertToFeaturedMotion(Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	Eigen::VectorXd vel = mSkel->getPositionDifferences(_current, _prev) / NORMALIZED_TIME_STEP;
    SitMotion curMotion = SitMotion(_current, vel);

    mSkel->setPositions(_current);
    Eigen::Isometry3d root = dart::dynamics::FreeJoint::convertToTransform(_current.head<6>());
    curMotion.upvec = root.linear()*Eigen::Vector3d::UnitY();
	Pose2d T = Transform::to2d(root);

    // get prev contact pos from prev pos
	mSkel->setPositions(_prev);
    std::vector<Eigen::Vector3d> prevContactPos;
    for(int j = 0; j < mContactJoints.size(); j++) {
        Eigen::Vector3d translation = mSkel->getBodyNode(mContactJoints[j])->getWorldTransform().translation();
        prevContactPos.push_back(translation);
    }

    // get features with _current
	mSkel->setPositions(_current);
    for(int j = 0; j < mContactJoints.size(); j++) {
        Eigen::Vector3d translation = mSkel->getBodyNode(mContactJoints[j])->getWorldTransform().translation();
        if(translation(1) < CONTACT_HEIGHT)
            curMotion.contacts.push_back(1);
        else if(translation(1) > CONTACT_HEIGHT && translation(1) < CONTACT_HEIGHT + 0.02)
            curMotion.contacts.push_back(0.5);
        else
            curMotion.contacts.push_back(0);

        Eigen::Vector3d velocity;
        velocity = (translation - prevContactPos[j]) / NORMALIZED_TIME_STEP;

        Pose2d Tcontact = Pose2d(Transform::to2dPos(translation), Transform::to2dPos(velocity));
        Pose2d transLocal = Transform::getLocalTransform(T, Tcontact);
        
        Eigen::Vector3d transLocal3d = Transform::to3dPos(transLocal.pos);
        transLocal3d(1)= translation(1);
        curMotion.contactPos.push_back(transLocal3d);

        double velNorm = sqrt(velocity(0) * velocity(0) + velocity(2) * velocity(2));
        Eigen::Vector3d velLocal3d = Transform::to3dPos(transLocal.dir) * velNorm;
        velLocal3d(1) = velocity(1);
        curMotion.contactVel.push_back(velLocal3d);
    }

	return curMotion;
}

std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>>
SitMotionDatabase::
searchNextMotion(std::vector<SIM::Pose2d> _localTarget, Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	SitMotion curMotion = convertToFeaturedMotion(_current, _prev);

	bool blend = true;
	bool foundMotion = false;
	double costMin = 1e6;
	double idxMin = -1;
	double costEEVel = 0;
	double costFootVelNormalized = 0;
	double transitionCostMin = 1e6;
	double velCostMin = 1e6;
	double transitionCostMinRecord = 1e6;
	double velCostMinRecord = 1e6;

	mTransitionRecord.clear();
	
    int fMotionLength = mFeaturedMotions.size();
    for(int i = 0; i < fMotionLength; i++) 
    {
        // consider upvec and ee/contact
        double costEEPos = 0;
		double costEEVel = 0;
		double costFootContact = 0;
		double costFootVelNormalized = 0;

		for(int j = 0; j < curMotion.contacts.size(); j++) {
			if(mContactName[j].find("Foot") != std::string::npos ){
				costFootContact += pow(mFeaturedMotions[i].contacts[j] - curMotion.contacts[j], 2);
				costEEPos += pow((mFeaturedMotions[i].contactPos[j] - curMotion.contactPos[j]).norm(), 2);
				costEEVel += pow((mFeaturedMotions[i].contactVel[j] - curMotion.contactVel[j]).norm(), 2);
				costFootVelNormalized += pow((mFeaturedMotions[i].contactVel[j].normalized() - curMotion.contactVel[j].normalized()).norm(), 2);

			} else {
				costEEPos += 0.5 * pow((mFeaturedMotions[i].contactPos[j] - curMotion.contactPos[j]).norm(), 2);
				costEEVel += 0.25 * pow((mFeaturedMotions[i].contactVel[j] - curMotion.contactVel[j]).norm(), 2);
			}
		}
		costEEPos /= curMotion.contacts.size();
		costEEVel /= curMotion.contacts.size();
		costFootContact /= 2;
		costFootVelNormalized /= 2;

		double costUpvec = pow((mFeaturedMotions[i].upvec - curMotion.upvec).norm(), 2);
		
		if(costEEPos > 0.1)
			continue;
		if(costFootContact > 0.5)
			continue;
		if(costEEVel > 1.5 || costFootVelNormalized > 1.5)
			continue;

		// target cost
		double costFutureDir = 0.0;
		double costFuturePos = 0.0;
		double costFuturePosRoot = 0.0;
		for(int j = 0; j < mTargetJoints.size(); j++) {
			double costPos = pow((mFeaturedMotions[i].targetPos[j].pos - _localTarget[j].pos).norm(), 2);
			if(mTargetJoints[j] == 0) { // root
				costFutureDir += pow((mFeaturedMotions[i].targetPos[j].dir - _localTarget[j].dir).norm(), 2);
				costFuturePosRoot = costPos;
			}
			costFuturePos += costPos;
		}
		costFuturePos /= mTargetJoints.size();

		if(costFuturePos > 0.2 || costFutureDir > 0.2 || costFuturePosRoot > 0.1) 
			continue;

		double velCost = 5 * costEEVel + 5 * costFootVelNormalized;
		double transitionCost = costUpvec + 5 * costFootContact + 5 * costEEPos + 0.5*velCost;
		double cost = 5 * costFuturePos + 5 * costFutureDir + transitionCost;

		double velCostRecord = costEEVel + 0.5 * costFootVelNormalized;
		double transitionCostRecord = costUpvec + 5 * costFootContact + 10 * costEEPos;

		if(cost < costMin) {
			costMin = cost;
			idxMin = i;
			transitionCostMin = transitionCost;
			velCostMin = velCost;
			transitionCostMinRecord = transitionCostRecord;
			velCostMinRecord = velCostRecord;
		}
    }

	if(idxMin > -1) // search success
		foundMotion = true;

	if(!foundMotion) {
		std::vector<Frame> motion;
		std::vector<std::vector<Pose2d>> trajectory;
		return std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>>(motion, foundMotion, trajectory);
	}


	mTransitionRecord["transition"] = transitionCostMin;
	mTransitionRecord["velocity"] = velCostMin;

	mTransitionRecord["transition_record"] = transitionCostMinRecord;
	mTransitionRecord["velocity_record"] = velCostMinRecord;

	// exit(0);
	// std::cout << __func__ << " transition: " << transitionCostMin << std::endl;
	// std::cout << __func__ << " velocity: " << velCostMin << std::endl;

	// std::cout << "transtion cost: " << transitionCostMin << std::endl;
	// std::cout << "velocity cost: " << velCostMin << std::endl;

	// std::cout << "*******************************" << std::endl;
	// std::cout << mFeaturedMotions[idxMin].name << std::endl;
	// exit(0);
    std::vector<Frame> nextMotionOriginal = mSitMotionClipList[mFeaturedMotions[idxMin].originalBVHId];
	int motionLength = nextMotionOriginal.size();
	std::vector<std::vector<Pose2d>> wholeTrajectory;

	for(int i = 0; i < motionLength; i++) { // compute trajectory here
		Eigen::Isometry3d root = dart::dynamics::FreeJoint::convertToTransform(nextMotionOriginal[i].position.head<6>());
		Pose2d T = Transform::to2d(root);
		std::vector<Pose2d> futurePose;
		
		for(int j = 1; j <= mLookahead; j++) {
			int idx = i + mSearchInterval * j;
			if(idx > motionLength - 1) {
				idx = motionLength - 1;
			} 
			Pose2d Tnext = Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(nextMotionOriginal[idx].position.head<6>()));
			Pose2d delta = Transform::getLocalTransform(T, Tnext);
			futurePose.push_back(delta);
		}
		wholeTrajectory.push_back(futurePose);
	}

    auto last = nextMotionOriginal.cbegin() + nextMotionOriginal.size();
    auto first = nextMotionOriginal.cbegin() + mFeaturedMotions[idxMin].originalFrameNo;
	auto tLast = wholeTrajectory.cbegin() + wholeTrajectory.size();
	auto tFirst = wholeTrajectory.cbegin() + mFeaturedMotions[idxMin].originalFrameNo;
	
	std::vector<Frame> nextMotion(first, last);    
	std::vector<std::vector<Pose2d>> nextTrajectory(tFirst, tLast);

    return std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>>(nextMotion, foundMotion, nextTrajectory);
}

}