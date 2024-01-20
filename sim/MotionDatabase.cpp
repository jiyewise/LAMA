#include "MotionDatabase.h"
#include "BVHParser.h"
#include "Configurations.h"
#include "Functions.h"
#include <fstream>

namespace SIM
{
double TIME_STEP = NORMALIZED_TIME_STEP;

MotionDatabase::
MotionDatabase(dart::dynamics::SkeletonPtr _skel): mRD(), mMT(mRD()), mUniform(0.0, 1.0)
{
	mSkel = _skel;
	mReferenceManager = new ReferenceManager(_skel);
	for(int i = 0 ; i < mSkel->getNumBodyNodes(); i++) {
		std::string name = mSkel->getBodyNode(i)->getName();
		if(name.find("Foot") != std::string::npos || name.find("Hand") != std::string::npos || name.find("Head") != std::string::npos ) {
			mContactName.push_back(name);
			mContactJoints.push_back(i);
		}
	}
	mSearchInterval = 10;
	mBlendInterval = 5;
	mLookahead = 3;

	mTotalFrames = 0;
}

void
MotionDatabase::
loadMotionFromBVH(std::string _bvh, bool _isCyclic)
{
	int start = mRawMotions.size();
	mReferenceManager->loadMotionFromBVH(_bvh, false);
	std::vector<Frame> motion = mReferenceManager->getMotionGen();
	int motionLength = motion.size();
	for(int i = 0; i < motionLength; i++) {
		Eigen::VectorXd pos = motion[i].position;
		Eigen::VectorXd vel = motion[i].velocity;
		mRawMotions.push_back(Motion(pos, vel));
	}

	int end = mRawMotions.size() - 1;
	generateFeature(start, end, _isCyclic, _bvh);

}

void 
MotionDatabase:: 
generateFeature(int _start, int _end, bool _isCyclic, std::string _bvh)
{	
	std::vector<Eigen::VectorXd> pos;
	for(int i =_start; i<= _end; i++) {
		pos.push_back(mRawMotions[i].position);
	}

	pos = generateCycles(_end - _start + mLookahead * mSearchInterval, pos);

	mTotalFrames += pos.size();

	std::vector<Eigen::Vector3d> prevContactPos;

	bool print = false;
	for(int i = _start; i <= _end; i++) {
		mRawMotions[i].id = i;
		mRawMotions[i].start = _start;
		mRawMotions[i].end = _end;
		mRawMotions[i].isCyclic = _isCyclic;
		mRawMotions[i].name = _bvh;
		mRawMotions[i].originalId = i - _start;

		bool isStart = false;
		if(i == _start)
			isStart = true;

		mSkel->setPositions(mRawMotions[i].position);

		Eigen::Isometry3d root = dart::dynamics::FreeJoint::convertToTransform(mRawMotions[i].position.head<6>());
		mRawMotions[i].upvec = root.linear()*Eigen::Vector3d::UnitY();

		Pose2d T = Transform::to2d(root);

		for(int j = 0; j < mContactJoints.size(); j++) {
			Eigen::Vector3d translation = mSkel->getBodyNode(mContactJoints[j])->getWorldTransform().translation();
			if(translation(1) < CONTACT_HEIGHT)
				mRawMotions[i].contacts.push_back(1);
			else if(translation(1) > CONTACT_HEIGHT && translation(1) < CONTACT_HEIGHT + 0.02)
				mRawMotions[i].contacts.push_back(0.5);
			else
				mRawMotions[i].contacts.push_back(0);

			Eigen::Vector3d velocity;
			if(i == _start) {
				velocity.setZero();
			} else {
				velocity = (translation - prevContactPos[j]) / TIME_STEP;
			}

			Pose2d Tcontact = Pose2d(Transform::to2dPos(translation), Transform::to2dPos(velocity));
			Pose2d transLocal = Transform::getLocalTransform(T, Tcontact);
			
			Eigen::Vector3d transLocal3d = Transform::to3dPos(transLocal.pos);
			transLocal3d(1)= translation(1);
			mRawMotions[i].contactPos.push_back(transLocal3d);

			double velNorm = sqrt(velocity(0) * velocity(0) + velocity(2) * velocity(2));
			Eigen::Vector3d velLocal3d = Transform::to3dPos(transLocal.dir) * velNorm;
			velLocal3d(1) = velocity(1);
			mRawMotions[i].contactVel.push_back(velLocal3d);

			if(i == _start) {
				prevContactPos.push_back(translation);
			} else {
				prevContactPos[j] = translation;
			}
		}

		for(int j = 1; j <= mLookahead; j++) {

			int idx = -_start + i + mSearchInterval * j;
			if(idx > pos.size() - 1) {
				idx = pos.size() - 1;
			} 

			Pose2d Tnext = Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(pos[idx].head<6>()));
			Pose2d delta = Transform::getLocalTransform(T, Tnext);

			mRawMotions[i].futurePose.push_back(delta);
			// if(j == mLookahead) {
			// 	if(delta.pos.norm() < 0.1) {
			// 		std::cout << i << std::endl;
			// 	}
			// }
		}
	}
	mRawMotions[_start].contactVel = mRawMotions[_start+1].contactVel;
}
std::vector<Eigen::VectorXd> 
MotionDatabase::
generateCycles(int _length, std::vector<Eigen::VectorXd> _clip)
{
	int idx = 0;
	int length = _clip.size();
	std::vector<Eigen::VectorXd> result;
	for(int i = 0; i < _clip.size(); i++) {
		result.push_back(_clip[i]);
	}
	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_clip[0].head<6>());
	Eigen::Isometry3d T0new;
	while(result.size() < _length) {
		int phase = idx % length;

		if(phase == 0) {
			Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(result.back().head<6>());

			Pose2d T0Local =Transform::to2d(T0);
			Pose2d T1Local = Transform::to2d(T1);

			Eigen::Isometry3d T01 = Transform::getGlobalTransform(T0Local, T1Local);
			T0new = T01 * T0;	
		}

		Eigen::VectorXd pos = _clip[phase];
		Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;

		pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(TCurrent);

		result.push_back(pos);
		idx += 1;
	}
	return result;
}

std::pair<std::vector<Eigen::VectorXd>,std::vector<std::vector<Pose2d>>>
MotionDatabase::
initialize(bool _isCustom, Pose2d _startPose)
{
	std::vector<Eigen::VectorXd> initialMotion;
	std::vector<std::vector<Pose2d>> initialTrajectory;
	int startIdx = 0;
	for(int i = 0; i <= mSearchInterval; i++) {
		if((i+startIdx) > mRawMotions[startIdx].end)
			break;
		initialMotion.push_back(mRawMotions[i+startIdx].position);
		initialTrajectory.push_back(mRawMotions[i+startIdx].futurePose);
	}
	// if custom starting pose is given, align
	if(_isCustom) {
		Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(initialMotion[0].head<6>());
		SIM::Pose2d T0Local = SIM::Transform::to2d(T0);
		Eigen::Isometry3d T0Updated = SIM::Transform::getGlobalTransform(T0Local, _startPose) * T0;

		for(int i = 0; i <= mSearchInterval; i++) {
			Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(initialMotion[i].head<6>());
			TCurrent = T0.inverse() * TCurrent;
			TCurrent = T0Updated * TCurrent;
			initialMotion[i].head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
		}
	}	
	return std::pair<std::vector<Eigen::VectorXd>,std::vector<std::vector<Pose2d>>>(initialMotion, initialTrajectory);
}

std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>>
MotionDatabase::
step(std::vector<Pose2d> _target, Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	bool success = true;
	std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>> result = searchNextMotion(_target, _current, _prev);
	std::vector<Frame> next = std::get<0>(result);
	bool blend = std::get<1>(result);

	int size = mAlignedMotions.size();
	next = mReferenceManager->align(_current, _prev, next, blend);
	std::vector<Eigen::VectorXd> positions;
	for(int i = 0; i < next.size(); i++) {
		positions.push_back(next[i].position);
	}
	return std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>>(success, positions, std::get<2>(result));
}

Motion 
MotionDatabase::
convertToFeaturedMotion(Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	Eigen::VectorXd vel = mSkel->getPositionDifferences(_current, _prev) / NORMALIZED_TIME_STEP;
    Motion curMotion = Motion(_current, vel);

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
MotionDatabase::
searchNextMotion(std::vector<Pose2d> _target, Eigen::VectorXd _current, Eigen::VectorXd _prev)
{
	Motion curMotion = convertToFeaturedMotion(_current, _prev);
	std::vector<Frame> motions;
	std::vector<std::vector<Pose2d>> trajectories;

	bool blend = true;

	double costMin = 1e6;
	double transitionCostMin = 1e6;
	double velCostMin = 1e6;
	double idxMin = -1;
	mTransitionRecord.clear();

	for(int i = 0; i < mRawMotions.size(); i++) {
		// if(mRawMotions[i].end - i < 10 || i == _current.id)
		// 	continue;

		if(mRawMotions[i].end - i < 10)
			continue;

		double costFutureDir = 0;
		double costFuturePos = 0;
	
		for(int j = 0 ; j < mLookahead; j++) {
			double weight = 2 * (j + 1.0) / mLookahead;
			costFuturePos += weight * pow((mRawMotions[i].futurePose[j].pos - _target[j].pos).norm(), 2);
			costFutureDir += weight * pow((mRawMotions[i].futurePose[j].dir - _target[j].dir).norm(), 2);
		}

		costFuturePos /= mLookahead;
		costFuturePos = std::min(costFuturePos, 20.0);
		costFutureDir /= mLookahead;

		double costEEPos = 0;
		double costEEVel = 0;
		double costFootContact = 0;
		double costFootVelNormalized = 0;

		for(int j = 0; j < curMotion.contacts.size(); j++) {
			if(mContactName[j].find("Foot") != std::string::npos ){
				costFootContact += pow(mRawMotions[i].contacts[j] - curMotion.contacts[j], 2);
				costEEPos += pow((mRawMotions[i].contactPos[j] - curMotion.contactPos[j]).norm(), 2);
				costEEVel += pow((mRawMotions[i].contactVel[j] - curMotion.contactVel[j]).norm(), 2);
				costFootVelNormalized += pow((mRawMotions[i].contactVel[j].normalized() - curMotion.contactVel[j].normalized()).norm(), 2);

			} else {
				costEEPos += 0.5 * pow((mRawMotions[i].contactPos[j] - curMotion.contactPos[j]).norm(), 2);
				costEEVel += 0.25 * pow((mRawMotions[i].contactVel[j] - curMotion.contactVel[j]).norm(), 2);
			}
		}
		costEEPos /= curMotion.contacts.size();
		costEEVel /= curMotion.contacts.size();

		costFootContact /= 2;
		costFootVelNormalized /= 2;

		double costUpvec = pow((mRawMotions[i].upvec - curMotion.upvec).norm(), 2);
		double velCost = costEEVel + 0.5 * costFootVelNormalized;
		double transitionCost = costUpvec + 5 * costFootContact + 10 * costEEPos;
		double cost = costFuturePos + 2 * costFutureDir + transitionCost + velCost;
		
		if(costEEPos > 0.1)
			continue;
		if(costFootContact > 0.5)
			continue;

		if(cost < costMin) {
			costMin = cost;
			idxMin = i;
			transitionCostMin = transitionCost;
			velCostMin = velCost;
		}
	}
	
	// if(idxMin == _current.id || idxMin == _current.id + 1) {
	// 	blend = false;
	// } // TODO think about holding id with Frame-based notations

	// if(transitionCostMin <= 0.05 && velCostMin <= 0.05) {
	// 	blend = false;
	// } 

	int count = 0;
	int id = idxMin;

	mTransitionRecord["transition"] = transitionCostMin;
	mTransitionRecord["velocity"] = velCostMin;
	mTransitionRecord["transition_record"] = transitionCostMin;
	mTransitionRecord["velocity_record"] = velCostMin;

	while(count < mSearchInterval) {
		if(mRawMotions[id].end == id) {
			if(count == 0) {
				id = mRawMotions[id].start;
			} else {
				break;
			}
		} else {
			id += 1;
		}
		count += 1;
		Frame f = Frame(mRawMotions[id].position, mRawMotions[id].velocity);
		motions.push_back(f);
		trajectories.push_back(mRawMotions[id].futurePose);
		// motions.push_back(mRawMotions[id]);
	}

	return std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>> (motions, blend, trajectories);
}
}