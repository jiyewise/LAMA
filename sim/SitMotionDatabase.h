#ifndef __SIM_SIT_MOTION_DATABASE_H__
#define __SIM_SIT_MOTION_DATABASE_H__

#include <string>
#include <Eigen/Core>
#include <vector>
#include "dart/dart.hpp"
#include "SkeletonBuilder.h"
#include "Transform.h"
#include "ReferenceManager.h"

namespace SIM
{
struct SitMotion
{
	SitMotion(Eigen::VectorXd _pos, Eigen::VectorXd _vel) : position(_pos), velocity(_vel) {}
	Eigen::VectorXd position;
	Eigen::VectorXd velocity;
	
	int originalBVHId;
	int originalFrameNo;
	std::string name;

	std::vector<double> contacts;
	std::vector<Eigen::Vector3d> contactPos;
	std::vector<Eigen::Vector3d> contactVel;

	Eigen::Vector3d upvec;

	std::vector<Pose2d> futurePos; // trajectory
	// for sitdown (target is local)
	// Pose2d targetPos; 
	// TODO test: include foot also 
	std::vector<Pose2d> targetPos;
};
class SitMotionDatabase
{
public:
	SitMotionDatabase(dart::dynamics::SkeletonPtr _skel);
	
	std::vector<std::vector<Frame>> mSitMotionClipList;
	ReferenceManager* mReferenceManager;
	
	// for motion matching
	void loadMotionFromBVH(std::string _bvh);
	std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>> step(std::vector<SIM::Pose2d> _localTarget, Eigen::VectorXd _current, Eigen::VectorXd _prev);


	// count
	int mTotalFrames;
	int countFeaturedMotions() { return mFeaturedMotions.size(); }

	// getter
	SitMotion convertToFeaturedMotion(Eigen::VectorXd _current, Eigen::VectorXd _prev);
	std::vector<int> getContactJoints() { return mContactJoints; }
	std::vector<std::string> getContactJointName() { return mContactName; }
	std::vector<int> getTargetJoints() { return mTargetJoints; }
	std::vector<std::string> getTargetJointName() { return mTargetName; }
	std::map<std::string, double> getTransitionRecord() { return mTransitionRecord; }
	double getTransitionRecordByKey(std::string _key) { return mTransitionRecord[_key]; }

private:
	void generateFeature(int _bvhIdx, std::string _bvh);
	std::vector<Eigen::VectorXd> generateCycles(int _length, std::vector<Eigen::VectorXd> _clip);
	std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>> searchNextMotion(std::vector<SIM::Pose2d> _localTarget, Eigen::VectorXd _current, Eigen::VectorXd _prev);

	dart::dynamics::SkeletonPtr mSkel;
	std::map<std::string, double> mTransitionRecord;

	std::vector<SitMotion> mFeaturedMotions;
	std::vector<Frame> mAlignedMotions;
	std::vector<int> mContactJoints;
	std::vector<std::string> mContactName;
	std::vector<std::string> mTargetName;
	std::vector<int> mTargetJoints;

	std::string mCurBVHName;

	int mBlendInterval;
	int mFeatureSearchEnd;
	int mSelectedBVHId;
	int mCurBVHFrameNo;
	int mCurIdx;
	int mSearchInterval;
	int mLookahead;

    // random
    std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;
};
}

#endif