#ifndef __SIM_MOTION_DATABASE_H__
#define __SIM_MOTION_DATABASE_H__

#include <string>
#include <Eigen/Core>
#include <vector>
#include "dart/dart.hpp"
#include "SkeletonBuilder.h"
#include "Transform.h"
#include "ReferenceManager.h"
namespace SIM
{
struct Motion
{
	Motion(Eigen::VectorXd _pos, Eigen::VectorXd _vel) : position(_pos), velocity(_vel) {}
	Eigen::VectorXd position;
	Eigen::VectorXd velocity;
	
	int id;
	int start;
	int end;

	int originalId;
	std::string name;

	bool isCyclic;

	std::vector<double> contacts;
	std::vector<Eigen::Vector3d> contactPos;
	std::vector<Eigen::Vector3d> contactVel;

	std::vector<Pose2d> futurePose;
	Eigen::Vector3d upvec;

};
class MotionDatabase
{
public:
	MotionDatabase(dart::dynamics::SkeletonPtr _skel);
	ReferenceManager* mReferenceManager;

	// for motion matching
	void loadMotionFromBVH(std::string _bvh, bool _isCyclic);
	std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> getWorldTrajectory(std::vector<Pose2d> _local);
	std::vector<Pose2d> generateTargetTrajectory(Eigen::Vector3d _world, std::string _type="");
	std::pair<std::vector<Eigen::VectorXd>,std::vector<std::vector<Pose2d>>> initialize(bool _isCustom, Pose2d _startPose);
	std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<Pose2d>>> step(std::vector<Pose2d> _target, Eigen::VectorXd _current, Eigen::VectorXd _prev);
	Motion convertToFeaturedMotion(Eigen::VectorXd _current, Eigen::VectorXd _prev);
	std::tuple<std::vector<Frame>, bool, std::vector<std::vector<Pose2d>>> searchNextMotion(std::vector<Pose2d> _target, Eigen::VectorXd _current, Eigen::VectorXd _prev);


	// count
	int mTotalFrames;
	int countFeaturedMotions() { return mRawMotions.size(); }

	// getter
	std::pair<Eigen::VectorXd, std::vector<Pose2d>> getCurrentMotion();
	int getLookahead() { return mLookahead; }
	std::map<std::string, double> getTransitionRecord() { return mTransitionRecord; }
	double getTransitionRecordByKey(std::string _key) { return mTransitionRecord[_key]; }

private:
	void generateFeature(int _start, int _end, bool _isCyclic, std::string _bvh);
	std::vector<Eigen::VectorXd> generateCycles(int _length, std::vector<Eigen::VectorXd> _clip);

	dart::dynamics::SkeletonPtr mSkel;
	std::map<std::string, double> mTransitionRecord;

	std::vector<Motion> mRawMotions;
	std::vector<Motion> mAlignedMotions;
	std::vector<int> mContactJoints;
	std::vector<std::string> mContactName;

	int mSearchInterval;
	int mBlendInterval;
	int mLookahead;

	int mCurIdx;

    // random
    std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;
};
}

#endif