#ifndef __SIM_REFERENCE_MANAGER_H__
#define __SIM_REFERENCE_MANAGER_H__

#include <tuple>
#include <mutex>
#include "dart/dart.hpp"
#include "Transform.h"
#include "SkeletonBuilder.h"

namespace SIM
{
struct Frame
{
	Frame(Eigen::VectorXd _p, Eigen::VectorXd _v): position(_p), velocity(_v), phaseDelta(1) {}
	Frame(Eigen::VectorXd _p, Eigen::VectorXd _v, double _t): position(_p), velocity(_v), phaseDelta(_t) {}
	Frame(const SIM::Frame& _mn): position(_mn.position), velocity(_mn.velocity), phaseDelta(_mn.phaseDelta) {}
	Eigen::VectorXd position;
	Eigen::VectorXd velocity;
	double phaseDelta;
};
class ReferenceManager
{
public:
	ReferenceManager(dart::dynamics::SkeletonPtr _skel);

	Eigen::VectorXd computeVelocity(bool _smooth, Eigen::VectorXd _p2, Eigen::VectorXd _p1, Eigen::VectorXd _p0=Eigen::VectorXd::Zero(1));
	void loadMotionFromBVH(std::string _bvh, bool _generateCycle=true);
	void generateMotionsFromSingleClip(int _numFrames, std::vector<Frame>& _clip, std::vector<Frame>& _gen,
									   bool _blend=false, bool _fixRootRot=false, bool _fixRootPos=false);
	// align and inertialize related
	std::vector<Frame> align(Eigen::VectorXd _target, Eigen::VectorXd _targetPrev, std::vector<Frame> _source, bool _blend);
	std::vector<Frame> inertialize(std::vector<Frame> _aligned, Eigen::VectorXd _old, Eigen::VectorXd _oldPrev, Eigen::VectorXd _new);
	
	std::vector<std::vector<bool>> extractContact(std::vector<Eigen::VectorXd> _motion);
	void footCleanup(std::vector<Frame>& _motion, std::vector<std::vector<bool>> _contacts);
	void footCleanup(std::vector<Frame>& _motion);
	void smoothBefore(std::vector<Frame>& _motion, int _start, int _mid, int _duration, std::map<std::string, int> _footJoints);
	void smoothAfter(std::vector<Frame>& _motion, int _end, int _mid, int _duration, std::map<std::string, int> _footJoints);
	void glueFoot(std::vector<Frame>& _frame, int mid, int start, int end, std::map<std::string, int> _footJoints); 
	void IKLimb(std::vector<Frame>& _motion, int _idx, Eigen::Vector3d _target, std::map<std::string, int> _footJoints);

	Frame getFrame(double _t);
	int getMotionLength() { return mMotionLength; }
	double getNumDof() {return mMotionClip[0].position.rows() + 1; }
	dart::dynamics::SkeletonPtr getSkel() { return mSkel; }
	std::vector<Frame> getMotionGen() { return mMotionGen; }
	std::vector<Frame> getMotionClip() { return mMotionClip; }
	void setMotionGen(std::vector<Frame> _motion);

	std::map<std::string, int> getLeftFoot() { return mLeftFoot; };
	std::map<std::string, int> getRightFoot() { return mRightFoot; };

protected:
	dart::dynamics::SkeletonPtr mSkel;
	int mBlendingInterval;
	int mMotionLength;
	bool mBlend;
	bool mFixRootRot;
	bool mFixRootPos;
	std::vector<Frame> mMotionClip;
	std::vector<Frame> mMotionGen;
	std::map<std::string, int> mLeftFoot;
	std::map<std::string, int> mRightFoot;

};
}

#endif