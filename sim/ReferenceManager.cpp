#include "ReferenceManager.h"
#include "Functions.h"
#include "BVHParser.h"
#include "Functions.h"
#include "Configurations.h"
namespace SIM
{
ReferenceManager::
ReferenceManager(dart::dynamics::SkeletonPtr _skel) 
:mSkel(_skel), mBlendingInterval(10), mBlend(false), mFixRootRot(false), mFixRootPos(false), mMotionLength(0)
{
	for(int i = 0; i < mSkel->getNumBodyNodes(); i++) {
		// for IK
		std::string name = mSkel->getBodyNode(i)->getName();
		if(name.find("Foot") != std::string::npos) {
			if(name.find("Left") != std::string::npos)
				mLeftFoot.insert(std::pair<std::string, int>("Foot", i));	
			else
				mRightFoot.insert(std::pair<std::string, int>("Foot", i));	
		}
		if(name.find("UpLeg") != std::string::npos) {
			if(name.find("Left") != std::string::npos)
				mLeftFoot.insert(std::pair<std::string, int>("UpLeg", i));	
			else
				mRightFoot.insert(std::pair<std::string, int>("UpLeg", i));	
		}
		else if(name.find("Leg") != std::string::npos) {
			if(name.find("Left") != std::string::npos)
				mLeftFoot.insert(std::pair<std::string, int>("Leg", i));	
			else
				mRightFoot.insert(std::pair<std::string, int>("Leg", i));	
		}
	}

}
Eigen::VectorXd 
ReferenceManager::
computeVelocity(bool _smooth, Eigen::VectorXd _p2, Eigen::VectorXd _p1, Eigen::VectorXd _p0)
{
	Eigen::VectorXd vel;
	if(_smooth) {
		vel = 0.5 * mSkel->getPositionDifferences(_p2, _p1) / NORMALIZED_TIME_STEP 
		      +  0.5 * mSkel->getPositionDifferences(_p2, _p0) / (NORMALIZED_TIME_STEP*2);
	} else {
		vel = mSkel->getPositionDifferences(_p2, _p1) / NORMALIZED_TIME_STEP;
	}
	return vel;
}
void 
ReferenceManager::
loadMotionFromBVH(std::string _bvh, bool _generateCycle)
{
	mMotionClip.clear();
	std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> bvhInfo = BVHParser::parse(_bvh);
	std::vector<std::pair<std::string, int>> idxList = BVHParser::getNodeIdx(std::get<0>(bvhInfo)); 


	double timeStep = std::get<1>(bvhInfo);
	int dof = mSkel->getNumDofs();

	double t = 0;

	int count = 0;
	for(int i = 0; i < std::get<2>(bvhInfo).size(); i++)
	{
		bool flag = false;
		Eigen::VectorXd	p;
		if(abs(timeStep - NORMALIZED_TIME_STEP) <= 1e-4) {
			p = std::get<2>(bvhInfo)[i];
			flag = true;
		}
		else if(t >= count * NORMALIZED_TIME_STEP) {

			if(i != 0) {
				Eigen::VectorXd p0 = std::get<2>(bvhInfo)[i-1];
				Eigen::VectorXd p1 = std::get<2>(bvhInfo)[i];

				double t0 = t - timeStep;
				double t1 = t;

				p = weightedSumPos(p0, p1, (count * NORMALIZED_TIME_STEP - t0) / (t1 - t0));
			} else {
				p = std::get<2>(bvhInfo)[i];
			}
			flag = true;
		}

		if(flag) {
			Eigen::VectorXd pos(dof);
			for(int j = 0; j < idxList.size(); j++) {
				dart::dynamics::BodyNode* bn = mSkel->getBodyNode(idxList[j].first);
				int idx = bn->getParentJoint()->getIndexInSkeleton(0);
				if(idx == 0) {
					pos.segment<3>(3) = p.segment<3>(0);
					pos.segment<3>(0) = p.segment<3>(3);
				} else {
					pos.segment<3>(idx) = p.segment<3>(idxList[j].second);
				}			
			}
			Eigen::VectorXd vel = Eigen::VectorXd::Zero(dof);
			if(count >= 2) {
				vel = computeVelocity(false, pos, mMotionClip[count - 1].position, mMotionClip[count - 2].position);
			} else if(count == 1) {
				vel = computeVelocity(false, pos, mMotionClip[0].position);
				mMotionClip[0].velocity = vel;
			}
			mMotionClip.push_back(Frame(pos, vel));
			count += 1;
		}
		if(t < count * NORMALIZED_TIME_STEP)
			t += timeStep;
	}
	mMotionLength = mMotionClip.size();
	if(_generateCycle) {
		generateMotionsFromSingleClip(std::max(mMotionLength*TERMINAL_ITERATION, mMotionLength), mMotionClip, mMotionGen, 
									mBlend, mFixRootRot, mFixRootPos);
	}
	else
		mMotionGen = mMotionClip;

}
void 
ReferenceManager::
generateMotionsFromSingleClip(int _numFrames, std::vector<Frame>& _clip, std::vector<Frame>& _gen,
							  bool _blend, bool _fixRootRot, bool _fixRootPos)
{
	_gen.clear();

	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_clip[0].position.head<6>());
	Eigen::Isometry3d T0Updated = T0;

	int totalLength = _clip.size();
	for(int i = 0; i < _numFrames; i++) {
		
		int phase = i % totalLength;
		
		if(i < totalLength) {
			_gen.push_back(Frame(_clip[i]));
		} 
		else {
			Eigen::VectorXd pos;
			if(phase == 0) {
				pos = _clip[0].position;
				if(!_fixRootRot) {
					Eigen::Vector3d rootOld = _clip[0].position.segment<3>(0);
					Eigen::Vector3d rootNew = _gen.back().position.segment<3>(0);
					Eigen::Vector3d deltaRoot = getPosDiffXZplane(rootNew, rootOld);
					pos.segment<3>(0) = logMap(expMapRot(rootOld) * expMapRot(deltaRoot)); 
				}
				if(!_fixRootPos) {
					pos.segment<3>(3) = _gen.back().position.segment<3>(3);
					pos(4) = _clip[0].position(4);
				}

				T0Updated = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
			} 
			else {
				pos = _clip[phase].position;
				Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
				TCurrent = T0.inverse() * TCurrent;
				TCurrent = T0Updated * TCurrent;
				pos.segment<6>(0) = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
			}

			Eigen::VectorXd vel = computeVelocity(false, pos, _gen[_gen.size() - 1].position, _gen[_gen.size() - 2].position);
			_gen.push_back(Frame(pos, vel));
	
			if(_blend && phase == 0) {
				for(int j = mBlendingInterval; j > 0; j--) {
					double weight = 1 - j / (double)(mBlendingInterval+1);
					Eigen::VectorXd oldPos = _gen[i - j].position;
					_gen[i - j].position = weightedSumPos(oldPos, pos, weight);
					_gen[i - j].velocity = computeVelocity(false, _gen[i - j].position, _gen[i - j - 1].position, _gen[i - j - 2].position);
				}
			}
		}
	}
}

// align and inertialize
// align source motion to the _Target frame's root
std::vector<Frame> 
ReferenceManager::
align(Eigen::VectorXd _target, Eigen::VectorXd _targetPrev, std::vector<Frame> _source, bool _blend) 
{
	std::vector<Frame> aligned = _source;

	Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(_source[0].position.head<6>());
	Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(_target.head<6>());

	Pose2d T0Local = Transform::to2d(T0);
	Pose2d T1Local = Transform::to2d(T1);

	Eigen::Isometry3d T01 = Transform::getGlobalTransform(T0Local, T1Local);
	Eigen::Isometry3d T0new = T01 * T0;
	
	for(int i = 0; i < _source.size(); i++) {
		Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(_source[i].position.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;

		aligned[i].position.head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);
	}
	for(int i = 0; i < _source.size(); i++) {
		if(i != 0)
			aligned[i].velocity = mSkel->getPositionDifferences(aligned[i].position, aligned[i - 1].position) / NORMALIZED_TIME_STEP;
		else
			aligned[i].velocity = mSkel->getPositionDifferences(aligned[i].position, _target) / NORMALIZED_TIME_STEP;
	}

	std::vector<Frame> inertialized = aligned;
	if(_blend) {
		inertialized = inertialize(aligned, _target, _targetPrev, aligned[0].position);
		footCleanup(inertialized);
	}

	return inertialized;
}
std::vector<Frame> 
ReferenceManager::
inertialize(std::vector<Frame> _aligned, Eigen::VectorXd _old, Eigen::VectorXd _oldPrev, Eigen::VectorXd _new)
{
	Eigen::ArrayXd t1 = Eigen::VectorXd::Constant(mSkel->getNumDofs(), 10*NORMALIZED_TIME_STEP);
	Eigen::ArrayXd x0(mSkel->getNumDofs());
	Eigen::ArrayXd v0(mSkel->getNumDofs());
	Eigen::VectorXd applyInertialize(mSkel->getNumDofs());

	for(int i = 0; i < mSkel->getNumBodyNodes(); i++) {
		if(i == 0) {
			x0.segment<3>(0) = jointPositionDifferences(_old.segment<3>(0), _new.segment<3>(0));
			v0.segment<3>(0) = jointPositionDifferences(_old.segment<3>(0), _oldPrev.segment<3>(0)) / NORMALIZED_TIME_STEP;
			
			x0.segment<3>(3) = _old.segment<3>(3) - _new.segment<3>(3);
			v0.segment<3>(3) = (_old.segment<3>(3) - _oldPrev.segment<3>(3)) / NORMALIZED_TIME_STEP;

		} else {
			x0.segment<3>(3*i + 3) = jointPositionDifferences(_old.segment<3>(3*i + 3), _new.segment<3>(3*i + 3));
			v0.segment<3>(3*i + 3) = jointPositionDifferences(_old.segment<3>(3*i + 3), _oldPrev.segment<3>(3*i + 3)) / NORMALIZED_TIME_STEP;
		}
	}

	for(int i = 0; i < x0.rows(); i++) {
		if(x0(i)*v0(i) > 0)
			v0(i) = 0;
	}

	for(int i = 0; i < t1.rows(); i++) {
		if(v0(i) != 0) {
			t1(i) = std::min(t1(i), -5 * x0(i) / v0(i));
			applyInertialize(i) = 1;
		}
		else {
			applyInertialize(i) = 0;
		}
	}

	Eigen::ArrayXd t1_2 = t1*t1;
	Eigen::ArrayXd t1_3 = t1_2*t1;
	Eigen::ArrayXd t1_4 = t1_3*t1;
	Eigen::ArrayXd t1_5 = t1_4*t1;

	Eigen::ArrayXd a0 = (-8*NORMALIZED_TIME_STEP*t1*v0 - 20 * x0) / t1_2;
	for(int i = 0; i < a0.rows(); i++) {
		if(v0(i)*a0(i) > 0)
			a0(i) = 0;
	}

	Eigen::ArrayXd a0t1_2 = a0*t1_2;
	Eigen::ArrayXd v0t1 = v0*t1;

	Eigen::ArrayXd A = -(a0t1_2 + 6 * v0t1 + 12 * x0) / (2 * t1_5);
	Eigen::ArrayXd B = (3 * a0t1_2 + 16 * v0t1 + 30 * x0) / (2 * t1_4);
	Eigen::ArrayXd C = -(3 * a0t1_2 + 12 * v0t1 + 20 * x0) / (2 * t1_3);

	for(int i = 0 ; i < _aligned.size(); i++) {
		Eigen::ArrayXd t = Eigen::VectorXd::Constant(mSkel->getNumDofs(), (i+1)*NORMALIZED_TIME_STEP);
		// if all rows are positive, no more computation is needed
		if((t - t1).matrix() == (t - t1).matrix().cwiseAbs())
			break;

		Eigen::ArrayXd t_2 = t*t;
		Eigen::ArrayXd t_3 = t_2*t;
		Eigen::ArrayXd t_4 = t_3*t;
		Eigen::ArrayXd t_5 = t_4*t;

		Eigen::ArrayXd xt = A*t_5 + B*t_4 + C*t_3 + 1 / 2.0 * a0*t_2 + v0*t + x0;
		for(int j = 0; j < t1.rows(); j++) {
			if(t(j) > t1(j))
				xt(j) = 0;
			else if(!applyInertialize(j)) {
				xt(j) = (1 - t(j) / (t1(j)+NORMALIZED_TIME_STEP)) * x0(j);
			}
		}
		for(int j = 0; j < mSkel->getNumBodyNodes(); j++) {
			if(j == 0) {
				_aligned[i].position.segment<3>(0) = rotate(_aligned[i].position.segment<3>(0), xt.segment<3>(0));
				_aligned[i].position.segment<3>(3) += xt.segment<3>(3).matrix();			
			} else {
				_aligned[i].position.segment<3>(3*j+3) = rotate(_aligned[i].position.segment<3>(3*j+3), xt.segment<3>(3*j+3));
			}
		}
	}
	return _aligned;
}

std::vector<std::vector<bool>>
ReferenceManager::
extractContact(std::vector<Eigen::VectorXd> _motion)
{
	int motionLength = _motion.size();
    std::vector<std::vector<bool>> contactsBoth;

    for(int i = 0; i <= 1; i++) {
		std::map<std::string, int> footJoints = i ? mLeftFoot : mRightFoot;
		std::map<std::string, int> oppFootJoints = i ? mRightFoot : mLeftFoot;
		std::vector<bool> contacts;

		mSkel->setPositions(_motion[0]);
		Eigen::VectorXd footPos = mSkel->getBodyNode(footJoints["Foot"])->getWorldTransform().translation();	
		Eigen::VectorXd oppFootPos = mSkel->getBodyNode(oppFootJoints["Foot"])->getWorldTransform().translation();	
		for(int j  = 1; j < motionLength; j++) 
		{
			mSkel->setPositions(_motion[j]);
			Eigen::VectorXd curFootPos = mSkel->getBodyNode(footJoints["Foot"])->getWorldTransform().translation();
			Eigen::VectorXd curOppFootPos = mSkel->getBodyNode(oppFootJoints["Foot"])->getWorldTransform().translation();
			
			int footHeight = mSkel->getBodyNode(footJoints["Toe"])->getWorldTransform().translation()(1);
			footHeight += mSkel->getBodyNode(footJoints["Foot"])->getWorldTransform().translation()(1);
			footHeight /= 2;

			// bool curContact = (curFootPos - footPos).norm() < 0.036;
			bool curContact = false;
			double curVel = (curFootPos - footPos).norm();
			double curOppVel = (curOppFootPos - oppFootPos).norm();
			if(curVel < 0.02 && footHeight < CONTACT_HEIGHT) {
				curContact = true;
			}
			else
				curContact = curVel < curOppVel;
			
            contacts.push_back(curContact);
			if(j == 1) {
				contacts.push_back(curContact);
			}
			footPos = curFootPos;
			oppFootPos = curOppFootPos;
		}
    contactsBoth.push_back(contacts);
    }
	return contactsBoth;
}

void 
ReferenceManager:: 
footCleanup(std::vector<Frame>& _motion, std::vector<std::vector<bool>> _contacts)
{
	int motionLength = _motion.size();
	bool prevCon = false;
	bool curCon;
	for(int i = 0; i <= 1; i++) {
		std::map<std::string, int> footJoints = i ? mLeftFoot : mRightFoot;
		std::vector<bool> contacts = _contacts[i];
		int frameStart = 0;
		for(int i = 1; i < motionLength; i++) 
		{
			prevCon = contacts[i-1];
			curCon = contacts[i];
			if(i == (motionLength-1))
				curCon = false;

			if(prevCon && !curCon) {
				int mid = (frameStart + i)/2;
				int start = std::min(mid, frameStart);
				int end = std::max(mid, i);
				if(start != end) {
					glueFoot(_motion, mid, start, end, footJoints);
				}
			} else if(!prevCon && curCon) {
				frameStart = i;
			}
		}
	}
}

void
ReferenceManager::
footCleanup(std::vector<Frame>& _motion)
{
	int motionlength = _motion.size();
	for(int i = 0; i <= 1; i++) {
		std::map<std::string, int> footJoints = i ? mLeftFoot : mRightFoot;
		std::vector<bool> contacts;	
		for(int i  = 0; i < motionlength; i++) 
		{
			mSkel->setPositions(_motion[i].position);
			int footHeight = mSkel->getBodyNode(footJoints["Toe"])->getWorldTransform().translation()(1);
			footHeight += mSkel->getBodyNode(footJoints["Foot"])->getWorldTransform().translation()(1);
			footHeight /= 2;

			bool curContact = footHeight < CONTACT_HEIGHT;
			contacts.push_back(curContact);
		}

		int frameStart = 0;
		for(int i = 1; i < motionlength; i++) 
		{
			bool prevCon = contacts[i-1];
			bool curCon = contacts[i];

			if(prevCon && !curCon) {
				int mid = (frameStart + i)/2;
				int start = std::min(mid, frameStart);
				int end = std::max(mid, i);
				if(start != end) {
					glueFoot(_motion, mid, start, end, footJoints);
					// smooth
					smoothBefore(_motion, start, mid, std::min(start, 4), footJoints);
					smoothAfter(_motion, end, mid, std::min((motionlength-end-1), 4), footJoints);
				}
			} else if(!prevCon && curCon) {
				frameStart = i;
			}
		}
	}
}

void
ReferenceManager::
glueFoot(std::vector<Frame>& _motion, int mid, int start, int end, std::map<std::string, int> _footJoints) 
{
	for(int i  = start; i < end; i++) {
		// foot position of mid (in world coord) - this is that target pose
		mSkel->setPositions(_motion[mid].position);
		Eigen::Isometry3d targetFoot = mSkel->getBodyNode(_footJoints["Foot"])->getWorldTransform() * mSkel->getBodyNode(_footJoints["Foot"])->getParentJoint()->getTransformFromChildBodyNode();
		IKLimb(_motion, i, targetFoot.translation(), _footJoints);
	}
}

void
ReferenceManager:: 
smoothBefore(std::vector<Frame>& _motion, int _start, int _mid, int _duration, std::map<std::string, int> _footJoints)
{
	mSkel->setPositions(_motion[_mid].position);
	Eigen::Isometry3d midFoot = mSkel->getBodyNode(_footJoints["Foot"])->getWorldTransform() * mSkel->getBodyNode(_footJoints["Foot"])->getParentJoint()->getTransformFromChildBodyNode();
	for(int i = 1; i < _duration; i++) {
		double weight = i/(double)_duration;
		weight = 0.5*cos(M_PI*weight)+0.5;
		int idx = _start-(_duration-i);

		mSkel->setPositions(_motion[idx].position);
		Eigen::Isometry3d targetFoot = mSkel->getBodyNode(_footJoints["Foot"])->getWorldTransform() * mSkel->getBodyNode(_footJoints["Foot"])->getParentJoint()->getTransformFromChildBodyNode();
		Eigen::Vector3d smoothTarget = weight * targetFoot.translation() + (1-weight) * midFoot.translation();
		IKLimb(_motion, idx, smoothTarget, _footJoints); 
	}
}

void
ReferenceManager:: 
smoothAfter(std::vector<Frame>& _motion, int _end, int _mid, int _duration, std::map<std::string, int> _footJoints)
{
	mSkel->setPositions(_motion[_mid].position);
	Eigen::Isometry3d midFoot = mSkel->getBodyNode(_footJoints["Foot"])->getWorldTransform() * mSkel->getBodyNode(_footJoints["Foot"])->getParentJoint()->getTransformFromChildBodyNode();
	for(int i = 1; i < _duration; i++) {
		double weight = i/(double)_duration;
		weight = 0.5*cos(M_PI*weight)+0.5;
		int idx = _end+(_duration-i);

		mSkel->setPositions(_motion[idx].position);
		Eigen::Isometry3d targetFoot = mSkel->getBodyNode(_footJoints["Foot"])->getWorldTransform() * mSkel->getBodyNode(_footJoints["Foot"])->getParentJoint()->getTransformFromChildBodyNode();
		Eigen::Vector3d smoothTarget = (1-weight) * targetFoot.translation() + (weight) * midFoot.translation();
		IKLimb(_motion, idx, smoothTarget, _footJoints); 
	}
}

void
ReferenceManager::
IKLimb(std::vector<Frame>& _motion, int _idx, Eigen::Vector3d _target, std::map<std::string, int> _footJoints)
{

	// target toe position in world coordinates
	mSkel->setPositions(_motion[_idx].position);
	dart::dynamics::BodyNode* bn_a = mSkel->getBodyNode(_footJoints["UpLeg"]);
	dart::dynamics::BodyNode* bn_b = mSkel->getBodyNode(_footJoints["Leg"]);
	dart::dynamics::BodyNode* bn_c = mSkel->getBodyNode(_footJoints["Foot"]);

	Eigen::Isometry3d a_iso = bn_a->getWorldTransform() * bn_a->getParentJoint()->getTransformFromChildBodyNode();
	Eigen::Isometry3d b_iso = bn_b->getWorldTransform() * bn_b->getParentJoint()->getTransformFromChildBodyNode();
	Eigen::Isometry3d c_iso = bn_c->getWorldTransform() * bn_c->getParentJoint()->getTransformFromChildBodyNode();

	int a_idx, b_idx, c_idx;
	a_idx = bn_a->getParentJoint()->getIndexInSkeleton(0);
	b_idx = bn_b->getParentJoint()->getIndexInSkeleton(0);
	c_idx = bn_c->getParentJoint()->getIndexInSkeleton(0);

	Eigen::Vector3d a = a_iso.translation();
	Eigen::Vector3d b = b_iso.translation();
	Eigen::Vector3d c = c_iso.translation();

	double lab = (b-a).norm();
    double lcb = (b-c).norm();
    double lat = dart::math::clip((_target-a).norm(), 0.01, lab+lcb-0.01);

	Eigen::Vector3d c_a = (c-a).normalized();
	Eigen::Vector3d b_a = (b-a).normalized();
	Eigen::Vector3d a_b = (a-b).normalized();
	Eigen::Vector3d c_b = (c-b).normalized();
	Eigen::Vector3d t_a = (_target-a).normalized();

    double ac_ab_0 = acos(dart::math::clip(c_a.dot(b_a), -1.0, 1.0));
    double ba_bc_0 = acos(dart::math::clip(a_b.dot(c_b), -1.0, 1.0));
    double ac_at_0 = acos(dart::math::clip(c_a.dot(t_a), -1.0, 1.0));

    double ac_ab_1 = acos(dart::math::clip((lcb*lcb-lab*lab-lat*lat) / (-2*lab*lat), -1.0, 1.0));
    double ba_bc_1 = acos(dart::math::clip((lat*lat-lab*lab-lcb*lcb) / (-2*lab*lcb), -1.0, 1.0));

	Eigen::Vector3d axis0, axis1;
    axis0 = ((c_a).cross(b_a)).normalized();
    axis1 = ((c_a).cross(t_a)).normalized();

	// change into local coordinates
	Eigen::AngleAxisd r0, r1, r2;
	r0.axis() = a_iso.linear().inverse() * axis0;
	r1.axis() = b_iso.linear().inverse() * axis0;
	r2.axis() = a_iso.linear().inverse() * axis1;

	double r0_angle, r1_angle, r2_angle;
	r0_angle = ac_ab_1 - ac_ab_0;
	r1_angle = ba_bc_1 - ba_bc_0;
	r2_angle = ac_at_0;

	Eigen::Vector3d a_lr = logMap(expMapRot(_motion[_idx].position.segment<3>(a_idx))*expMapRot(r0.axis() * r0_angle)*expMapRot(r2.axis() * r2_angle));
	Eigen::Vector3d b_lr = logMap(expMapRot(_motion[_idx].position.segment<3>(b_idx))*expMapRot(r1.axis() * r1_angle));

	_motion[_idx].position.segment<3>(a_idx) = a_lr;
	_motion[_idx].position.segment<3>(b_idx) = b_lr;
	
}

Frame 
ReferenceManager::
getFrame(double _t)
{

	if(mMotionGen.size()-1 < _t) {
	 	return mMotionGen.back();
	}
	
	int k0 = (int) std::floor(_t);
	int k1 = (int) std::ceil(_t);	

	if (k0 == k1)
		return mMotionGen[k0];
	else {
		return Frame(weightedSumPos(mMotionGen[k0].position, mMotionGen[k1].position, (_t-k0)), 
					      weightedSumVec(mMotionGen[k0].velocity, mMotionGen[k1].velocity, (_t-k0)));		
	}
}

void
ReferenceManager::
setMotionGen(std::vector<Frame> _motion) 
{ 
	mMotionGen = _motion;
	mMotionLength = mMotionGen.size(); 
}


};
