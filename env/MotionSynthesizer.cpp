#include "MotionSynthesizer.h"
#include "Configurations.h"
#include "Functions.h"
#include <fstream>
// #include <chrono>
#include "EnvConfigurations.h"

namespace ENV
{

// constructor
MotionSynthesizer::
MotionSynthesizer(dart::dynamics::SkeletonPtr _skel, std::map<std::string, std::string> _database_folder)
{
    // build motion database for walk and sit
    for(auto const& pair : _database_folder) {
        std::cout << "motion type: " << pair.first << std::endl;
        std::string mmPath = LAMA_DIR + std::string("/data/motion/") + pair.second;
        if(!boost::filesystem::is_directory(mmPath)) {
            std::cout << __func__ << " not a valid directory : " << mmPath << std::endl; 
        }
        if(pair.first == "walk") {
            mMotionDatabase = new SIM::MotionDatabase(_skel);
            std::vector<std::string> filenameList;
            // for(auto& file : boost::make_iterator_range(boost::filesystem::directory_iterator(mmPath))) {
            //     std::string motionPath = file.path().string();
            //     mMotionDatabase->loadMotionFromBVH(motionPath, false);
            //     std::cout << "added " << motionPath << " to motion database" << std::endl;
            // }
            for(auto& file : boost::make_iterator_range(boost::filesystem::directory_iterator(mmPath))) {
                std::string motionPath = file.path().string();
                filenameList.push_back(motionPath);
            }
            std::sort(filenameList.begin(), filenameList.end());
            for(std::string motionPath : filenameList) {
                mMotionDatabase->loadMotionFromBVH(motionPath, false);
                std::cout << "added " << motionPath << " to motion database" << std::endl;
            }

        }
        else if(pair.first == "sit") {
            mSitMotionDatabase = new SIM::SitMotionDatabase(_skel);
            for(auto& file : boost::make_iterator_range(boost::filesystem::directory_iterator(mmPath))) {
                std::string motionPath = file.path().string();
                mSitMotionDatabase->loadMotionFromBVH(motionPath);
                std::cout << "added " << motionPath << " to motion database" << std::endl;
            }
        }
    }
    mSkel = _skel;
    mReferenceManager = new SIM::ReferenceManager(_skel);
}

void 
MotionSynthesizer::
initialize(bool _isCustom, SIM::Pose2d _startPose, bool _keepCustomRecord)
{
    mMotionRecord.clear();
    mTrajectoryRecord.clear();
    std::pair<std::vector<Eigen::VectorXd>,std::vector<std::vector<SIM::Pose2d>>> initial = mMotionDatabase->initialize(_isCustom, _startPose);
    mMotionRecord.insert(mMotionRecord.end(), initial.first.begin(), initial.first.end());
    mTrajectoryRecord.insert(mTrajectoryRecord.end(), initial.second.begin(), initial.second.end());
    mCurIdx = 0;
    mCurActionFeature = ENV::ActionFeature();
    mCurActionFeature.type = "walk";
    if(!_keepCustomRecord)
        mTransitionRecord.clear();
}

std::pair<bool,std::map<std::string, double>>
MotionSynthesizer::
step(ActionFeature _actionFeature, InteractionCue _interactionCue)
{
    // when to search for new motion
    // when idx reached to an end - search if walk 
    // when action feature change 
    bool searchFlag = checkSearchNewMotion(_actionFeature);
    std::vector<SIM::Pose2d> target  = extractTarget(_actionFeature, _interactionCue);
    std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<SIM::Pose2d>>> result;
    double editReward = 1;

    if(searchFlag) {
        result = searchNewMotion(_actionFeature, target);
        if(!std::get<0>(result)) {
            _actionFeature.type = "walk";
            step(_actionFeature, _interactionCue); 
            mCurIdx -= 1; // no stepping in failure
        }
    }
    
    if(std::get<0>(result)) {
        if(_actionFeature.type != mCurActionFeature.type) {
            mTransitionRecord.push_back(std::tuple<int,int,std::string,InteractionCue>(mCurIdx+1, std::get<1>(result).size(), _actionFeature.type, _interactionCue));
        }
        std::vector<Eigen::VectorXd> motion = std::get<1>(result);
        for(int i = 0; i < motion.size(); i++) {
            int curIdx = mCurIdx + i+1;
            if(curIdx < mMotionRecord.size())
                mMotionRecord[curIdx] = motion[i];
            else    
                mMotionRecord.push_back(motion[i]);
        }
        mTrajectoryRecord.insert(mTrajectoryRecord.end(), std::get<2>(result).begin(), std::get<2>(result).end());
        mCurActionFeature = _actionFeature;
    }

    mCurIdx += 1;
    std::map<std::string, double> record = getTransitionCostRecord(_actionFeature.type);
    return std::pair<bool, std::map<std::string, double>>(searchFlag, record);
}

std::vector<SIM::Pose2d> 
MotionSynthesizer::
extractTarget(ActionFeature _actionFeature, InteractionCue _interactionCue)
{
    if(_actionFeature.type == "walk") {
        return _actionFeature.targetTrajectory;
    }
    
    Eigen::VectorXd current = mMotionRecord[mCurIdx];
    Eigen::Isometry3d curRoot = dart::dynamics::FreeJoint::convertToTransform(current.head<6>());
    
    if(_actionFeature.type == "stop") {
        std::vector<SIM::Pose2d> localTraj;
        for(int i = 0; i < _actionFeature.targetTrajectory.size(); i++) {
            localTraj.push_back(_actionFeature.targetTrajectory[i]);
        }
        for(Contact c : _interactionCue.mContactList) {
            auto bn = mSkel->getBodyNode(c.jointIdx);
            if(bn->getName() == "Hips") {
                SIM::Pose2d localTarget = SIM::Transform::getLocalTransform(SIM::Transform::to2d(curRoot), c.contactRootPos);
                localTraj.push_back(localTarget);
            }
        }
        return localTraj;
    }
    else if(_actionFeature.type == "sit") {
        std::vector<SIM::Pose2d> target;
        // sort contact list based on joint index -> assume IC in joint is given in order
        sort(_interactionCue.mContactList.begin(), _interactionCue.mContactList.end(), 
              [](const Contact& lc, const Contact& rc) {return lc.jointIdx < rc.jointIdx;});

        for(Contact c : _interactionCue.mContactList) {
            auto bn = mSkel->getBodyNode(c.jointIdx);
            if(bn->getName() == "Hips") {
                SIM::Pose2d localTarget = SIM::Transform::getLocalTransform(SIM::Transform::to2d(curRoot), c.contactRootPos);
                target.push_back(localTarget);
            } else if((bn->getName()).find("Foot") != std::string::npos) {
                SIM::Pose2d targetPos = SIM::Pose2d(SIM::Transform::to2dPos(c.contactPos));
                SIM::Pose2d localTarget = SIM::Transform::getLocalTransform(SIM::Transform::to2d(curRoot), targetPos);
                target.push_back(localTarget);
            }
        }
        return target;
    }
}

std::pair<Eigen::VectorXd, std::vector<SIM::Pose2d>>
MotionSynthesizer::
getCurrentMotion() 
{

    if(mCurIdx >= mMotionRecord.size())
        mCurIdx = mMotionRecord.size()-1;

    std::vector<SIM::Pose2d> traj = mTrajectoryRecord[mCurIdx];
    Eigen::VectorXd pos = mMotionRecord[mCurIdx];
    std::pair<Eigen::VectorXd, std::vector<SIM::Pose2d>> result = { pos, traj };
    return result;
}

std::map<std::string, double>
MotionSynthesizer:: 
getTransitionCostRecord(std::string _actionFeatureType)
{
    std::map<std::string, double> rec;
    std::string type = mCurActionFeature.type;
    if(type == "sit") {
        return mSitMotionDatabase->getTransitionRecord();
    }
    else if(type == "walk") {
        return mMotionDatabase->getTransitionRecord();
    }    
    return rec;
}

std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<SIM::Pose2d>>>
MotionSynthesizer::
searchNewMotion(ActionFeature _actionFeature, std::vector<SIM::Pose2d> _localTarget)
{
    std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<SIM::Pose2d>>> result;
    Eigen::VectorXd current = mMotionRecord[mCurIdx];
    Eigen::VectorXd prev = mMotionRecord[mCurIdx-1];

    if(_actionFeature.type == "sit") {
        result = mSitMotionDatabase->step(_localTarget, current, prev);
    }
    else if(_actionFeature.type == "walk") {
        result = mMotionDatabase->step(_localTarget, current, prev);
    }
    else if(_actionFeature.type == "stop") {
        if(_localTarget.back().pos.norm() > 0.5) {
            std::get<0>(result) = false;
            return result;
        }
        std::vector<SIM::Pose2d> localTarget(_localTarget.begin(), _localTarget.end()-1);
        result = mMotionDatabase->step(localTarget, current, prev);
    }
    return result;
}

bool 
MotionSynthesizer::
checkSearchNewMotion(ActionFeature _actionFeature)
{
    std::string type = _actionFeature.type;
    if(mCurActionFeature.type == "stop" || mCurActionFeature.type == "sit")
        return false;
    if(type != "walk" && mCurActionFeature.type == "walk") {
        return true;
    }
    if (type == "stop" && mCurActionFeature.type == "walk")
        return true;
    else if(type == "walk" && mCurActionFeature.type != "sit") // sit -> walk is not activated yet 
    {
        if(mCurActionFeature.type == "walk" && mMotionRecord.size()-2 == mCurIdx)
            return true;
        else if(mCurActionFeature.type == "stop")
            return true;
    }
    return false;
}

bool 
MotionSynthesizer:: 
checkActionComplete()
{
    std::string type = mCurActionFeature.type;
    if(type != "walk" && (mMotionRecord.size()-1 == mCurIdx)) 
        return true;
    else
        return false;
}

Eigen::Vector3d 
MotionSynthesizer:: 
checkRootPos(int _interval)
{
    int count = 0;
    Eigen::Vector3d avgRootPos = Eigen::Vector3d::Zero();
    for(int i = 0; i < _interval; i++) {
        int idx = mCurIdx - i;
        if(idx < 0) idx = 0;
        avgRootPos += mMotionRecord[idx].segment<3>(3);
        count += 1;
    }
    avgRootPos /= (double) count;
    return avgRootPos;
}

double 
MotionSynthesizer::
checkRootVel(int _interval)
{
    double velNorm = 1.0;
    if(_interval >= mCurIdx)
        return velNorm;
    for(int i = 0; i < (_interval-1); i++) {
        Eigen::Vector3d vel = mMotionRecord[mCurIdx-i].segment<3>(3) - mMotionRecord[mCurIdx-(i+1)].segment<3>(3);
        velNorm += vel.norm();
    }
    velNorm /= (double) _interval;
    return velNorm;
}

std::vector<SIM::Pose2d>
MotionSynthesizer:: 
generateTargetTrajectory(Eigen::Vector3d _world, bool _localPos)
{
	std::vector<SIM::Pose2d> trajectory;
        
    if(mCurActionFeature.type == "sit")
        return trajectory;
    
    // reference: https://theorangeduck.com/page/spring-roll-call#damper -> The Spring Damper
	SIM::Pose2d T = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mMotionRecord[mCurIdx].head<6>())); 
	SIM::Pose2d Ttarget = SIM::Pose2d(SIM::Transform::to2dPos(_world), Eigen::Vector2d(1, 0));
	Ttarget = SIM::Transform::getLocalTransform(T, Ttarget);
	if(_localPos) {
        Ttarget.pos = SIM::Transform::to2dPos(_world);
    }
    if(mCurActionFeature.type == "stop") {
        Ttarget.pos = SIM::Transform::to2dPos(Eigen::Vector3d::Zero());
    }

	if(Ttarget.pos.norm() > 1) {
		Ttarget.pos = Ttarget.pos.normalized() * 1;
	}
	Ttarget.pos = 1.9 * Ttarget.pos.norm() * Ttarget.pos.normalized();
	
	std::vector<Eigen::Vector2d> xs;
	std::vector<Eigen::Vector2d> vs;

	double stiffness = 0.5;
	double dt1 = 0.3;
	double dt2 = 0.5;

	xs.push_back(Eigen::Vector2d(0, 0));
	vs.push_back(mTrajectoryRecord[mCurIdx][0].pos);

	for(int i = 0; i < mMotionDatabase->getLookahead(); i++) {
		Eigen::Vector2d a = stiffness * (Ttarget.pos - xs[i]);
		Eigen::Vector2d v = vs[i] + dt1 * a;
		Eigen::Vector2d x = xs[i] + dt2 * v;
		SIM::Pose2d t = SIM::Pose2d(x, (x - xs.back()).normalized());
		trajectory.push_back(t);
		xs.push_back(x);
		vs.push_back(v);
	}

	return trajectory;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> 
MotionSynthesizer::
getWorldTrajectory(std::vector<SIM::Pose2d> _local)
{
    if(mCurActionFeature.type != "walk") {
        std::vector<Eigen::Vector3d> t1, t2;
        std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> traj = {t1, t2};
        return traj;
    }

    SIM::Pose2d T = SIM::Transform::to2d(dart::dynamics::FreeJoint::convertToTransform(mMotionRecord[mCurIdx].head<6>()));

	std::vector<Eigen::Vector3d> pos;
	std::vector<Eigen::Vector3d> dir;
	for(int i = 0; i < mMotionDatabase->getLookahead(); i++) {

		SIM::Pose2d next = SIM::Transform::applyTransform(T, _local[i]);		
		pos.push_back(SIM::Transform::to3dPos(next.pos));
		dir.push_back(SIM::Transform::to3dPos(next.dir));
	}

	return std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> (pos, dir);
}

std::pair<std::string, int> 
MotionSynthesizer:: 
getCurrentTransitionInfo(int _frame)
{
    if(_frame >= mMotionRecord.size())
        _frame = mMotionRecord.size()-1;
    if(mTransitionRecord.size() == 0) 
        return std::pair<std::string, int>(std::string("walk"), _frame);
    int offsetMin = 1e6;
    std::string type = "";
    for(auto t : mTransitionRecord) {
        int start = std::get<0>(t);
        if(start > _frame) continue;
        int offset = _frame - start;
        if(offset < offsetMin) {
            offsetMin = offset;
            type = std::get<2>(t);
        }
    }
    return std::pair<std::string, int>(type, offsetMin);
}

// autoencoder-based motion editing
void
MotionSynthesizer::
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

std::pair<np::ndarray, np::ndarray>
MotionSynthesizer::
convertToAutoEncoderInput(int _recordIdx, bool _fromEditedMotion)
{
    std::tuple<int,int,std::string,InteractionCue> record = mTransitionRecord[_recordIdx];
    std::vector<Eigen::VectorXd> baseMotionRecord = (_fromEditedMotion) ? mEditedMotionRecord : mMotionRecord;

    std::vector<Eigen::VectorXd> motion;
    std::vector<Eigen::VectorXd> targetList;
    
    int start = std::get<0>(record);

    Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(baseMotionRecord[start].head<6>());
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();

    SIM::Pose2d T0Local = SIM::Transform::to2d(T0);
	SIM::Pose2d T1Local = SIM::Transform::to2d(T1);

	Eigen::Isometry3d T01 = SIM::Transform::getGlobalTransform(T0Local, T1Local);
	Eigen::Isometry3d T0new = T01 * T0;

    // should normalize this motion too!
    for(int i = 0; i < std::get<1>(record); i++) {
        Eigen::VectorXd pos = baseMotionRecord[i+start];
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
    for(Contact c : std::get<3>(record).mContactList) {
        Eigen::Isometry3d targetT = Eigen::Isometry3d::Identity();
        targetT.translation() = c.contactPos;
        targetT = T01 * targetT;
        targetT.translation()[1] = c.contactPos[1];
        if(c.jointIdx == 3 || c.jointIdx == 7) // foot
            continue;
        if(std::get<2>(record) == "sit") {
            int frame = motion.size();
            for(int i = 0; i < 10; i++) {
                Eigen::VectorXd target(1+1+3+3);        
                Eigen::Vector3d ori = Eigen::Vector3d::Zero();
                if(c.jointIdx == 0)
                    ori = SIM::logMap(targetT.linear());
                target << (frame-i), c.jointIdx, targetT.translation(), ori;
                std::cout << target.transpose() << std::endl;
                targetList.push_back(target);
            }
        }
    }
    return std::pair<np::ndarray,np::ndarray>(SIM::toNumPyArray(motion), SIM::toNumPyArray(targetList));
}

void 
MotionSynthesizer::
optimizeAutoEncoder(int _recordIdx, bool _fromEditedMotion)
{
    std::pair<np::ndarray, np::ndarray> input = convertToAutoEncoderInput(_recordIdx, _fromEditedMotion);
    std::tuple<int,int,std::string,InteractionCue> record = mTransitionRecord[_recordIdx];
    bool useRootVel = (std::get<2>(record) == "sit") ? true : false;
    int epoch = EDIT_POSTPROCESS_EPOCH;

    try {
        mAutoEncoderOptimize.attr("init_optimize")(mAutoEncoderDir, input.first, input.second);
        // mAutoEncoderOptimize.attr("init_optimize")(mAutoEncoderDir, SIM::toNumPyArray(motion));
        mAutoEncoderOptimize.attr("latent_optimize")(useRootVel, epoch);
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
}

// only for demo
std::vector<Eigen::VectorXd>
MotionSynthesizer:: 
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
    // mLeftFoot = mReferenceManager->getLeftFoot();
    // mRightFoot = mReferenceManager->getRightFoot();
    return cleaned;
}

void
MotionSynthesizer::
setEditedMotion(int _recordIdx, bool _fromEditedMotion)
{
    std::tuple<int,int,std::string,InteractionCue> record = mTransitionRecord[_recordIdx];

    std::vector<Eigen::VectorXd> baseMotionRecord = (_fromEditedMotion) ? mEditedMotionRecord : mMotionRecord;

    std::cout << "original motion record size: " << baseMotionRecord.size() << std::endl;

    mEditedMotionRecord.resize((int)(baseMotionRecord.size()));
    std::copy( baseMotionRecord.begin(), baseMotionRecord.end(), mEditedMotionRecord.begin() );

    std::vector<Eigen::VectorXd> editedMotion;
    try {
        p::object e = mAutoEncoderOptimize.attr("get_logmap")(0);
        np::ndarray ne = np::from_object(e);
        editedMotion = SIM::toEigenVectorVector(ne);
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    
    int start = std::get<0>(record);
    
    std::cout << __func__ << "edited motion starts from: " << start << std::endl;

    // swap
    for(int i = 0; i < std::get<1>(record); i++) {
        Eigen::VectorXd pos = editedMotion[i];
        Eigen::Vector3d rootPos = pos.segment<3>(3);
        pos.segment<3>(3) = pos.segment<3>(0);
        pos.segment<3>(0) = rootPos;
        editedMotion[i] = pos;
    }

    // align root to the original motion !!
    Eigen::Isometry3d T0 = dart::dynamics::FreeJoint::convertToTransform(editedMotion[0].head<6>());
    Eigen::Isometry3d T1 = dart::dynamics::FreeJoint::convertToTransform(baseMotionRecord[start-1].head<6>());

    SIM::Pose2d T0Local = SIM::Transform::to2d(T0);
	SIM::Pose2d T1Local = SIM::Transform::to2d(T1);

	Eigen::Isometry3d T01 = SIM::Transform::getGlobalTransform(T0Local, T1Local);
	Eigen::Isometry3d T0new = T01 * T0;

    for(int i = 0; i < std::get<1>(record); i++) {
        Eigen::VectorXd pos = editedMotion[i];
        Eigen::Isometry3d TCurrent = dart::dynamics::FreeJoint::convertToTransform(pos.head<6>());
		TCurrent = T0.inverse() * TCurrent;
		TCurrent = T0new * TCurrent;
		pos.head<6>() = dart::dynamics::FreeJoint::convertToPositions(TCurrent);

        mEditedMotionRecord[i+start] = pos;
        if(i == 0) {
            // blend
            for(int j = 6; j > 0; j--) {
                double weight = 1 - j / (double)(6+1);
                Eigen::VectorXd oldPos = mEditedMotionRecord[start+i-j];
                mEditedMotionRecord[start+i-j] = SIM::weightedSumPos(oldPos, pos, weight);
            }
        }
    }
}
}