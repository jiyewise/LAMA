#ifndef __ENV_MOTION_SYNTHESIZER_H__
#define __ENV_MOTION_SYNTHESIZER_H__
#include <string>
#include <Eigen/Core>
#include <vector>
#include <tuple>
#include "dart/dart.hpp"
#include "MotionDatabase.h"
#include "SitMotionDatabase.h"
#include "InteractionCue.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;

namespace ENV
{

struct ActionFeature
{
    ActionFeature(): type(std::string("walk")), isEdit(false) {}
    std::string type;
    std::vector<SIM::Pose2d> targetTrajectory; // for loco
    Eigen::VectorXd posOffset;
    bool useInteractionCue;
    bool isEdit;
};

class MotionSynthesizer
{
public:
    MotionSynthesizer(dart::dynamics::SkeletonPtr _skel, std::map<std::string, std::string> _database_folder);
    
    dart::dynamics::SkeletonPtr mSkel;
    SIM::ReferenceManager* mReferenceManager;
    SIM::MotionDatabase* mMotionDatabase;
    SIM::SitMotionDatabase* mSitMotionDatabase;

    std::vector<Eigen::VectorXd> mMotionRecord;
    std::vector<std::vector<SIM::Pose2d>> mTrajectoryRecord;

    void initialize(bool _isCustom=false, SIM::Pose2d _startPose=SIM::Pose2d(), bool _keepCustomRecord=false);
    std::pair<bool,std::map<std::string, double>> step(ActionFeature _actionFeature, InteractionCue _interactionCue); 
    std::pair<Eigen::VectorXd, std::vector<SIM::Pose2d>> getCurrentMotion();
    bool checkSearchNewMotion(ActionFeature _actionFeature);
    std::tuple<bool, std::vector<Eigen::VectorXd>, std::vector<std::vector<SIM::Pose2d>>> searchNewMotion(ActionFeature _actionFeature, std::vector<SIM::Pose2d> _localTarget);
    std::vector<SIM::Pose2d> extractTarget(ActionFeature _actionFeature, InteractionCue _interactionCue);
    bool checkActionComplete();
    Eigen::Vector3d checkRootPos(int _interval);
    double checkRootVel(int _interval);
    std::map<std::string, double> getTransitionCostRecord(std::string _actionFeatureType);
    std::pair<std::string, int> getCurrentTransitionInfo(int _frame); // return current transition type and local frame
    void clearTransitionRecord() { mTransitionRecord.clear(); }

    // TODO add autoencoder
    std::pair<np::ndarray, np::ndarray> convertToAutoEncoderInput(int _recordIdx, bool _fromEditedMotion=false);
    void loadAutoEncoder(std::string _dir);
    void optimizeAutoEncoder(int _recordIdx, bool _fromEditedMotion=false);
    void setEditedMotion(int _recordIdx, bool _fromEditedMotion=false);

    std::vector<std::tuple<int,int,std::string,InteractionCue>> mTransitionRecord; // start frame, duration, action type, ic
    std::string mAutoEncoderDir;
    p::object mAutoEncoderOptimize;
    std::vector<Eigen::VectorXd> mEditedMotionRecord;

    // trajectory
    std::vector<SIM::Pose2d> generateTargetTrajectory(Eigen::Vector3d _world, bool _localPos=false);
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> getWorldTrajectory(std::vector<SIM::Pose2d> _local);
    ActionFeature mCurActionFeature;    
    int mCurIdx;

    // post process for demo
    std::vector<Eigen::VectorXd> postProcess(std::vector<Eigen::VectorXd> _original);
    
};

}

#endif