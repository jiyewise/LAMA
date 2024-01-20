#ifndef __ENV_MANIP_MOTION_EDITOR_H__
#define __ENV_MANIP_MOTION_EDITOR_H__
#include <string>
#include <Eigen/Core>
#include <vector>
#include <tuple>
#include "dart/dart.hpp"
#include "InteractionCue.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "ReferenceManager.h"
namespace p = boost::python;
namespace np = boost::python::numpy;

namespace ENV
{

class ManipMotionEditor
{
public:
    ManipMotionEditor(dart::dynamics::SkeletonPtr _skel);

    dart::dynamics::SkeletonPtr mSkel;
    SIM::ReferenceManager* mReferenceManager;

    // config
    std::string mAutoEncoderDir;
    int mSeqLength;

    // original and edited motion
    void setOriginalMotion(std::vector<Eigen::VectorXd> _motionRecord);
    void setManipPointInMotion(std::vector<Eigen::VectorXd> _motionRecord);
    std::vector<Eigen::VectorXd> mOriginalMotionRecord;
    std::vector<Eigen::VectorXd> mMotionToEdit;
    std::vector<Eigen::VectorXd> mManipEditedMotionRecord;
    int mStartManipFrame, mEndManipFrame, mEditedMotionLength, mEndEditFrame;

    // TODO add autoencoder
    void loadAutoEncoder(std::string _dir);
    std::pair<np::ndarray, np::ndarray> convertToAutoEncoderInput(std::vector<Contact> _contactList, bool _fromEditedMotion=false);
    void optimizeAutoEncoder(int _manipStart, int _manipEnd, std::vector<Contact> _contactList, bool _fromEditedMotion=false);
    void setEditedMotion(bool _fromEditedMotion=false);
    std::vector<Eigen::VectorXd> getEditedMotion() { return mManipEditedMotionRecord; }
    int getStartManipFrame() { return mStartManipFrame; }
    p::object mAutoEncoderOptimize;

    // postprocess
    std::vector<Eigen::VectorXd> postProcess(std::vector<Eigen::VectorXd> _original);
    std::vector<Eigen::VectorXd> smooth(std::vector<Eigen::VectorXd> _original);


    // IK (ablation)
    void editWithIK(int _manipStart, int _manipEnd, std::vector<Contact> _contactList, bool _fromEditedMotion);
    std::map<int, std::vector<std::pair<std::string, Eigen::Vector3d>>> convertToIKConstraint(std::vector<Contact> _contactList);
};

}

#endif