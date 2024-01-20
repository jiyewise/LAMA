#ifndef __ENV_INTERACTION_CUE__
#define __ENV_INTERACTION_CUE__
#include "Transform.h"
#include "dart/dart.hpp"
#include <Eigen/Core>

namespace ENV {


struct Contact
{
    Contact() {};
    Contact(int _jointIdx, Eigen::Vector3d _pos): frame(-1), jointIdx(_jointIdx), contactPos(_pos) {}
    int jointIdx;
    Eigen::Vector3d contactPos;
    SIM::Pose2d contactRootPos; // only for root

    Contact(int _frame, int _jointIdx, Eigen::Vector3d _pos): frame(_frame), jointIdx(_jointIdx), contactPos(_pos) {}
    int frame;
    
};

struct ManipContact
{
    ManipContact() {};
    ManipContact(int _frame): frame(_frame) {}
    int frame;
    Eigen::VectorXd objPos;
};

struct InteractionCue
{
    InteractionCue() {};
    InteractionCue(std::string _type): mType(_type) {}
    std::vector<Contact> mContactList;
    std::string mType;
};

struct ManipInteractionCue
{
    ManipInteractionCue() {};
    ManipInteractionCue(int _jointIdx, int _bnIdx, int _vertex): jointIdx(_jointIdx), bnIdx(_bnIdx), vertex(_vertex) {}
    int jointIdx, bnIdx, vertex;
    dart::dynamics::SkeletonPtr mManipObjSkel;
    Eigen::VectorXd mInitialObjPos;
    std::vector<ManipContact> mContactList;
};

}

#endif