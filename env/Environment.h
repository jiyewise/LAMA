#ifndef __ENV_ENVIRONMENT__
#define __ENV_ENVIRONMENT__
#include "InteractionCue.h"
#include "Scene.h"
#include "Transform.h"
#include "SkeletonBuilder.h"
#include "MotionSynthesizer.h"
#include "dart/dart.hpp"
#include <tinyxml2.h>

typedef tinyxml2::XMLElement TiXmlElement;
typedef tinyxml2::XMLDocument TiXmlDocument;

namespace ENV {

class Environment 
{
public:
    Environment(std::string _env_path);
    void parseEnvConfig(std::string _path);
    std::vector<InteractionCue> parseICList(std::string _path);
    std::vector<InteractionCue> parseICList(TiXmlElement* _icdoc);
    double getICDistance(int _icIdx);

    Scene* mScene;
    std::vector<InteractionCue> mInteractionCueList;
    std::vector<std::pair<std::string, std::string>> mMotionDatabasePathList; // change to map
    SIM::Pose2d mInitial; // starting pose of human

    // manip related
    ManipInteractionCue mManipIC;
    Object mManipObject;
    void parseManipIC(std::string _path);

    // for re-printing into xml file
    std::string mName;

};

}

#endif