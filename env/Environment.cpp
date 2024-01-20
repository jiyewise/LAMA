#include "Environment.h"
#include <fstream>
#include "SkeletonBuilder.h"
#include "Functions.h"

// typedef tinyxml2::XMLElement TiXmlElement;
// typedef tinyxml2::XMLDocument TiXmlDocument;

namespace ENV
{

Environment::
Environment(std::string _env_path)
{
    parseEnvConfig(_env_path);
}

void
Environment::
parseEnvConfig(std::string _path)
{
    TiXmlDocument doc;
    if(doc.LoadFile(_path.c_str())) {
        std::cout << "Can't open file : " << _path << std::endl;
    }

    TiXmlElement *envdoc = doc.FirstChildElement("Env");

    // name
    std::string envname = envdoc->Attribute("name");
    mName = envname;

    // get scene
    TiXmlElement *scenedoc = envdoc->FirstChildElement("Scene");
    if (scenedoc != nullptr) {
        mScene = new Scene();
        for (TiXmlElement *object = scenedoc->FirstChildElement("Object"); object != nullptr; object = object->NextSiblingElement("Object")){
            // read object xml file and create skeleton
            std::string skelPath = LAMA_DIR + std::string("/data/character/object/") + object->Attribute("file");
            dart::dynamics::SkeletonPtr objSkel = SIM::SkeletonBuilder::buildFromFile(skelPath).first;

            // get transformation information (translation and linear)
            TiXmlElement *objPosElem = object->FirstChildElement("ObjectPosition");
            Eigen::Isometry3d objectPosition;
            objectPosition.setIdentity();

            if(objPosElem->Attribute("linear") != nullptr)
                objectPosition.linear() = SIM::expMapRot(SIM::str2Vec3d(objPosElem->Attribute("linear")));
            if(objPosElem->Attribute("translation") != nullptr)
                objectPosition.translation() = SIM::str2Vec3d(objPosElem->Attribute("translation"));

            int dof = objSkel->getNumDofs();
            Eigen::VectorXd objInitialPos(dof);
            objInitialPos.setZero();
            // TODO remove initialPos attribute?
            if(objPosElem->Attribute("initialPos") != nullptr) {
                if(objPosElem->Attribute("initialPos") != std::string("")) {
                    objInitialPos = SIM::str2VecXd(objPosElem->Attribute("initialPos"));
                }
            }
            // create object
            objInitialPos.head(6) = dart::dynamics::FreeJoint::convertToPositions(objectPosition); // assume that all object root node is freejoint (6 dof)
            objSkel->setPositions(objInitialPos);
            Object obj = Object(objSkel);
            obj.mInitialPos = objSkel->getPositions();
            obj.mFilePath = object->Attribute("file");

            // add object to scene
            mScene->mObjectList.push_back(obj);
        }
    }
    
    mScene->initialize();

    // get interaction cue (TODO add parse cue file)
    TiXmlElement *cuedoc = envdoc->FirstChildElement("Cue");
    if(cuedoc != nullptr) {
        if (cuedoc->Attribute("file") != nullptr) {
            // if file attribute exists, parse file
            if(cuedoc->Attribute("file") != std::string("")) {
                std::string icPath = LAMA_DIR + std::string("/data/ic/") + cuedoc->Attribute("file");
                mInteractionCueList = parseICList(icPath);
            }
        }
        else {
            // if cue (ic) is in original file, pass doc pointer to parseICList
            TiXmlElement *icdoc = cuedoc->FirstChildElement("ICList");
            mInteractionCueList = parseICList(icdoc);
        }            
    }
    // motion database path list
    TiXmlElement *datadoc = envdoc->FirstChildElement("Database");
    if(datadoc != nullptr) {
        for (TiXmlElement *motiondb = datadoc->FirstChildElement("MotionData"); motiondb != nullptr; motiondb = motiondb->NextSiblingElement("MotionData")) 
            mMotionDatabasePathList.push_back(std::pair<std::string, std::string>(motiondb->Attribute("name"), motiondb->Attribute("type")));        
    }

    // initial
    TiXmlElement *initialdoc = envdoc->FirstChildElement("Initial");
    // std::cout << "start parsing initial" << std::endl;
    if(initialdoc != nullptr) {
        Eigen::Vector2d pos = SIM::str2Vec2d(initialdoc->Attribute("pos"));
        mInitial = SIM::Pose2d(pos);
        if(initialdoc->Attribute("dir") != std::string("")) {
            Eigen::Vector2d d = SIM::str2Vec2d(initialdoc->Attribute("dir"));
            mInitial.dir = d;
        }
    }

}

std::vector<InteractionCue> 
Environment::
parseICList(TiXmlElement* _icdoc)
{
    std::vector<InteractionCue> icList;
    for(TiXmlElement *ic = _icdoc->FirstChildElement("IC"); ic != nullptr; ic = ic->NextSiblingElement("IC")){
        std::string type = ic->Attribute("type");
        InteractionCue IC = InteractionCue(type);
        for(TiXmlElement *contact = ic->FirstChildElement("Contact"); contact != nullptr; contact = contact->NextSiblingElement("Contact")) {
            int jointIdx = std::stoi(contact->Attribute("jointIdx")); 
            Eigen::Vector3d contactPos = SIM::str2Vec3d(contact->Attribute("contactPos"));
            Contact c = Contact(jointIdx, contactPos);
            if(contact->Attribute("contactRootPos") != nullptr) {
                // get root
                Eigen::Vector2d rootPos = SIM::str2Vec2d(contact->Attribute("contactRootPos"));
                SIM::Pose2d root2d = SIM::Pose2d(rootPos);
                if(contact->Attribute("contactRootDir") != nullptr) {
                    if(contact->Attribute("contactRootDir") != std::string("")) {
                        Eigen::Vector2d rootDir = SIM::str2Vec2d(contact->Attribute("contactRootDir"));
                        root2d.dir = rootDir;
                    }
                }
                c.contactRootPos = root2d;
            }
            if(contact->Attribute("frame") != nullptr) {
                int frame = std::stoi(contact->Attribute("frame"));
                c.frame = frame;
            }
            IC.mContactList.push_back(c);
        }
        icList.push_back(IC);
    }
    return icList;
}

std::vector<InteractionCue> 
Environment:: 
parseICList(std::string _path)
{
    std::vector<InteractionCue> icList;
    TiXmlDocument doc;
    if(doc.LoadFile(_path.c_str())) {
        std::cout << "Can't open file : " << _path << std::endl;
        return icList;
    }


    TiXmlElement *icListdoc = doc.FirstChildElement("ICList");
    int num = std::stoi(icListdoc->Attribute("num"));
    for(TiXmlElement *ic = icListdoc->FirstChildElement("IC"); ic != nullptr; ic = ic->NextSiblingElement("IC")){
        std::string type = ic->Attribute("type");
        InteractionCue IC = InteractionCue(type);
        for(TiXmlElement *contact = ic->FirstChildElement("Contact"); contact != nullptr; contact = contact->NextSiblingElement("Contact")) {
            int jointIdx = std::stoi(contact->Attribute("jointIdx")); 
            Eigen::Vector3d contactPos = SIM::str2Vec3d(contact->Attribute("contactPos"));
            Contact c = Contact(jointIdx, contactPos);
            if(contact->Attribute("contactRootPos") != nullptr) {
                // get root
                Eigen::Vector2d rootPos = SIM::str2Vec2d(contact->Attribute("contactRootPos"));
                SIM::Pose2d root2d = SIM::Pose2d(rootPos);
                if(contact->Attribute("contactRootDir") != nullptr) {
                    if(contact->Attribute("contactRootDir") != std::string("")) {
                        Eigen::Vector2d rootDir = SIM::str2Vec2d(contact->Attribute("contactRootDir"));
                        root2d.dir = rootDir;
                    }
                }
                c.contactRootPos = root2d;
            }
            if(contact->Attribute("frame") != nullptr) {
                int frame = std::stoi(contact->Attribute("frame"));
                c.frame = frame;
            }
            IC.mContactList.push_back(c);
        }
        icList.push_back(IC);
    }
    return icList;
}

void 
Environment:: 
parseManipIC(std::string _path)
{
    TiXmlDocument doc;
    if(doc.LoadFile(_path.c_str())) {
        std::cout << "Can't open file: " << _path << std::endl;
        return;
    }

    TiXmlElement *manipICdoc = doc.FirstChildElement("ManipIC");
    // set manip IC info (joint idx, bn idx, vertex)
    std::string skelName = manipICdoc->Attribute("obj");
    int jointIdx = std::stoi(manipICdoc->Attribute("jointIdx"));
    int bnIdx = std::stoi(manipICdoc->Attribute("bnIdx"));
    int vertex = std::stoi(manipICdoc->Attribute("vertex"));
    Eigen::Matrix3d linear = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    if(manipICdoc->Attribute("translation") != nullptr)
        translation = SIM::str2Vec3d(manipICdoc->Attribute("translation"));
    if(manipICdoc->Attribute("linear") != nullptr)
        linear = SIM::expMapRot(SIM::str2Vec3d(manipICdoc->Attribute("linear")));

    // load manip object
    std::string skelPath = LAMA_DIR + std::string("/data/character/object/") + skelName;
    dart::dynamics::SkeletonPtr objSkel = SIM::SkeletonBuilder::buildFromFile(skelPath).first;

    // get transformation information (translation and linear)
    Eigen::Isometry3d objectPosition;
    objectPosition.setIdentity();
    objectPosition.linear() = linear;
    objectPosition.translation() = translation;

    int dof = objSkel->getNumDofs();
    Eigen::VectorXd objInitialPos(dof);
    objInitialPos.setZero();
    if(manipICdoc->Attribute("initialPos") != nullptr) {
        if(manipICdoc->Attribute("initialPos") != std::string("")) {
            objInitialPos = SIM::str2VecXd(manipICdoc->Attribute("initialPos"));
        }
    }
    // create object
    objInitialPos.head(6) = dart::dynamics::FreeJoint::convertToPositions(objectPosition); // assume that all object root node is freejoint (6 dof)
    objSkel->setPositions(objInitialPos);
    Object obj = Object(objSkel);
    obj.mInitialPos = objSkel->getPositions();
    
    // set to manip object
    mManipObject = obj;

    // load manip ic
    mManipIC = ManipInteractionCue(jointIdx, bnIdx, vertex);
    mManipIC.mInitialObjPos = obj.mInitialPos;

    // load manip contact list
    for(TiXmlElement *contact = manipICdoc->FirstChildElement("ManipContact"); contact != nullptr; contact = contact->NextSiblingElement("ManipContact")){
        int frame = atoi(contact->Attribute("frame"));
        Eigen::VectorXd objPos = SIM::str2VecXd(contact->Attribute("objPos"));
        // make global (currently root local)
        Eigen::Isometry3d globalObjPos = dart::dynamics::FreeJoint::convertToTransform(objPos.head(6));
        globalObjPos = objectPosition * globalObjPos;
        objPos.head(6) = dart::dynamics::FreeJoint::convertToPositions(globalObjPos);
        ManipContact mc = ManipContact(frame);
        mc.objPos = objPos;
        mManipIC.mContactList.push_back(mc);
    }
}

double
Environment:: 
getICDistance(int _icIdx)
{
    ENV::InteractionCue ic = mInteractionCueList[_icIdx];
    return (ic.mContactList[0].contactRootPos.pos - mInitial.pos).norm();
}

}