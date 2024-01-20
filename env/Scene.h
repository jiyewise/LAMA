#ifndef __ENV_SCENE__
#define __ENV_SCENE__
#include <iostream>
#include <tuple>
#include "dart/dart.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"

#include <Eigen/Core>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <mutex>
namespace p = boost::python;
namespace np = boost::python::numpy;

namespace ENV {

struct IntersectPoint
{
    IntersectPoint(Eigen::Vector3d _pos): pos(_pos) {}
    IntersectPoint(int _bnIdx, Eigen::Vector3d _pos, int _faceNum, Eigen::Vector3d _faceOffset): bnIdx(_bnIdx), pos(_pos), faceNum(_faceNum), faceCenterOffset(_faceOffset) {}
    // related to manip
    int objIdx, bnIdx, faceNum;
    Eigen::Vector3d pos;
    Eigen::Vector3d faceCenterOffset;
};

class Object 
{
public:
    Object() {};
    Object(dart::dynamics::SkeletonPtr _obj_skel);
    dart::dynamics::SkeletonPtr mObjectSkel;

    // add functions to get mesh information
    void loadTrimesh();
    void applyTransform(int _bnIdx);
    void applyTransformAll();
    Eigen::Vector3d getVertexPos(int _bnIdx, int _vIdx);
    std::pair<int, p::object> loadTrimeshPair(int _bnIdx);
    bool checkIntersect(int _bnIdx, Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose);
    void checkIntersectAll(Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose);
    Eigen::Vector3d getFaceCenter(int _bnIdx, int _faceNum);    

    std::vector<std::pair<int, p::object>> mTrimesh; // list of python trimesh (mesh and bodynode index pair for each bodynode of skel)
    std::vector<IntersectPoint> mPointListObj;
    Eigen::VectorXd mInitialPos;
    std::string mFilePath;
};

class Scene 
{
public:
    Scene();
    Scene(std::vector<Object> _obj_list);

    Eigen::Vector3d getObjectVertex(int _objIdx, int _bnIdx, int _vIdx);
    void setIntersectInfo(Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose=true);

    void integrateAllMesh(bool _update=false);
    p::object sceneHelper;
    std::vector<Eigen::Vector3d> checkPenetration(std::vector<Eigen::Vector3d> _points);
    std::vector<Eigen::Vector3d> checkPenetrationByBodyNode(dart::dynamics::BodyNode* _bn);
    std::vector<Eigen::Vector3d> checkPenetrationByBodyNodeList(std::vector<dart::dynamics::BodyNode*> _bnList, double _heightLimit);
    std::vector<Eigen::Vector3d> checkPenetrationByBodyNodeList(std::vector<Eigen::Vector3d> ray_origin, std::vector<Eigen::Vector3d> ray_dir, 
                               std::vector<Eigen::Vector3d> bnSizeList, std::vector<Eigen::Isometry3d> bnTransformList, 
                               double _heightLimit=0.1);
    std::vector<Eigen::Vector3d> checkPenetrationByRay(std::pair<Eigen::Vector3d, Eigen::Vector3d> _ray);
    void updateScene(int _objIdx, Eigen::VectorXd _objPos);

    void initialize();
    bool mBuildSceneHelper;

    std::vector<Object> mObjectList;
    std::vector<IntersectPoint> mPointListScene;

    // 2D grid of the whole scene
    void buildGrid();
    bool checkRootOccupied(Eigen::Vector2d _world, bool _considerOcc=false);
    Eigen::VectorXd getGridOccupancy(Eigen::Vector2d _world);
    std::pair<int,int> getGridCoord(Eigen::Vector2d _world);
    Eigen::Vector2d getWorldCoord(std::pair<int,int> _gridCoord);

    // parametric
    bool checkCanStand(std::pair<int,int> _gridCoord);
    bool checkCanStand(Eigen::Vector2d _world);
    Eigen::Vector2d randomSampleFromGrid();  
    Eigen::Vector2d randomSampleInitial();  
    std::vector<Eigen::Vector2d> uniformSampleInitial();

    // random
	std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;

    Eigen::MatrixXd mGrid, mBBox;
    double mGridSizeX, mGridSizeZ;
    int mGridResolution;
    int mOccupancyRange;

    std::mutex mLock;
};

}

#endif