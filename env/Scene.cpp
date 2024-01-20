#include "Scene.h"
#include "Functions.h"
#include "EnvConfigurations.h"

using namespace dart::dynamics;
using namespace dart::simulation;
// std::mutex mLock;
namespace ENV {

Object::
Object(dart::dynamics::SkeletonPtr _obj_skel)
{
    mObjectSkel = _obj_skel;
    loadTrimesh();
    applyTransformAll();
}

void 
Object::
loadTrimesh()
{
    for(int bnIdx = 0; bnIdx < mObjectSkel->getNumBodyNodes(); bnIdx++) {
        auto bn = mObjectSkel->getBodyNode(bnIdx);

        // get mesh from bodynode
        auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
        auto T = shapeNodes[0]->getTransform();
        auto* meshShape = dynamic_cast<const MeshShape*>(shapeNodes[0]->getShape().get());
        const aiScene* meshScene = meshShape->getMesh();
        if(!meshScene) {
            std::cout << "cannot find mesh scene" << std::endl;
        }
        const struct aiMesh* mesh = meshScene->mMeshes[meshScene->mRootNode->mMeshes[0]];

        // convert mesh mVertices and mFaces
        std::vector<Eigen::VectorXd> vertices;
        std::vector<Eigen::VectorXd> faces;

        int numVertices = mesh->mNumVertices;
        int numFaces = mesh->mNumFaces;

        // vertices
        for(int i = 0; i < numVertices; i++){
            Eigen::VectorXd v(3);
            v(0) = mesh->mVertices[i].x; 
            v(1) = mesh->mVertices[i].y;
            v(2) = mesh->mVertices[i].z;
            vertices.push_back(v);
        }

        // faces
        for(int i = 0; i < numFaces; i++) {
            // check whether indices are 3 (should be triangular mesh)
            int num_indices = mesh->mFaces[i].mNumIndices;
            if(num_indices != 3) {
                std::cout << "num indices should be 3. Current num indices: " << num_indices << std::endl;
                exit(0);
            }
            Eigen::VectorXd face(num_indices);
            for(int j = 0; j < num_indices; j++) {
                face[j] = mesh->mFaces[i].mIndices[j];
            }
            faces.push_back(face);
        }

        // connect to python
        Py_Initialize();
        np::initialize();
        try {
            p::object sys_module = p::import("sys");
            p::str module_dir = (std::string(LAMA_DIR) + "/mesh").c_str();
            sys_module.attr("path").attr("insert")(1, module_dir);

            p::object mesh_helper_main = p::import("TrimeshHelper");
            p::object trimesh = mesh_helper_main.attr("TrimeshHelper")(SIM::toNumPyArray(vertices), SIM::toNumPyArray(faces));
            mTrimesh.push_back(std::pair<int,p::object>(bnIdx, trimesh));
        } catch (const p::error_already_set&) {
            PyErr_Print();
        }
    }
}

void 
Object::
applyTransform(int _bnIdx)
{
    // first get transform of the skel node 
    auto bn = mObjectSkel->getBodyNode(_bnIdx); // assume that there is only one node (ground) for scene
    auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
    int type = 0;
    auto T = shapeNodes[type]->getTransform();

    Eigen::MatrixXd transform(4, 4);
    transform.setIdentity();
    transform.block<3,3>(0,0) = T.linear();
    transform.block<3,1>(0,3) = T.translation();

    std::pair<int, p::object> trimeshPair = mTrimesh[_bnIdx];
    if(trimeshPair.first != _bnIdx) {
        std::cout << "different body node index. Should be: " << trimeshPair.first << " input: " << _bnIdx << std::endl;
        exit(0);
    }
    try {
        trimeshPair.second.attr("transform")(SIM::toNumPyArray(transform));
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
}

void 
Object::
applyTransformAll()
{
    for(int i = 0; i < mObjectSkel->getNumBodyNodes(); i++) {
        applyTransform(i);
    }
}

std::pair<int, p::object>
Object::
loadTrimeshPair(int _bnIdx)
{
    std::pair<int, p::object> trimeshPair = mTrimesh[_bnIdx];
    if(trimeshPair.first != _bnIdx) {
        std::cout << "different body node index. Should be: " << trimeshPair.first << " input: " << _bnIdx << std::endl;
        exit(0);
    }
    return trimeshPair;    
}

Eigen::Vector3d
Object::
getVertexPos(int _bnIdx, int _vIdx)
{
    std::pair<int, p::object> trimeshPair = loadTrimeshPair(_bnIdx);
    Eigen::Vector3d vertexPos;
    try {
        p::object v = trimeshPair.second.attr("get_vertex")(_vIdx);    
        np::ndarray nv = np::from_object(v);
        vertexPos = SIM::toEigenVector(nv, 3);
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }

    return vertexPos;
}

bool 
Object::
checkIntersect(int _bnIdx, Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose)
{
    std::pair<int, p::object> trimeshPair = loadTrimeshPair(_bnIdx);
    bool intersect = false;
    try {
        p::object b = trimeshPair.second.attr("check_intersection")(SIM::toNumPyArray(_ray_origin), SIM::toNumPyArray(_ray_dir));
        intersect = boost::python::extract<bool>(b);
        if(intersect) {
            // get number of intersecting points
            p::object num = trimeshPair.second.attr("get_num_intersection_point")();
            int numPoint = p::extract<int>(num);
            // get location of intersecting points
            p::object pos = trimeshPair.second.attr("get_intersection_point")();
            np::ndarray npos = np::from_object(pos);
            Eigen::MatrixXd posMat = SIM::toEigenMatrix(npos, numPoint, 3); // could be many points for a single body node!
            // get faces
            p::object faces = trimeshPair.second.attr("get_intersection_face")();
            np::ndarray nfaces = np::from_object(faces);
            Eigen::MatrixXd faceMat = SIM::toEigenMatrix(nfaces, numPoint, 1);

            for(int i = 0; i < numPoint; i++) {
                Eigen::Vector3d pos = posMat.row(i);
                if(_verbose) {
                    int faceNum = (int)(faceMat.row(i)[0]);
                    // get offset from face center
                    p::object faceCenter = trimeshPair.second.attr("get_face_center")(faceNum);
                    np::ndarray nFaceCenter = np::from_object(faceCenter);
                    Eigen::Vector3d faceOffset = pos - SIM::toEigenVector(nFaceCenter, 3); 
                    // set intersect point
                    mPointListObj.push_back(IntersectPoint(_bnIdx, pos, faceNum, faceOffset));
                } else {
                    mPointListObj.push_back(IntersectPoint(pos));
                }
            }
        }
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    return intersect;
}

void
Object::
checkIntersectAll(Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose)
{
    mPointListObj.clear();
    for(int i = 0; i < mObjectSkel->getNumBodyNodes(); i++) {
        bool intersectPerNode = checkIntersect(i, _ray_origin, _ray_dir, _verbose); 
    }
    return;
}

Eigen::Vector3d
Object::
getFaceCenter(int _bnIdx, int _faceNum)
{
    Eigen::Vector3d faceCenter;
    std::pair<int, p::object> trimeshPair = loadTrimeshPair(_bnIdx);
    try {
        p::object pfaceCenter = trimeshPair.second.attr("get_face_center")(_faceNum);
        np::ndarray nFaceCenter = np::from_object(pfaceCenter);
        faceCenter = SIM::toEigenVector(nFaceCenter, 3); 

    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    return faceCenter;
}


Scene::
Scene()
 :mGridResolution(100), mOccupancyRange(5), mBuildSceneHelper(false), mRD(), mMT(mRD()), mUniform(0.0, 1.0)
{}

Scene::
Scene(std::vector<Object> _obj_list)
 :Scene()
{
    mObjectList = _obj_list;
}

void 
Scene:: 
updateScene(int _objIdx, Eigen::VectorXd _objPos)
{
    mObjectList[_objIdx].mObjectSkel->setPositions(_objPos);
    mObjectList[_objIdx].applyTransformAll();
    integrateAllMesh(true);
}

void
Scene::
setIntersectInfo(Eigen::Vector3d _ray_origin, Eigen::Vector3d _ray_dir, bool _verbose)
{
    mPointListScene.clear();
    for(int i = 0; i < mObjectList.size(); i++) {
        mObjectList[i].checkIntersectAll(_ray_origin, _ray_dir, _verbose);
        std::vector<IntersectPoint> list = mObjectList[i].mPointListObj;
        for(int j = 0; j < list.size(); j++) {
            IntersectPoint p = list[j];
            p.objIdx = i;
            mPointListScene.push_back(p);
        }
    }
}

void 
Scene::
initialize()
{
    bool isFirst = !mBuildSceneHelper;
    if(isFirst) {
        try {
            p::object sys_module = p::import("sys");
            p::str module_dir = (std::string(LAMA_DIR) + "/mesh").c_str();
            sys_module.attr("path").attr("insert")(1, module_dir);

            p::object mesh_helper_main = p::import("SceneHelper");
            sceneHelper = mesh_helper_main.attr("SceneHelper")();
            mBuildSceneHelper = true;
        } catch (const p::error_already_set&) {
                PyErr_Print();
        }  
    }
    for(auto obj : mObjectList) {
        obj.mObjectSkel->setPositions(obj.mInitialPos);
        obj.applyTransformAll();
    }
    integrateAllMesh(!isFirst); // isfirst false -> update is true
}

void 
Scene::
integrateAllMesh(bool _update)
{
    if(_update) {
        try {
            sceneHelper.attr("clear_all_mesh")();
        } catch (const p::error_already_set&) {
                PyErr_Print();
        }
    }

    for(auto obj : mObjectList) {
        try {
        for(auto meshPair : obj.mTrimesh) 
            sceneHelper.attr("add_mesh")(meshPair.second);
        } catch (const p::error_already_set&) {
            PyErr_Print();
        }
    }
    // integrate mesh
    try {
        sceneHelper.attr("integrate_all_mesh")();
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    // build grid
    buildGrid();
}

std::vector<Eigen::Vector3d>
Scene::
checkPenetrationByBodyNodeList(std::vector<dart::dynamics::BodyNode*> _bnList, double _heightLimit)
{
    std::vector<Eigen::Vector3d> intersectingPoints;
    std::vector<Eigen::Vector3d> bnSizeList;
    std::vector<Eigen::Isometry3d> bnTransformList;
    std::vector<Eigen::Vector3d> ray_origin;
    std::vector<Eigen::Vector3d> ray_dir;

    for(auto bn : _bnList) {
        auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
        Eigen::Isometry3d transform = shapeNodes[0]->getTransform();
        Eigen::Vector3d size = dynamic_cast<const dart::dynamics::BoxShape*>(shapeNodes[0]->getShape().get())->getSize();
        bnSizeList.push_back(size);
        bnTransformList.push_back(transform);
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < 2; j++) {
                double x = (2*i-1) * size(0) * 0.5;
                double z = (2*j-1) * size(2) * 0.5;
                Eigen::Vector3d point(x, size(1)*0.5, z);
                Eigen::Vector3d pointLow(x, -1*size(1)*0.5, z);
                Eigen::Vector3d dir = (pointLow - point).normalized();
                ray_origin.push_back(transform * point);
                ray_dir.push_back(transform.linear() * dir);
            }
        }
    }
    mLock.lock();
    try {
        p::object b = sceneHelper.attr("scene_check_intersection")(SIM::toNumPyArray(ray_origin), SIM::toNumPyArray(ray_dir));
        bool intersect = boost::python::extract<bool>(b);

        if(intersect) {
            p::object num = sceneHelper.attr("get_num_intersection_point")();
            int numPoint = p::extract<int>(num);

            p::object pos = sceneHelper.attr("get_intersection_point")();
            np::ndarray npos = np::from_object(pos);
            Eigen::MatrixXd posMat = SIM::toEigenMatrix(npos, numPoint, 3);

            for(int i = 0; i < numPoint; i++) {
                Eigen::Vector3d pos = posMat.row(i);
                if(pos[1] < _heightLimit)
                    continue;
                for(int j = 0; j < bnSizeList.size(); j++) {
                    bool withinBox = SIM::checkWithinBox(pos, bnSizeList[j], bnTransformList[j]);
                    if(withinBox) intersectingPoints.push_back(pos);
                }
            }
        }
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    mLock.unlock();
    return intersectingPoints;    

}


std::vector<Eigen::Vector3d>
Scene::
checkPenetrationByBodyNodeList(std::vector<Eigen::Vector3d> ray_origin, std::vector<Eigen::Vector3d> ray_dir, 
                               std::vector<Eigen::Vector3d> bnSizeList, std::vector<Eigen::Isometry3d> bnTransformList, 
                               double _heightLimit)
{
    std::vector<Eigen::Vector3d> intersectingPoints;

    try {
        p::object b = sceneHelper.attr("scene_check_intersection")(SIM::toNumPyArray(ray_origin), SIM::toNumPyArray(ray_dir));
        bool intersect = boost::python::extract<bool>(b);

        if(intersect) {
            p::object num = sceneHelper.attr("get_num_intersection_point")();
            int numPoint = p::extract<int>(num);

            p::object pos = sceneHelper.attr("get_intersection_point")();
            np::ndarray npos = np::from_object(pos);
            Eigen::MatrixXd posMat = SIM::toEigenMatrix(npos, numPoint, 3);

            for(int i = 0; i < numPoint; i++) {
                Eigen::Vector3d pos = posMat.row(i);
                if(pos[1] < _heightLimit)
                    continue;
                for(int j = 0; j < bnSizeList.size(); j++) {
                    bool withinBox = SIM::checkWithinBox(pos, bnSizeList[j], bnTransformList[j]);
                    if(withinBox) 
                        intersectingPoints.push_back(pos);
                }
            }
        }
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    return intersectingPoints;    

}

std::vector<Eigen::Vector3d>
Scene::
checkPenetrationByRay(std::pair<Eigen::Vector3d, Eigen::Vector3d> _ray)
{
    std::vector<Eigen::Vector3d> intersectingPoints;
    std::vector<Eigen::Vector3d> ray_origin; 
    std::vector<Eigen::Vector3d> ray_dir;
    ray_origin.push_back(_ray.first);
    ray_dir.push_back(_ray.second);

    try {
        p::object b = sceneHelper.attr("scene_check_intersection")(SIM::toNumPyArray(ray_origin), SIM::toNumPyArray(ray_dir));
        bool intersect = boost::python::extract<bool>(b);

        if(intersect) {
            p::object num = sceneHelper.attr("get_num_intersection_point")();
            int numPoint = p::extract<int>(num);

            p::object pos = sceneHelper.attr("get_intersection_point")();
            np::ndarray npos = np::from_object(pos);
            Eigen::MatrixXd posMat = SIM::toEigenMatrix(npos, numPoint, 3);
            
            for(int i = 0; i < numPoint; i++) {
                Eigen::Vector3d pos = posMat.row(i);
                if((pos(1) > _ray.first(1)) && (_ray.second(1) < 0))
                    continue;
                if((pos(1) < _ray.first(1)) && (_ray.second(1) > 0))
                    continue;
                intersectingPoints.push_back(pos);
            }
        }
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    return intersectingPoints;
}


std::vector<Eigen::Vector3d>
Scene::
checkPenetrationByBodyNode(dart::dynamics::BodyNode* _bn)
{
    std::vector<Eigen::Vector3d> intersectingPoints;
    auto shapeNodes = _bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
    Eigen::Isometry3d transform = shapeNodes[0]->getTransform();
    Eigen::Vector3d size = dynamic_cast<const dart::dynamics::BoxShape*>(shapeNodes[0]->getShape().get())->getSize();

    std::vector<Eigen::Vector3d> ray_origin;
    std::vector<Eigen::Vector3d> ray_dir;
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            double x = (2*i-1) * size(0) * 0.5;
            double z = (2*j-1) * size(2) * 0.5;
            Eigen::Vector3d point(x, size(1)*0.5, z);
            Eigen::Vector3d pointLow(x, -1*size(1)*0.5, z);
            Eigen::Vector3d dir = (pointLow - point).normalized();
            ray_origin.push_back(transform * point);
            ray_dir.push_back(transform.linear() * dir);
        }
    }
    mLock.lock();
    try {
        p::object b = sceneHelper.attr("scene_check_intersection")(SIM::toNumPyArray(ray_origin), SIM::toNumPyArray(ray_dir));
        bool intersect = boost::python::extract<bool>(b);

        if(intersect) {
            p::object num = sceneHelper.attr("get_num_intersection_point")();
            int numPoint = p::extract<int>(num);

            p::object pos = sceneHelper.attr("get_intersection_point")();
            np::ndarray npos = np::from_object(pos);
            Eigen::MatrixXd posMat = SIM::toEigenMatrix(npos, numPoint, 3);

            for(int i = 0; i < numPoint; i++) {
                Eigen::Vector3d pos = posMat.row(i);
                if(pos[1] < 0)
                    continue;
                // check whether the pos is within the box
                Eigen::Vector3d localPos = transform.inverse() * pos;
                if(localPos[0] > (size(0)*0.5+0.03) || localPos[0] < (-1*size(0)*0.5-0.03))
                    continue;
                if(localPos[1] > (size(1)*0.5+0.03) || localPos[1] < (-1*size(1)*0.5-0.03))
                    continue;
                if(localPos[2] > (size(2)*0.5+0.03) || localPos[2] < (-1*size(2)*0.5-0.03))
                    continue;
                // save intersecting points
                intersectingPoints.push_back(pos);
            }
        }
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    mLock.unlock();
    return intersectingPoints;
}

void 
Scene::
buildGrid()
{
    // compute grid
    try {
        p::object g = sceneHelper.attr("build_height_grid")(mGridResolution, mGridResolution);
        np::ndarray ng = np::from_object(g);
        mGrid = SIM::toEigenMatrix(ng, mGridResolution, mGridResolution);
    } catch (const p::error_already_set&) {
        PyErr_Print();
    }

    // get bbox
    p::object b = sceneHelper.attr("get_bbox")();
    np::ndarray nb = np::from_object(b);
    mBBox = SIM::toEigenMatrix(nb, 2, 3);

    // build grid size
    mGridSizeX = (mBBox(1, 0) - mBBox(0, 0)) / (double) mGrid.rows(); 
    mGridSizeZ = (mBBox(1, 2) - mBBox(0, 2)) / (double) mGrid.cols(); 

}

bool 
Scene:: 
checkRootOccupied(Eigen::Vector2d _world, bool _considerOcc)
{
    std::pair<int, int> gridCoord = getGridCoord(_world);
    // if(gridCoord.first < 0 || gridCoord.second < 0 || gridCoord.first >= mGridResolution || gridCoord.second >= mGridResolution)
    //     return true;
    if(_considerOcc && (mGrid(gridCoord.first, gridCoord.second) > 0.15))
        return true;
    if(_world[0] > mBBox(1,0)*BBOX_SIZE || _world[0] < mBBox(0,0)*BBOX_SIZE || _world[1] > mBBox(1,2)*BBOX_SIZE || _world[1] < mBBox(0,2)*BBOX_SIZE)
        return true;
    return false;
}

bool
Scene::
checkCanStand(Eigen::Vector2d _world)
{
    int offset = mOccupancyRange/2;
    std::pair<int, int> gridCoord = getGridCoord(_world);
    for(int i = 0; i < mOccupancyRange; i++) {
        for(int j = 0; j < mOccupancyRange; j++) {
            int newX = gridCoord.first - offset + i;
            int newZ = gridCoord.second - offset + j;
            if(newX < 0 || newZ < 0 || newX >= mGridResolution || newZ >= mGridResolution)
                return false;
            if(mGrid(newX, newZ) > 0)
                return false;
        }
    }
    return true;
}

bool 
Scene:: 
checkCanStand(std::pair<int,int> _gridCoord) {
    int range = 10;
    int offset = range/2;
    for(int i = 0; i < range; i++) {
        for(int j = 0; j < range; j++) {
            int newX = _gridCoord.first - offset + i;
            int newZ = _gridCoord.second - offset + j;
            if(newX < 0 || newZ < 0 || newX >= mGridResolution || newZ >= mGridResolution)
                return false;
            if(mGrid(newX, newZ) > 0)
                return false;
        }
    }
    return true;
}

Eigen::Vector2d 
Scene:: 
randomSampleFromGrid()
{
    int x = std::floor(mUniform(mMT)*mGridResolution);
    int z = std::floor(mUniform(mMT)*mGridResolution);

    std::pair<int,int> gridCoord = std::pair<int,int>(x,z);
    Eigen::Vector2d worldCoord = getWorldCoord(gridCoord);
    return worldCoord;
}

Eigen::Vector2d 
Scene:: 
randomSampleInitial()
{
    while(true){
        Eigen::Vector2d random = randomSampleFromGrid();
        if(checkCanStand(random))
            return random;
    }
}

std::vector<Eigen::Vector2d> 
Scene:: 
uniformSampleInitial()
{
    std::vector<Eigen::Vector2d> possibleInitialPoints;
    for(int i = 0; i < mGridResolution; i++) {
        for(int j = 0; j < mGridResolution; j++) {
            std::pair<int,int> gridCoord = std::pair<int,int>(i,j);
            if(checkCanStand(gridCoord)) {
                Eigen::Vector2d worldCoord = getWorldCoord(gridCoord);
                possibleInitialPoints.push_back(worldCoord); 
            }
        }
    }
    return possibleInitialPoints;
}


Eigen::VectorXd
Scene::
getGridOccupancy(Eigen::Vector2d _world)
{
    int offset = mOccupancyRange/2;
    std::pair<int, int> gridCoord = getGridCoord(_world);
    Eigen::VectorXd occupancy(mOccupancyRange*mOccupancyRange);
    occupancy.setZero();
    int count = 0;
    for(int i = 0; i < mOccupancyRange; i++) {
        for(int j = 0; j < mOccupancyRange; j++) {
            int newX = gridCoord.first - offset + i;
            int newZ = gridCoord.second - offset + j;
            if(newX < 0 || newZ < 0 || newX >= mGridResolution || newZ >= mGridResolution)
                occupancy[count] = 1;
            if(mGrid(newX, newZ) > 0)
                occupancy[count] = 1;
            count += 1;
        }
    }
    return occupancy;
}

std::pair<int,int>
Scene::
getGridCoord(Eigen::Vector2d _world)
{

    int X = std::floor((_world(0) - mBBox(0,0)) / mGridSizeX);
    int Z = std::floor((_world(1) - mBBox(0,2)) / mGridSizeZ);

    return {X, Z};    
}

Eigen::Vector2d 
Scene::
getWorldCoord(std::pair<int, int> _gridCoord)
{
    // world coordinates of the center of the grid
    double x = mBBox(0,0) +  ((double) _gridCoord.first + 0.5)*mGridSizeX;
    double z = mBBox(0,2) +  ((double) _gridCoord.second + 0.5)*mGridSizeZ;

    Eigen::Vector2d world(x, z);
    return world;
}

std::vector<Eigen::Vector3d>
Scene::
checkPenetration(std::vector<Eigen::Vector3d> _points)
{
    Eigen::VectorXd result;
    std::vector<Eigen::Vector3d> penetratedPoints;
    try {
        p::object po = sceneHelper.attr("check_point_in_mesh")(SIM::toNumPyArray(_points));
        np::ndarray npo = np::from_object(po);
        result = SIM::toEigenVector(npo, _points.size());
    } catch (const p::error_already_set&) {
            PyErr_Print();
    }
    for(int i = 0; i < _points.size(); i++) {
        if(result[i] < 0.01) // reverse
            penetratedPoints.push_back(_points[i]);
    }
    return penetratedPoints;
}

}