#ifndef __SIM_FUNCTIONS_H__
#define __SIM_FUNCTIONS_H__

#include "BVHParser.h"
#include "dart/dart.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;

namespace SIM
{
    Eigen::Matrix3d eulerX2Rot(double _x);
	Eigen::Matrix3d eulerY2Rot(double _y);
	Eigen::Matrix3d eulerZ2Rot(double _z);
	Eigen::Vector3d logMap(Eigen::Matrix3d _mat);
	Eigen::Matrix3d makeSkewSymmetric(Eigen::Vector3d _v);
	Eigen::Matrix3d expMapRot(Eigen::Vector3d _q);
	Eigen::Vector3d proj(Eigen::Vector3d _u, Eigen::Vector3d _v);	
	Eigen::Matrix3d orthonormalize(Eigen::Matrix3d _mat);
	Eigen::Vector3d quat2Pos(Eigen::Quaterniond _q);
	Eigen::Quaterniond pos2Quat(Eigen::Vector3d _in);
	void quatNormalize(Eigen::Quaterniond& in);
	Eigen::VectorXd weightedSumPosClip(Eigen::VectorXd _target, Eigen::VectorXd _source, double _weight, Eigen::VectorXd _prevPos, bool _blendRoot=true);
	Eigen::VectorXd weightedSumPos(Eigen::VectorXd _target, Eigen::VectorXd _source, double _weight, bool _blendRoot=true);
	Eigen::VectorXd weightedSumVec(Eigen::VectorXd _target, Eigen::VectorXd _source, double _weight);
	Eigen::Vector3d getPosDiff(Eigen::Vector3d _p1, Eigen::Vector3d _p0);
	Eigen::Vector3d getPosDiffXZplane(Eigen::Vector3d _p1, Eigen::Vector3d _p0);
	double expOfSquared(Eigen::VectorXd _vec, double _sigma = 1.0);

	Eigen::Vector3d rotate(Eigen::Vector3d _q0, Eigen::Vector3d _q1);
	Eigen::Vector3d rotateVector(Eigen::Quaterniond _q, Eigen::Vector3d _v);
	Eigen::Vector3d jointPositionDifferences(Eigen::Vector3d _q1, Eigen::Vector3d _q0);
	
	Eigen::VectorXd addDisplacement(Eigen::VectorXd _p, Eigen::VectorXd _d);
	Eigen::VectorXd subDisplacement(Eigen::VectorXd _p0, Eigen::VectorXd _p1);

	Eigen::VectorXd convertNodeOrder(Eigen::VectorXd _pos, dart::dynamics::SkeletonPtr _targetSkel, std::string _sourceSkel);
	Eigen::VectorXd convertNodeOrder(Eigen::VectorXd _pos, dart::dynamics::SkeletonPtr _targetSkel, std::vector<std::pair<std::string, int>> _idxList);
	
	//str related
	std::string vec2Str(Eigen::VectorXd _vec);
	Eigen::VectorXd str2VecXd(std::string _str);
	std::vector<double> str2Double(std::string _str);
	Eigen::Matrix3d str2Mat3d(std::string _str);
	Eigen::Vector3d str2Vec3d(std::string _str);
	Eigen::Vector2d str2Vec2d(std::string _str);
	std::vector<std::string> split (std::string _s, std::string _delimiter);

	//boost python
	np::ndarray toNumPyArray(Eigen::VectorXd _vec);
	np::ndarray toNumPyArray(Eigen::Vector3d _vec);
	np::ndarray toNumPyArray(std::vector<double> _vec);
	np::ndarray toNumPyArray(Eigen::MatrixXd _mat);
	np::ndarray toNumPyArray(std::vector<Eigen::VectorXd> _mat);
	np::ndarray toNumPyArray(std::vector<Eigen::Vector3d> _mat);
	np::ndarray toNumPyArray(std::vector<std::vector<double>> _mat);
	Eigen::VectorXd toEigenVector(np::ndarray _array, int _n);
	Eigen::MatrixXd toEigenMatrix(np::ndarray _array, int _n, int _m);
	std::vector<Eigen::VectorXd> toEigenVectorVector(const np::ndarray& array);
	
	bool checkWithinBox(Eigen::Vector3d _point, Eigen::Vector3d _size, Eigen::Isometry3d _transform);
	std::vector<Eigen::Vector3d> getBoxVertices(Eigen::Isometry3d _center, Eigen::Vector3d _size);

	// two-joint ik
	void IKLimb(dart::dynamics::SkeletonPtr _skel, Eigen::VectorXd& _position, Eigen::Vector3d _target, bool _isLeft);

	// IK
	Eigen::VectorXd solveIK(dart::dynamics::SkeletonPtr skel, const std::string& bodyname, const Eigen::Vector3d& delta,  const Eigen::Vector3d& offset);
	Eigen::VectorXd solveMCIK(dart::dynamics::SkeletonPtr skel, std::vector<std::pair<std::string, Eigen::Vector3d>>& constraints);
	// Eigen::VectorXd solveMCIKRoot(dart::dynamics::SkeletonPtr skel, const std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>>& constraints);

	// IK for cma-es
	void optimizePosition(Eigen::VectorXd& _pos,
							 std::vector<std::string> _optimBodies,
							 std::vector<std::pair<std::string, Eigen::Vector3d>> _constraints, 
						   	 dart::dynamics::SkeletonPtr _skel);
	
}
#endif