#ifndef __SIM_TRANSFORM_H__
#define __SIM_TRANSFORM_H__
#include "dart/dart.hpp"
#include <queue>

namespace SIM
{
struct Pose2d
{
	Pose2d(): dir(Eigen::Vector2d(0, 1)), pos(Eigen::Vector2d(0,0)) {}
	Pose2d(Eigen::Vector2d _p): dir(Eigen::Vector2d(0, 1)), pos(_p) {}	
	Pose2d(Eigen::Vector2d _p, Eigen::Vector2d _d) : dir(_d.normalized()), pos(_p) {}
	Eigen::Vector2d dir;
	Eigen::Vector2d pos;
};
class Transform
{
public:
	static Pose2d to2d(Eigen::Isometry3d _world);
	static Eigen::Isometry3d to3d(Pose2d _local);

	static Eigen::Vector2d to2dPos(Eigen::Vector3d _pos);
	static Eigen::Vector3d to3dPos(Eigen::Vector2d _pos);

	static Pose2d getLocalTransform(Pose2d _base, Pose2d _target);
	static Eigen::Isometry3d getGlobalTransform(Pose2d _from, Pose2d _to);

	static Pose2d applyTransform(Pose2d _from, Pose2d _transform);

	static Pose2d rotateDir(Pose2d _base, double _yRot);
	static double angleBtwTwoDir(Eigen::Vector2d _from, Eigen::Vector2d _to);
};
}
#endif
