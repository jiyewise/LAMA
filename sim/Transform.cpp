#include "Transform.h"
#include "Configurations.h"
#include "Functions.h"
#include <boost/filesystem.hpp>
#include <Eigen/QR>
#include <fstream>
#include <numeric>
#include <algorithm>
namespace SIM
{	
Pose2d 
Transform::
to2d(Eigen::Isometry3d _world)
{
	Eigen::Matrix3d m;

	m = _world.linear();
	
	Eigen::Matrix3d adjust = Eigen::Quaterniond::FromTwoVectors(m*Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()).toRotationMatrix();
	adjust = adjust*m;

	double angle = logMap(adjust)(1);
	Eigen::Vector2d dir = Eigen::Vector2d(sin(angle), cos(angle)); //
	Eigen::Vector2d pos = Eigen::Vector2d(_world.translation()(0), _world.translation()(2)); //

	return Pose2d(pos, dir);
}
Eigen::Isometry3d 
Transform::
to3d(Pose2d _local)
{
	double angle = atan2(_local.dir(0), _local.dir(1)); //
	Eigen::Vector3d dir = Eigen::Vector3d(0, angle, 0);
	Eigen::Vector3d pos = Eigen::Vector3d(_local.pos(0), 0, _local.pos(1)); //

	Eigen::Matrix3d m;
	Eigen::Matrix3d local = expMapRot(dir);
	
	m = local;

	Eigen::Isometry3d _world;
	_world.linear() = m;
	_world.translation() = pos;

	return _world;
}
Eigen::Vector2d 
Transform::
to2dPos(Eigen::Vector3d _pos)
{
	return Eigen::Vector2d(_pos(0), _pos(2)); //
}
Eigen::Vector3d 
Transform::
to3dPos(Eigen::Vector2d _pos)
{
	return Eigen::Vector3d(_pos(0), 0, _pos(1)); //
}

Pose2d
Transform::
rotateDir(Pose2d _base, double _yRot)
{
	Eigen::Vector2d pos = _base.pos;
	Eigen::Matrix2d yRot;
	yRot << cos(_yRot), -sin(_yRot), sin(_yRot), cos(_yRot);
	Eigen::Vector2d newDir = yRot * _base.dir;

	Pose2d newPos(pos, newDir);
	return newPos;
}

Pose2d 
Transform::
getLocalTransform(Pose2d _base, Pose2d _target)
{
	Eigen::Vector2d xAxis = _base.dir;
	Eigen::Vector2d yAxis = Eigen::Vector2d(-_base.dir(1), _base.dir(0));

	Eigen::Vector2d dt = _target.pos - _base.pos;
	Eigen::Vector2d pos = Eigen::Vector2d(dt.dot(xAxis), dt.dot(yAxis));
	Eigen::Vector2d dir = Eigen::Vector2d(_target.dir.dot(xAxis), _target.dir.dot(yAxis));

	return Pose2d(pos, dir);
}
Eigen::Isometry3d 
Transform::
getGlobalTransform(Pose2d _from, Pose2d _to)
{
	Eigen::Isometry3d from = to3d(_from);
	Eigen::Isometry3d to = to3d(_to);

	return to * from.inverse();
}
Pose2d 
Transform::
applyTransform(Pose2d _from, Pose2d _transform)
{
	Eigen::Vector2d xAxis = _from.dir;
	Eigen::Vector2d yAxis = Eigen::Vector2d(-_from.dir(1), _from.dir(0));

	double angle = atan2(_from.dir(1), _from.dir(0)) + atan2(_transform.dir(1), _transform.dir(0));

	Eigen::Vector2d pos = _transform.pos(0) * xAxis + _transform.pos(1) * yAxis + _from.pos;
	Eigen::Vector2d dir = Eigen::Vector2d(cos(angle), sin(angle));

	return Pose2d(pos, dir);
}

double 
Transform:: 
angleBtwTwoDir(Eigen::Vector2d _from, Eigen::Vector2d _to)
{
	double angle = atan2(_to(1), _to(0)) - atan2(_from(1), _from(0));
	if(angle < 0)
		angle += 2*M_PI;
	return angle;
}
}