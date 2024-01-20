#ifndef __SIM_SKELETON_BUILDER_H__
#define __SIM_SKELETON_BUILDER_H__
#include "dart/dart.hpp"
namespace SIM
{
class XMLNode
{
public:
	XMLNode(std::string _name, Eigen::Vector3d _offset);
	void setParent(XMLNode* _parent);
	void setBodyTranslation(Eigen::Vector3d _bt);
	void setSize(Eigen::Vector3d _size);
	XMLNode* getParent();
	std::string getName();
	Eigen::Vector3d getBodyTranslation();
	Eigen::Vector3d getJointTranslation();
	Eigen::Vector3d getSize();
private:
	XMLNode* mParent;
	std::string mName;
	Eigen::Vector3d mJTfromParentJoint;
	Eigen::Vector3d mBTfromJoint;
	Eigen::Vector3d mSize;
};
class SkeletonBuilder
{
public:
	static void generateNewSkeleton(std::string _motion, std::string _path);
	static std::pair<dart::dynamics::SkeletonPtr, std::map<std::string, double>> buildFromFile(std::string _xml, bool _isScene=false, std::string _objPath="");
	
	static dart::dynamics::BodyNode* makeFreeJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		std::string _shape
		);

	static dart::dynamics::BodyNode* makeBallJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		bool _isLimitEnforced,
		Eigen::Vector3d _upperLimit,
		Eigen::Vector3d _lowerLimit,
		std::string _shape
		);

	static dart::dynamics::BodyNode* makeWeldJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d& _jointPosition,
		Eigen::Isometry3d& _bodyPosition,
		std::string _shape
		);

	static dart::dynamics::BodyNode* makeRevoluteJointBody(
		const dart::dynamics::SkeletonPtr& _skel,
		dart::dynamics::BodyNode* const _parent,
		std::string _name,
		Eigen::Vector3d _size,
		double _mass,
		Eigen::Isometry3d _jointPosition,
		Eigen::Isometry3d _bodyPosition,
		bool _isLimitEnforced,
		double _upperLimit,
		double _lowerLimit,
		std::string _shape,
		const Eigen::Vector3d& _axis
		);

	// static dart::dynamics::BodyNode* makePrismaticJointBody(
	// 	const std::string& body_name,
	// 	const dart::dynamics::SkeletonPtr& target_skel,
	// 	dart::dynamics::BodyNode* const parent,
	// 	const Eigen::Vector3d& size,
	// 	const Eigen::Isometry3d& joint_position,
	// 	const Eigen::Isometry3d& body_position,
	// 	bool isLimitEnforced,
	// 	double upper_limit,
	// 	double lower_limit,
	// 	double mass,
	// 	const Eigen::Vector3d& axis,
	// 	bool contact);

	// for rendering scenes (with obj mesh)
	// static dart::dynamics::BodyNode* makeSceneJointBody(
	// 	const dart::dynamics::SkeletonPtr& _skel,
	// 	dart::dynamics::BodyNode* const _parent,
	// 	std::string _name,
	// 	Eigen::Vector3d _size,
	// 	double _mass,
	// 	Eigen::Isometry3d& _jointPosition,
	// 	Eigen::Isometry3d& _bodyPosition,
	// 	std::string _objPath);	

};
}

#endif