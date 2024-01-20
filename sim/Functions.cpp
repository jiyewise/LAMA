#include <string>
#include <iostream>
#include "Functions.h"
// #include "cmaes.h"
namespace SIM
{
Eigen::Matrix3d
eulerX2Rot(double _x)
{
	double cosa = cos(_x*M_PI/180.0);
	double sina = sin(_x*M_PI/180.0);
	Eigen::Matrix3d R;
	R<<	1,0		,0	  ,
		0,cosa	,-sina,
		0,sina	,cosa ;
	return R;
}
Eigen::Matrix3d 
eulerY2Rot(double _y)
{
	double cosa = cos(_y*M_PI/180.0);
	double sina = sin(_y*M_PI/180.0);
	Eigen::Matrix3d R;
	R <<cosa ,0,sina,
		0    ,1,   0,
		-sina,0,cosa;
	return R;	
}
Eigen::Matrix3d 
eulerZ2Rot(double _z)
{
	double cosa = cos(_z*M_PI/180.0);
	double sina = sin(_z*M_PI/180.0);
	Eigen::Matrix3d R;
	R<<	cosa,-sina,0,
		sina,cosa ,0,
		0   ,0    ,1;
	return R;		
}
//from Dart library
Eigen::Vector3d 
logMap(Eigen::Matrix3d _mat) 
{
    Eigen::AngleAxisd aa(_mat);
    return aa.angle()*aa.axis();
}
//from Dart library
Eigen::Matrix3d 
makeSkewSymmetric(Eigen::Vector3d _v) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

  result(0, 1) = -_v(2);
  result(1, 0) =  _v(2);
  result(0, 2) =  _v(1);
  result(2, 0) = -_v(1);
  result(1, 2) = -_v(0);
  result(2, 1) =  _v(0);

  return result;
}
//from Dart library
Eigen::Matrix3d 
expMapRot(Eigen::Vector3d _q) {
  double theta = _q.norm();

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d qss = makeSkewSymmetric(_q);
  Eigen::Matrix3d qss2 = qss*qss;

  if (theta < 1e-3)
    R = Eigen::Matrix3d::Identity() + qss + 0.5*qss2;
  else
    R = Eigen::Matrix3d::Identity()
        + (sin(theta)/theta)*qss
        + ((1-cos(theta))/(theta*theta))*qss2;

  return R;
}
std::string
vec2Str(Eigen::VectorXd _vec)
{
	std::string str = "";
	for(int i = 0; i < _vec.rows(); i++) {
		str += std::to_string(_vec[i]);
		if(i != _vec.rows() - 1) 
			str += " ";
	}
	return str;
}
Eigen::Vector3d 
proj(Eigen::Vector3d _u, Eigen::Vector3d _v)
{
	Eigen::Vector3d proj;
	proj = _u.dot(_v)/_u.dot(_u)*_u;
	return proj;	
}
Eigen::Matrix3d
orthonormalize(Eigen::Matrix3d _mat)
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d v0,v1,v2;
	Eigen::Vector3d u0,u1,u2;
	v0 = _mat.col(0);
	v1 = _mat.col(1);
	v2 = _mat.col(2);

	u0 = v0;
	u1 = v1 - proj(u0,v1);
	u2 = v2 - proj(u0,v2) - proj(u1,v2);

	u0.normalize();
	u1.normalize();
	u2.normalize();

	rot.col(0) = u0;
	rot.col(1) = u1;
	rot.col(2) = u2;
	return rot;
}

Eigen::VectorXd 
str2VecXd(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	double* ptr = &v[0];
	Eigen::Map<Eigen::VectorXd> result(ptr, v.size());
	return result;
}
std::vector<double> 
str2Double(std::string _str)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    while(sz < _str.length()){
        result.push_back(std::stold(_str.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}
Eigen::Matrix3d
str2Mat3d(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	Eigen::Matrix3d mat;
	mat << v[0], v[1], v[2],
			v[3], v[4], v[5],
			v[6], v[7], v[8];
	return mat;
}
Eigen::Vector3d
str2Vec3d(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	Eigen::Vector3d vec;
	vec << v[0], v[1], v[2];
	return vec;
}

Eigen::Vector2d
str2Vec2d(std::string _str)
{
	std::vector<double> v = str2Double(_str);
	Eigen::Vector2d vec;
	vec << v[0], v[1];
	return vec;
}

std::vector<std::string>
split(std::string _str, std::string _delim)
{
    size_t start;
    size_t end = 0;
	
	std::vector<std::string> out;
    while ((start = _str.find_first_not_of(_delim, end)) != std::string::npos)
    {
        end = _str.find(_delim, start);
        out.push_back(_str.substr(start, end - start));
    }
	return out;
}

Eigen::Vector3d 
quat2Pos(Eigen::Quaterniond _q)
{
	Eigen::AngleAxisd aa(_q);
	double angle = aa.angle();
	angle = std::fmod(angle + M_PI, 2 * M_PI)-M_PI;
	return angle*aa.axis();
}
Eigen::Quaterniond
pos2Quat(Eigen::Vector3d _in) 
{
	if(_in.norm() < 1e-8 ){
		return Eigen::Quaterniond::Identity();
	}
	Eigen::AngleAxisd aa(_in.norm(), _in.normalized());
	Eigen::Quaterniond q(aa);
	quatNormalize(q);
	return q;
}

void 
quatNormalize(Eigen::Quaterniond& in){
	if(in.w() < 0){
		in.coeffs() *= -1;
	}
}

Eigen::VectorXd 
weightedSumPos(Eigen::VectorXd _source, Eigen::VectorXd _target, double _weight, bool _blendRoot)
{
	Eigen::VectorXd pos(_target.rows());
	pos = _target;

	for(int i = 0; i < pos.size(); i += 3) {
		if (i == 3) {
			if(_blendRoot)	pos.segment<3>(i) = (1 - _weight) * _source.segment<3>(i) + _weight * _target.segment<3>(i); 
			else pos[4] = (1 - _weight) * _source[4] + _weight * _target[4]; 
		} 
		else if (i == 0 && !_blendRoot) {
			pos(0) = _source(0) * (1-_weight) + _target(0) * _weight;
			pos(1) = _target(1);
			pos(2) = _source(2) * (1-_weight) + _target(2) * _weight;
		} 
		else {
			Eigen::AngleAxisd tAA(_target.segment<3>(i).norm(), _target.segment<3>(i).normalized());
			Eigen::AngleAxisd sAA(_source.segment<3>(i).norm(), _source.segment<3>(i).normalized());
					
			Eigen::Quaterniond tQ(tAA);
			Eigen::Quaterniond sQ(sAA);

			pos.segment<3>(i) = quat2Pos(sQ.slerp(_weight, tQ));
			pos.segment<3>(i) = quat2Pos(pos2Quat(pos.segment<3>(i)));
		}
	}

	return pos;
}

Eigen::VectorXd 
weightedSumPosClip(Eigen::VectorXd _source, Eigen::VectorXd _target, double _weight, Eigen::VectorXd _prevPos, bool _blendRoot)
{
	Eigen::VectorXd pos(_target.rows());
	pos = _target;

	for(int i = 0; i < pos.size(); i += 3) {
		if (i == 3) {
			if(_blendRoot)	pos.segment<3>(i) = (1 - _weight) * _source.segment<3>(i) + _weight * _target.segment<3>(i); 
			else pos[4] = (1 - _weight) * _source[4] + _weight * _target[4]; 
		} 
		else if (i == 0 && !_blendRoot) {
			pos(0) = _source(0) * (1-_weight) + _target(0) * _weight;
			pos(1) = _target(1);
			pos(2) = _source(2) * (1-_weight) + _target(2) * _weight;
		} 
		else {
			Eigen::AngleAxisd tAA(_target.segment<3>(i).norm(), _target.segment<3>(i).normalized());
			Eigen::AngleAxisd sAA(_source.segment<3>(i).norm(), _source.segment<3>(i).normalized());
					
			Eigen::Quaterniond tQ(tAA);
			Eigen::Quaterniond sQ(sAA);

			Eigen::Quaterniond cQ = sQ.slerp(_weight, tQ);

			pos.segment<3>(i) = quat2Pos(cQ);
			pos.segment<3>(i) = quat2Pos(pos2Quat(pos.segment<3>(i)));

			bool flip = false;
			if(i == 0) {
				if(_prevPos.segment<3>(i)[1] * pos.segment<3>(i)[1] < 0)
					pos.segment<3>(i)[1] *= -1;
			}
		}
	}

	return pos;
}
Eigen::VectorXd 
weightedSumVec(Eigen::VectorXd _source, Eigen::VectorXd _target, double _weight)
{
	Eigen::VectorXd vec = (1 - _weight) * _source + _weight * _target; 
	return vec;
}
Eigen::Vector3d 
getPosDiff(Eigen::Vector3d _p1, Eigen::Vector3d _p0)
{
	Eigen::Matrix3d R0 = expMapRot(_p0);
    Eigen::Matrix3d R1 = expMapRot(_p1);

  	return logMap(R0.transpose() * R1);
}
Eigen::Vector3d 
getPosDiffXZplane(Eigen::Vector3d _p1, Eigen::Vector3d _p0)
{
	_p1 = _p1.cwiseProduct(Eigen::Vector3d::UnitY());
	_p0 = _p0.cwiseProduct(Eigen::Vector3d::UnitY());
	return getPosDiff(_p1, _p0);
}
double 
expOfSquared(Eigen::VectorXd _vec,double _sigma)
{
	return exp(-1.0*_vec.dot(_vec)/(_sigma*_sigma)/_vec.rows());
}

Eigen::Vector3d 
rotate(Eigen::Vector3d _q0, Eigen::Vector3d _q1) {
	Eigen::AngleAxisd aa0 = Eigen::AngleAxisd(_q0.norm(), _q0.normalized());
	Eigen::AngleAxisd aa1 = Eigen::AngleAxisd(_q1.norm(), _q1.normalized());
  	Eigen::AngleAxisd aa;
  	aa = aa0 * aa1;
  	return aa.axis() * aa.angle();
}

Eigen::Vector3d 
rotateVector(Eigen::Quaterniond _q, Eigen::Vector3d _v) {
	Eigen::Isometry3d iso;
	iso.linear() = _q.toRotationMatrix();
	iso.translation() = Eigen::Vector3d::Zero();

	return iso * _v;
}

Eigen::Vector3d 
jointPositionDifferences(Eigen::Vector3d _q1, Eigen::Vector3d _q0)
{
	Eigen::Matrix3d R0 = expMapRot(_q0);
    Eigen::Matrix3d R1 = expMapRot(_q1);

  	return logMap(R0.transpose() * R1);
}

np::ndarray 
toNumPyArray(std::vector<double> _vec)
{
	int n = _vec.size();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i=0;i<n;i++)
	{
		dest[i] = _vec[i];
	}

	return array;
}
np::ndarray 
toNumPyArray(Eigen::VectorXd _vec)
{
	int n = _vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0; i<n; i++)
	{
		dest[i] = _vec[i];
	}

	return array;
}
np::ndarray 
toNumPyArray(Eigen::Vector3d _vec)
{
	int n = 3;
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0; i<n; i++)
	{
		dest[i] = _vec[i];
	}

	return array;
}
np::ndarray 
toNumPyArray(Eigen::MatrixXd _mat)
{
	int n = _mat.rows();
	int m = _mat.cols();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat(i,j);
		}
	}

	return array;
}
np::ndarray 
toNumPyArray(std::vector<Eigen::VectorXd> _mat)
{
	int n = _mat.size();
	int m;
	if(n == 0)
		m = 0;
	else  
		m = _mat[0].rows();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat[i][j];
		}
	}

	return array;
}
np::ndarray 
toNumPyArray(std::vector<Eigen::Vector3d> _mat)
{
	int n = _mat.size();
	int m;
	if(n == 0)
		m = 0;
	else  
		m = 3;

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat[i][j];
		}
	}

	return array;
}
np::ndarray 
toNumPyArray(std::vector<std::vector<double>> _mat)
{
	int n = _mat.size();
	int m = _mat[0].size();

	p::tuple shape = p::make_tuple(n, m);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape, dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			dest[index++] = _mat[i][j];
		}
	}

	return array;
}
Eigen::VectorXd 
toEigenVector(np::ndarray _array, int _n)
{
	Eigen::VectorXd vec(_n);

	float* srcs = reinterpret_cast<float*>(_array.get_data());

	for(int i=0; i<_n; i++)
	{
		vec[i] = srcs[i];
	}
	return vec;
}
Eigen::MatrixXd 
toEigenMatrix(np::ndarray _array, int _n, int _m)
{
	Eigen::MatrixXd mat(_n, _m);

	float* srcs = reinterpret_cast<float*>(_array.get_data());

	int index = 0;
	for(int i=0; i<_n; i++)
	{
		for(int j=0; j<_m; j++)
		{
			mat(i,j) = srcs[index++];
		}
	}
	return mat;
}

std::vector<Eigen::VectorXd> 
toEigenVectorVector(const np::ndarray& array)
{
	std::vector<Eigen::VectorXd> mat;
	mat.resize(array.shape(0));
	
	float* srcs = reinterpret_cast<float*>(array.get_data());
	int index = 0;
	
	for(int i=0;i<array.shape(0);i++){
		mat[i].resize(array.shape(1));
		for(int j=0;j<array.shape(1);j++)
			mat[i][j] = srcs[index++];
	}

	return mat;	
}

Eigen::VectorXd 
addDisplacement(Eigen::VectorXd _p, Eigen::VectorXd _d)
{
	Eigen::VectorXd result(_p.rows());
	for(int j = 0; j < result.rows(); j += 3) {
		if(j == 3) {
			result.segment<3>(j) = _d.segment<3>(j) + _p.segment<3>(j);
		} else {
			result.segment<3>(j) = rotate(_p.segment<3>(j), _d.segment<3>(j));
		} 
	}
	return result;
}
Eigen::VectorXd 
subDisplacement(Eigen::VectorXd _p0, Eigen::VectorXd _p1)
{
	Eigen::VectorXd d(_p0.rows());

	for(int j = 0; j < _p0.rows(); j += 3) {			
		if(j == 3) {
			d.segment<3>(j) = _p1.segment<3>(j) - _p0.segment<3>(j);
		} else {
			Eigen::Matrix3d r0 = expMapRot(_p0.segment<3>(j));
			Eigen::Matrix3d r1 = expMapRot(_p1.segment<3>(j));
			d.segment<3>(j) = logMap(r1.transpose() * r0);
		}
	}

	return d;
}

void 
IKLimb(dart::dynamics::SkeletonPtr _skel, Eigen::VectorXd& _position, Eigen::Vector3d _target, bool _isLeft)
{

	std::string FootJoint = _isLeft ? std::string("LeftFoot") : std::string("RightFoot"); 
	std::string LegJoint = _isLeft ? std::string("LeftLeg") : std::string("RightLeg"); 
	std::string UpLegJoint = _isLeft ? std::string("LeftUpLeg") : std::string("RightUpLeg"); 

	_skel->setPositions(_position);
	Eigen::Isometry3d prevFoot = _skel->getBodyNode(FootJoint)->getWorldTransform() * _skel->getBodyNode(FootJoint)->getParentJoint()->getTransformFromChildBodyNode();

	dart::dynamics::BodyNode* bn_a = _skel->getBodyNode(UpLegJoint);
	dart::dynamics::BodyNode* bn_b = _skel->getBodyNode(LegJoint);
	dart::dynamics::BodyNode* bn_c = _skel->getBodyNode(FootJoint);

	Eigen::Isometry3d a_iso = bn_a->getWorldTransform() * bn_a->getParentJoint()->getTransformFromChildBodyNode();
	Eigen::Isometry3d b_iso = bn_b->getWorldTransform() * bn_b->getParentJoint()->getTransformFromChildBodyNode();
	Eigen::Isometry3d c_iso = bn_c->getWorldTransform() * bn_c->getParentJoint()->getTransformFromChildBodyNode();

	int a_idx, b_idx, c_idx;
	a_idx = bn_a->getParentJoint()->getIndexInSkeleton(0);
	b_idx = bn_b->getParentJoint()->getIndexInSkeleton(0);
	c_idx = bn_c->getParentJoint()->getIndexInSkeleton(0);

	Eigen::Vector3d a = a_iso.translation();
	Eigen::Vector3d b = b_iso.translation();
	Eigen::Vector3d c = c_iso.translation();

	double lab = (b-a).norm();
    double lcb = (b-c).norm();
    double lat = dart::math::clip((_target-a).norm(), 0.01, lab+lcb-0.01);

	Eigen::Vector3d c_a = (c-a).normalized();
	Eigen::Vector3d b_a = (b-a).normalized();
	Eigen::Vector3d a_b = (a-b).normalized();
	Eigen::Vector3d c_b = (c-b).normalized();
	Eigen::Vector3d t_a = (_target-a).normalized();

    double ac_ab_0 = acos(dart::math::clip(c_a.dot(b_a), -1.0, 1.0));
    double ba_bc_0 = acos(dart::math::clip(a_b.dot(c_b), -1.0, 1.0));
    double ac_at_0 = acos(dart::math::clip(c_a.dot(t_a), -1.0, 1.0));

    double ac_ab_1 = acos(dart::math::clip((lcb*lcb-lab*lab-lat*lat) / (-2*lab*lat), -1.0, 1.0));
    double ba_bc_1 = acos(dart::math::clip((lat*lat-lab*lab-lcb*lcb) / (-2*lab*lcb), -1.0, 1.0));

	Eigen::Vector3d axis0, axis1;
    axis0 = ((c_a).cross(b_a)).normalized();
    axis1 = ((c_a).cross(t_a)).normalized();

	// change into local coordinates
	Eigen::AngleAxisd r0, r1, r2;
	r0.axis() = a_iso.linear().inverse() * axis0;
	r1.axis() = b_iso.linear().inverse() * axis0;
	r2.axis() = a_iso.linear().inverse() * axis1;

	double r0_angle, r1_angle, r2_angle;
	r0_angle = ac_ab_1 - ac_ab_0;
	r1_angle = ba_bc_1 - ba_bc_0;
	r2_angle = ac_at_0;

	Eigen::Vector3d a_lr = logMap(expMapRot(_position.segment<3>(a_idx))*expMapRot(r0.axis() * r0_angle)*expMapRot(r2.axis() * r2_angle));
	Eigen::Vector3d b_lr = logMap(expMapRot(_position.segment<3>(b_idx))*expMapRot(r1.axis() * r1_angle));

	_position.segment<3>(a_idx) = a_lr;
	_position.segment<3>(b_idx) = b_lr;

	// _skel->setPositions(_position);
	// Eigen::Isometry3d curFoot = _skel->getBodyNode(FootJoint)->getWorldTransform() * _skel->getBodyNode(FootJoint)->getParentJoint()->getTransformFromChildBodyNode();
	// std::cout << "frame : " << _idx << " target : " << _target.transpose() << " cur " << curFoot.translation().transpose() << " prev: " << prevFoot.translation().transpose() << std::endl;

}

bool 
checkWithinBox(Eigen::Vector3d _point, Eigen::Vector3d _size, Eigen::Isometry3d _transform)
{
	Eigen::Vector3d localPos = _transform.inverse() * _point;
	if(localPos[0] > (_size(0)*0.5+0.01) || localPos[0] < (-1*_size(0)*0.5-0.01))
		return false;
	if(localPos[1] > (_size(1)*0.5+0.01) || localPos[1] < (-1*_size(1)*0.5-0.01))
		return false;
	if(localPos[2] > (_size(2)*0.5+0.01) || localPos[2] < (-1*_size(2)*0.5-0.01))
		return false;
	return true;
}

std::vector<Eigen::Vector3d>
getBoxVertices(Eigen::Isometry3d _center, Eigen::Vector3d _size)
{
	std::vector<Eigen::Vector3d> points;
	for(int x = 0; x < 2; x++) {
		for(int y = 0; y < 2; y++) {
			for(int z = 0; z < 2; z++) {
				double _x = (2*x-1) * _size(0) * 0.5;
				double _y = (2*y-1) * _size(1) * 0.5;
				double _z = (2*z-1) * _size(2) * 0.5;
				Eigen::Vector3d point(_x, _y, _z);
				points.push_back(point);
			}
		}
	}
	// convert to global
	for(int i = 0; i < 8; i++) {
		points[i] = _center * points[i];
	}
	return points;
}


Eigen::VectorXd
convertNodeOrder(Eigen::VectorXd _pos, dart::dynamics::SkeletonPtr _targetSkel, std::vector<std::pair<std::string, int>> _idxList)
{
	Eigen::VectorXd pos(_pos.rows());
	for(int i = 0; i < _idxList.size(); i++) {
		dart::dynamics::BodyNode* bn = _targetSkel->getBodyNode(_idxList[i].first);
		int idx = bn->getParentJoint()->getIndexInSkeleton(0);
		if(idx == 0) {
			pos.segment<3>(3) = _pos.segment<3>(0);
			pos.segment<3>(0) = _pos.segment<3>(3);
		} else {
			pos.segment<3>(idx) = _pos.segment<3>(_idxList[i].second);
			if(pos.segment<3>(idx).isZero()) {
				Eigen::Vector3d p(1, 0, 0);
				pos.segment<3>(idx) = p;
			}
		}
	}
	return pos;
}

Eigen::VectorXd
convertNodeOrder(Eigen::VectorXd _pos, dart::dynamics::SkeletonPtr _targetSkel, std::string _sourceSkel)
{
	std::tuple<BVHNode*, double, std::vector<Eigen::VectorXd>> bvhInfo = BVHParser::parse(_sourceSkel);
	std::vector<std::pair<std::string, int>> idxList = BVHParser::getNodeIdx(std::get<0>(bvhInfo)); 

	Eigen::VectorXd pos(_pos.rows());
	for(int i = 0; i < idxList.size(); i++) {
		dart::dynamics::BodyNode* bn = _targetSkel->getBodyNode(idxList[i].first);
		int idx = bn->getParentJoint()->getIndexInSkeleton(0);
		if(idx == 0) {
			pos.segment<3>(3) = _pos.segment<3>(0);
			pos.segment<3>(0) = _pos.segment<3>(3);
		} else {
			pos.segment<3>(idx) = _pos.segment<3>(idxList[i].second);
			if(pos.segment<3>(idx).isZero()) {
				Eigen::Vector3d p(1, 0, 0);
				pos.segment<3>(idx) = p;
			}
		}
	}
	return pos;
}


Eigen::VectorXd solveIK(dart::dynamics::SkeletonPtr skel, const std::string& bodyname, const Eigen::Vector3d& delta, const Eigen::Vector3d& offset)
{
	auto bn = skel->getBodyNode(bodyname);
	int foot_l_idx = skel->getBodyNode("LeftToe")->getParentJoint()->getIndexInSkeleton(0);
	int foot_r_idx = skel->getBodyNode("RightToe")->getParentJoint()->getIndexInSkeleton(0);
	int footend_l_idx = skel->getBodyNode("LeftToeEnd")->getParentJoint()->getIndexInSkeleton(0);
	int footend_r_idx = skel->getBodyNode("RightToeEnd")->getParentJoint()->getIndexInSkeleton(0);
	int femur_l_idx = skel->getBodyNode("LeftUpLeg")->getParentJoint()->getIndexInSkeleton(0);
	int femur_r_idx = skel->getBodyNode("RightUpLeg")->getParentJoint()->getIndexInSkeleton(0);
	int tibia_l_idx = skel->getBodyNode("LeftLeg")->getParentJoint()->getIndexInSkeleton(0);
	int tibia_r_idx = skel->getBodyNode("RightLeg")->getParentJoint()->getIndexInSkeleton(0);
	Eigen::VectorXd newPose = skel->getPositions();
	Eigen::Vector3d tp = delta;
	for(std::size_t i = 0; i < 1000; ++i)
	{
		Eigen::Vector3d deviation = tp - bn->getTransform()*offset;
		if(deviation.norm() < 0.001)
			break;
		// Eigen::Vector3d localCOM = bn->getCOM(bn);
		dart::math::LinearJacobian jacobian = skel->getLinearJacobian(bn, offset);
		jacobian.block<3,6>(0,0).setZero();
		// jacobian.block<3,3>(0,foot_l_idx).setZero();
		// jacobian.block<3,3>(0,foot_r_idx).setZero();
		jacobian.block<3,3>(0,footend_l_idx).setZero();
		jacobian.block<3,3>(0,footend_r_idx).setZero();
		// jacobian.block<3,2>(0,femur_l_idx+1).setZero();
		// jacobian.block<3,2>(0,femur_r_idx+1).setZero();
		// jacobian.block<3,2>(0,tibia_l_idx+1).setZero();
		// jacobian.block<3,2>(0,tibia_r_idx+1).setZero();

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d inv_singular_value;
		
		inv_singular_value.setZero();
		for(int k=0;k<3;k++)
		{
			if(svd.singularValues()[k]==0)
				inv_singular_value(k,k) = 0.0;
			else
				inv_singular_value(k,k) = 1.0/svd.singularValues()[k];
		}


		Eigen::MatrixXd jacobian_inv = svd.matrixV()*inv_singular_value*svd.matrixU().transpose();

		// Eigen::VectorXd gradient = jacobian.colPivHouseholderQr().solve(deviation);
		Eigen::VectorXd gradient = jacobian_inv * deviation;
		double prev_norm = deviation.norm();
		double gamma = 0.5;
		for(int j = 0; j < 24; j++){
			Eigen::VectorXd newDirection = gamma * gradient;
			Eigen::VectorXd np = newPose + newDirection;
			skel->setPositions(np);
			skel->computeForwardKinematics(true, false, false);
			double new_norm = (tp - bn->getTransform()*offset).norm();
			if(new_norm < prev_norm){
				newPose = np;
				break;
			}
			gamma *= 0.5;
		}
	}
	return newPose;
}
Eigen::VectorXd solveMCIKRoot(dart::dynamics::SkeletonPtr skel, const std::vector<std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d>>& constraints)
{
	Eigen::VectorXd newPose = skel->getPositions();
	int num_constraints = constraints.size();

	std::vector<dart::dynamics::BodyNode*> bodynodes(num_constraints);
	std::vector<Eigen::Vector3d> targetposes(num_constraints);
	std::vector<Eigen::Vector3d> offsets(num_constraints);

	for(int i = 0; i < num_constraints; i++){
		bodynodes[i] = skel->getBodyNode(std::get<0>(constraints[i]));
		targetposes[i] = std::get<1>(constraints[i]);
		offsets[i] = std::get<2>(constraints[i]);
	}

	int not_improved = 0;
	for(std::size_t i = 0; i < 100; i++)
	{

		// make deviation vector and jacobian matrix
		Eigen::VectorXd deviation(num_constraints*3);
		for(int j = 0; j < num_constraints; j++){
			deviation.segment<3>(j*3) = targetposes[j] - bodynodes[j]->getTransform()*offsets[j];
		}
		if(deviation.norm() < 0.001)
			break;

		int nDofs = skel->getNumDofs();
		Eigen::MatrixXd jacobian_concatenated(3*num_constraints, nDofs);
		for(int j = 0; j < num_constraints; j++){
			dart::math::LinearJacobian jacobian = skel->getLinearJacobian(bodynodes[j], offsets[j]);
			jacobian.block(0, 0, 3, 3).setZero();
			jacobian.block(0, 3, 3, 1).setZero();
			jacobian.block(0, 5, 3, 1).setZero();
			jacobian.block(0, 6, 3, nDofs - 6).setZero();
			std::cout << jacobian << std::endl;
			jacobian_concatenated.block(3*j, 0, 3, nDofs) = jacobian;
		}
		// std::cout << jacobian_concatenated << std::endl;

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_concatenated, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd inv_singular_value(3*num_constraints, 3*num_constraints);
		
		inv_singular_value.setZero();
		for(int k=0;k<3*num_constraints;k++)
		{
			if(svd.singularValues()[k]<1e-8)
				inv_singular_value(k,k) = 0.0;
			else
				inv_singular_value(k,k) = 1.0/svd.singularValues()[k];
		}


		Eigen::MatrixXd jacobian_inv = svd.matrixV()*inv_singular_value*svd.matrixU().transpose();
		// std::cout << svd.singularValues().transpose() << std::endl;
		// std::cout << svd.matrixV().size() << std::endl;

		// std::cout << jacobian_inv << std::endl;
		// exit(0);
		// Eigen::VectorXd gradient = jacobian.colPivHouseholderQr().solve(deviation);
		Eigen::VectorXd gradient = jacobian_inv * deviation;
		double prev_norm = deviation.norm();
		double gamma = 0.5;
		not_improved++;
		for(int j = 0; j < 24; j++){
			Eigen::VectorXd newDirection = gamma * gradient;
			Eigen::VectorXd np = newPose + newDirection;
			skel->setPositions(np);
			skel->computeForwardKinematics(true, false, false);

			Eigen::VectorXd new_deviation(num_constraints*3);
			for(int j = 0; j < num_constraints; j++){
				new_deviation.segment<3>(j*3) = targetposes[j] - bodynodes[j]->getTransform()*offsets[j];
			}
			double new_norm = new_deviation.norm();
			if(new_norm < prev_norm){
				newPose = np;
				not_improved = 0;
				break;
			}
			gamma *= 0.5;
		}
		if(not_improved > 1){
			break;
		}
	}
	return newPose;
}

Eigen::VectorXd 
solveMCIK(dart::dynamics::SkeletonPtr skel, std::vector<std::pair<std::string, Eigen::Vector3d>>& constraints)
{
	Eigen::VectorXd newPose = skel->getPositions();
	int num_constraints = constraints.size();
	int llIdx = skel->getBodyNode("LeftLeg")->getParentJoint()->getIndexInSkeleton(0);
	int rlIdx = skel->getBodyNode("RightLeg")->getParentJoint()->getIndexInSkeleton(0);

	std::vector<dart::dynamics::BodyNode*> bodynodes(num_constraints);
	std::vector<Eigen::Vector3d> targetposes(num_constraints);

	for(int i = 0; i < num_constraints; i++){
		bodynodes[i] = skel->getBodyNode(constraints[i].first);
		targetposes[i] = constraints[i].second;
	}

	Eigen::VectorXd deviation(num_constraints*3);
	Eigen::Isometry3d curRootInv = skel->getRootBodyNode()->getWorldTransform().inverse();

	for(int j = 0; j < num_constraints; j++){
		deviation.segment<3>(j*3) = targetposes[j] - bodynodes[j]->getWorldTransform().translation();
	}


	for(std::size_t i = 0; i < 100; i++)
	{
		curRootInv = skel->getRootBodyNode()->getWorldTransform().inverse();

		// make deviation vector and jacobian matrix
		Eigen::VectorXd deviation(num_constraints*3);
		for(int j = 0; j < num_constraints; j++){
			deviation.segment<3>(j*3) = targetposes[j] - bodynodes[j]->getWorldTransform().translation();
		}

		if(deviation.norm() < 0.001)
			break;

		int nDofs = skel->getNumDofs();
		Eigen::MatrixXd jacobian_concatenated(3*num_constraints, nDofs);

		for(int j = 0; j < num_constraints; j++){
			dart::math::LinearJacobian jacobian = skel->getLinearJacobian(bodynodes[j]);
			jacobian.block(0, 3, 3, 3).setZero();
			jacobian.block(0, 0, 3, 1).setZero();
			jacobian.block(0, 2, 3, 1).setZero();

			// jacobian.block(0, 3, 3, 1).setZero();
			// jacobian.block(0, 5, 3, 1).setZero();
			// jacobian.block(0, 6, 3, nDofs-6).setZero();


			jacobian_concatenated.block(3*j, 0, 3, nDofs) = jacobian;
			jacobian_concatenated.block(3*j, llIdx, 3, 3).setZero();
			jacobian_concatenated.block(3*j, rlIdx, 3, 3).setZero();
		}
		// std::cout << jacobian_concatenated << std::endl;

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_concatenated, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd inv_singular_value(3*num_constraints, 3*num_constraints);
		
		inv_singular_value.setZero();
		for(int k=0;k<3*num_constraints;k++)
		{
			if(svd.singularValues()[k]<1e-8)
				inv_singular_value(k,k) = 0.0;
			else
				inv_singular_value(k,k) = 1.0/svd.singularValues()[k];
		}

		Eigen::MatrixXd jacobian_inv = svd.matrixV()*inv_singular_value*svd.matrixU().transpose();
		Eigen::VectorXd gradient = jacobian_inv * deviation;
		double prev_norm = deviation.norm();

		double gamma = 0.05;
		Eigen::VectorXd bestPos = newPose;
		double min = prev_norm;
		bool improved = false;
		for(int j = 0; j < 100; j++){
			Eigen::VectorXd newDirection = gamma  * j * gradient;
			Eigen::VectorXd np = newPose + newDirection;
			skel->setPositions(np);

			Eigen::VectorXd new_deviation(num_constraints*3);
			curRootInv = skel->getRootBodyNode()->getWorldTransform().inverse();

			for(int k = 0; k < num_constraints; k++){
				new_deviation.segment<3>(k*3) = targetposes[k] - bodynodes[k]->getWorldTransform().translation();
			}
			double new_norm = new_deviation.norm();

			if(new_norm < min){
				newPose = np;
				min = new_norm;
				improved = true;
			}
		}
		skel->setPositions(newPose);

		if(!improved){
			break;
		}
	}

	return newPose;
}


};