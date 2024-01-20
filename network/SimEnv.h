#ifndef __SIM_ENV_H__
#define __SIM_ENV_H__
#include "ActionController.h"
#include <vector>
#include <string>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;
class SimEnv
{
public:
	
	SimEnv(int _nslave, std::string _env);

	//For general properties
	int getNumState();
	int getNumAction();
	p::list getRewardLabels();

	//For each slave
	void step(int _id);
	void reset(int _id);
	void setAction(np::ndarray _array, int _id);
	p::tuple getStepInfo(int _id);
	np::ndarray getState(int _id);
	np::ndarray getRewardVector(int _id);

	//For all slaves
	void stepAll();
	void resetAll();
	void setActionAll(np::ndarray _array);
	np::ndarray getStateAll();
	np::ndarray getRewardVectorAll();
	void setTargetWeightAll(double _weight);
	void setPenetrationWeightAll(double _weight);
	
private:
	std::vector<ENV::ActionController*> mSlaves;

	int mNumSlave;
	int mNumAction;
	int mNumState;

};


#endif