#include "SimEnv.h"
#include <omp.h>
#include "dart/math/math.hpp"
#include "Functions.h"
#include "Configurations.h"
#include "SkeletonBuilder.h"
SimEnv::
SimEnv(int _nslave, std::string _env) :mNumSlave(_nslave)
{
	dart::math::seedRand();
	omp_set_num_threads(mNumSlave);
	
	std::string envPath = std::string(LAMA_DIR) + std::string("/env/env_config/") + _env + std::string(".xml");
	ENV::Environment* env = new ENV::Environment(envPath);

	for(int i = 0 ;i < mNumSlave; i++)
	{
		mSlaves.push_back(new ENV::ActionController(env, i, false));
	}
	mNumAction = mSlaves[0]->getNumAction();
	mNumState = mSlaves[0]->getNumState();
}
//For general properties
int
SimEnv::
getNumState()
{
	return mNumState;
}
int
SimEnv::
getNumAction()
{
	return mNumAction;
}
p::list 
SimEnv::
getRewardLabels()
{
	p::list l;
	std::vector<std::string> sl = mSlaves[0]->getRewardLabels();
	for(int i =0 ; i < sl.size(); i++) l.append(sl[i]);
	return l;
}
//For each slave
void 
SimEnv::
step(int _id)
{
	mSlaves[_id]->step();
}
void 
SimEnv::
reset(int _id)
{
	mSlaves[_id]->reset();
}
p::tuple 
SimEnv::
getStepInfo(int _id) // TODO check
{
	// is_terminal, nan_occur, nt_elapsed, t_elapsed
	bool t = mSlaves[_id]->isTerminalState();
	// if(t) {
	// 	std::cout << "id: " << _id << " terminal" << std::endl;
	// }
	// bool n = mSlaves[_id]->isNan();
	// int start = mSlaves[_id]->getStartFrame();
	double cur = mSlaves[_id]->getCurrentFrame();
	double te = mSlaves[_id]->getTimeElapsed();

	// return p::make_tuple(t, n, cur - start, te);
	return p::make_tuple(t, cur, te); // is_terminal, (nan), nt_elapsed, t_elapsed(not used)
}
np::ndarray
SimEnv::
getState(int _id)
{
	return SIM::toNumPyArray(mSlaves[_id]->getState());
}
void 
SimEnv::
setAction(np::ndarray _array, int _id)
{
	mSlaves[_id]->setAction(SIM::toEigenVector(_array, mNumAction));
}
np::ndarray
SimEnv::
getRewardVector(int _id)
{
	std::vector<double> ret = mSlaves[_id]->getRewardParts();

	return SIM::toNumPyArray(ret);
}
void
SimEnv::
stepAll()
{
#pragma omp parallel for
	for (int id = 0; id < mNumSlave; id++)
	{
		step(id);
	}
}
void
SimEnv::
resetAll()
{
	for (int id = 0; id < mNumSlave; id++)
	{
		reset(id);
	}

}
np::ndarray
SimEnv::
getStateAll()
{
	Eigen::MatrixXd states(mNumSlave, mNumState);

	for (int id = 0; id < mNumSlave; id++)
	{
		states.row(id) = mSlaves[id]->getState().transpose();
	}
	return SIM::toNumPyArray(states);
}
void
SimEnv::
setActionAll(np::ndarray np_array)
{
	Eigen::MatrixXd action = SIM::toEigenMatrix(np_array, mNumSlave, mNumAction);

	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setAction(action.row(id).transpose());
	}

}
np::ndarray
SimEnv::
getRewardVectorAll()
{
	std::vector<std::vector<double>> rewards(mNumSlave);
	for (int id = 0; id < mNumSlave; ++id)
	{
		rewards.push_back(mSlaves[id]->getRewardParts());
	}
	return SIM::toNumPyArray(rewards);
}
void 
SimEnv::
setTargetWeightAll(double _weight) 
{
	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setTargetRewardWeight(_weight);
	}
}
void 
SimEnv::
setPenetrationWeightAll(double _weight) 
{
	for (int id = 0; id < mNumSlave; ++id)
	{
		mSlaves[id]->setPenetrationRewardWeight(_weight);
	}
}

using namespace boost::python;

BOOST_PYTHON_MODULE(simEnv)
{
	Py_Initialize();
	np::initialize();

	class_<SimEnv>("Env", init<int, std::string>())
		.def("getNumState",&SimEnv::getNumState)
		.def("getNumAction",&SimEnv::getNumAction)
		.def("getRewardLabels",&SimEnv::getRewardLabels)
		.def("step",&SimEnv::step)
		.def("reset",&SimEnv::reset)
		.def("getState",&SimEnv::getState)
		.def("setAction",&SimEnv::setAction)
		.def("getStepInfo",&SimEnv::getStepInfo)
		.def("getRewardVector",&SimEnv::getRewardVector)
		.def("stepAll",&SimEnv::stepAll)
		.def("resetAll",&SimEnv::resetAll)
		.def("getStateAll",&SimEnv::getStateAll)
		.def("setActionAll",&SimEnv::setActionAll)
		.def("getRewardVectorAll",&SimEnv::getRewardVectorAll)
		.def("setTargetWeightAll",&SimEnv::setTargetWeightAll)
		.def("setPenetrationWeightAll",&SimEnv::setPenetrationWeightAll)
		;

}