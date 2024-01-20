#ifndef __ENV_ACTION_CONTROLLER_H__
#define __ENV_ACTION_CONTROLLER_H__

#include "ReferenceManager.h"
#include "MotionSynthesizer.h"
#include "InteractionCue.h"
#include "Environment.h"
#include "Scene.h"
#include "Functions.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
namespace p = boost::python;
namespace np = boost::python::numpy;

namespace ENV
{
class ActionController
{
public:
	ActionController(std::string _env, int _id=0, bool _render=false);
	ActionController(Environment* _env, int _id=0, bool _render=false);

	void step();
	void reset();

	// parametric test
	void reset(SIM::Pose2d _custom_initial);
	void reset(int _icIdx, int _initialIdx, double _dirAngle);

	void setAction(Eigen::VectorXd _action);

	Eigen::VectorXd getState() {return mStates; }
	std::vector<double> getRewardParts() {return mRewardParts; }
	std::vector<std::string> getRewardLabels() {return mRewardLabels; }
	
	int getNumState() {return mNumState; }
	int getNumAction() {return mNumAction; }
	
	double getTimeElapsed(){return mTimeElapsed;}
	double getCurrentFrame(){return mCurrentFrame;}
	double getStartFrame(){ return mStartFrame; }
	
	bool isTerminalState() {return mIsTerminal; }
	int getTerminalReason() {return mTerminalReason; }

    // env
    Environment* mEnv;
    Scene* mEnvScene;
	dart::dynamics::SkeletonPtr mCharacter;

    // motion synthesizer & action feature
    std::vector<InteractionCue> mInteractionCue; 
    MotionSynthesizer* mMotionSynthesizer;
    ActionFeature mCurActionFeature;
	std::vector<std::string> mActionTypeLabels;
	bool mIsActionComplete;
	void loadSynthesizer();

    // compute terminal condition and rewards
	ActionFeature generateActionFeatureFromAction();
	std::vector<Eigen::Vector3d> computePenetration(std::string _key, double _heightLimit=0.1);

	// interaction cue to fufill - rewards are based on this
	std::vector<InteractionCue> mTargetIC;
	int mCurrentStage;

	// renderer
	Eigen::VectorXd getPosition(int _idx) { return mRecordPosition[_idx]; }
	ActionFeature getActionFeature(int _idx) { return mRecordActionFeature[_idx]; }
	std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> getTargetTrajectory(int _idx) { return mRecordTargetTrajectory[_idx]; }
	std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> getCurrentTrajectory(int _idx) { return mRecordCurrentTrajectory[_idx]; }

	// autoencoder
	void loadAutoEncoder();
	std::vector<Eigen::VectorXd> editMotion(int _recordIdx, bool _postProcess=false);

	// edit bodies
	double getEditReward(Eigen::VectorXd _original, Eigen::VectorXd _edited, Eigen::VectorXd _prev);
	// double getContactReward(Eigen::VectorXd _original, Eigen::VectorXd _edited);
	
	std::vector<std::string> mEditBodies;
	Eigen::VectorXd mOriginalPos;
	bool mIsEdit;
	int mEditBodyDim;
	double mEditBodiesWeight;

	// cost record
	std::vector<std::pair<int, std::map<std::string, double>>> mRecordTransitionCost;
	std::vector<std::pair<int, std::map<std::string, double>>> getRecordTransitionCost() {return mRecordTransitionCost; }

	// weights
	void setTargetRewardWeight(double _weight) { mTargetRewardWeight = _weight; }
	void setPenetrationRewardWeight(double _weight) { mPenetrationRewardWeight = _weight; }
	double mTargetRewardWeight, mPenetrationRewardWeight;

	double checkPenetrationRewardAvg(int _interval);
	std::vector<double> mPenetrationRewardRecord;

	// parametric 
	void setInitialPosList();
	void setTargetICList(std::string _icListPath);
	void setRender(bool _render) { mRender = _render; }
	void setParametric(bool _isParam) { mIsParametric = _isParam; }
	std::vector<Eigen::VectorXd> getMotionRecord() { return mRecordPosition; }
	bool mIsParametric;
	std::vector<Eigen::Vector2d> mInitialPosList;
	std::vector<std::vector<InteractionCue>> mTargetICList;
	std::vector<std::pair<SIM::Pose2d, InteractionCue>> mInitialTargetRecord;

	// random
	std::random_device mRD;
	std::mt19937 mMT;
	std::uniform_real_distribution<double> mUniform;

protected:
	Eigen::VectorXd getEndEffectorStatePosAndVel(Eigen::VectorXd _pos, Eigen::VectorXd _vel);
	void updateReward(std::map<std::string, double> _transRecord);
	double getTargetReward();
	void updateState();
	void updateTerminalInfo();
	std::vector<std::pair<bool, Eigen::Vector3d>> getContactInfo(Eigen::VectorXd _pos);

	int mId;
	bool mRender;

	double mStartFrame;
	double mCurrentFrame;
	double mTimeElapsed;
	
	Eigen::VectorXd mActions;
	Eigen::VectorXd mStates;
	
	bool mIsTerminal;
	int mTerminalReason;

	std::vector<std::string> mEndEffectors;
	std::vector<std::string> mRewardLabels;
	std::vector<double> mRewardParts;

	std::vector<Eigen::VectorXd> mRecordPosition;
	// std::vector<std::map<std::string, double>> mRecordTransitionCost;
	// record trajectory and action feature
	std::vector<ActionFeature> mRecordActionFeature;
	std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> mRecordTargetTrajectory;
	std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> mRecordCurrentTrajectory;

	int mNumState;
	int mNumAction;
	int mTotalDof;

};
}
#endif