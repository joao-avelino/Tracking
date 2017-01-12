#pragma once
#include "HybridEstimator.hpp"
class MMAE :
	HybridEstimator
{
public:
	MMAE(std::vector<BaseKalmanFilter*> modelList);
	~MMAE();
	VectorXd getStatePrediction();
	VectorXd getStateCovariancePrediction();
	VectorXd getStateEstimate();
	VectorXd getStateCovariance();
	void runEstimator();
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();
};

