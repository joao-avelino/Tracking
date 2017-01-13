#ifndef MMAE_HPP
#define MMAE_HPP

#include "HybridEstimator.hpp"
class MMAE :
	HybridEstimator
{
public:
	MMAE(std::vector<std::shared_ptr<BaseKalmanFilter> > modelList);
	~MMAE();
	VectorXd getStatePrediction();
	VectorXd getStateCovariancePrediction();
	VectorXd getStateEstimate();
	VectorXd getStateCovariance();
	void runEstimator();
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();
};

#endif // MMAE_HPP


