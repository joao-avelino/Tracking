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
	virtual void predict();
	virtual void update();
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();

private:
	void computeProbabilities();

};

#endif // MMAE_HPP


