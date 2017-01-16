#ifndef MMAE_HPP
#define MMAE_HPP

#include "HybridEstimator.hpp"
#include "MMAEItem.hpp"

class MMAE :
	HybridEstimator
{
public:
	MMAE(std::vector<std::shared_ptr<MMAEItem> > filterBank);
	~MMAE();
	VectorXd getStatePrediction();
	VectorXd getStateCovariancePrediction();
	VectorXd getStateEstimate();
	VectorXd getStateCovariance();
	void predict(VectorXd &control=VectorXd());
	void update(VectorXd &measure=VectorXd());
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();

private:
	void computeProbabilities();
	std::vector< std::shared_ptr<MMAEItem> > filterBank;

};

#endif // MMAE_HPP


