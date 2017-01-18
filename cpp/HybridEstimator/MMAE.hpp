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
	MatrixXd getStateCovariancePrediction();
	VectorXd getStatePosterior();
	MatrixXd getStateCovariancePosterior();
	void predict(VectorXd &control=VectorXd());
	void update(VectorXd &measure=VectorXd());
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();

private:
	void computeProbabilities(VectorXd & measure);
	std::vector< std::shared_ptr<MMAEItem> > filterBank;
	double stateDim;

};

#endif // MMAE_HPP


