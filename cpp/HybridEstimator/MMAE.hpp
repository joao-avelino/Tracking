#ifndef MMAE_HPP
#define MMAE_HPP

#include "HybridEstimator.hpp"
#include "MMAEItem.hpp"

class MMAE :
	HybridEstimator
{
public:
	MMAE(std::vector<std::shared_ptr<MMAEItem> > filterBank, double minimumProbability=0.05, bool initializeProbs=true);
	~MMAE();
	VectorXd getStatePrediction();
	MatrixXd getStateCovariancePrediction();
	VectorXd getStatePosterior();
	MatrixXd getStateCovariancePosterior();
    void predict(VectorXd &control);
	void predict()
	{
		VectorXd empty = VectorXd();
		predict(empty);
	};
    void update(VectorXd &measure);
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();

private:
	void computeProbabilities(VectorXd & measure);
	std::vector< std::shared_ptr<MMAEItem> > filterBank;
	double stateDim;
	double minimumProbability;

};

#endif // MMAE_HPP


