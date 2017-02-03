#ifndef MMAE_HPP
#define MMAE_HPP

#include "HybridEstimator.hpp"
#include "MMAEItem.hpp"


class MMAE :
	public HybridEstimator
{
public:
	MMAE(std::vector<std::shared_ptr<MMAEItem> > filterBank, double minimumProbability=0.05, bool initializeProbs=true);
	~MMAE();
	VectorXd getStatePred();
	MatrixXd getCovPred();
	VectorXd getStatePost();
	MatrixXd getCovPost();

	void setStatePred(VectorXd statePred);
	void setCovPred(MatrixXd covPred);
	void setStatePost(VectorXd statePost);
	void setCovPost(MatrixXd covPost);

    void predict(VectorXd &control);
	void predict()
	{
		VectorXd empty = VectorXd();
		predict(empty);
	};
    void update(VectorXd &measure);
	void update(VectorXd & measure, MatrixXd &measurementCov);
	void updateDeltaT(double deltaT);
	std::vector<double> getAllModelProbabilities();

	std::shared_ptr<BaseBayesianFilter> clone();
	std::shared_ptr<BaseBayesianFilter> clone(VectorXd initial_state);
	std::shared_ptr<BaseBayesianFilter> clone(VectorXd initial_state, MatrixXd measurementCov);

private:
	void computeProbabilities(VectorXd & measure);
	std::vector< std::shared_ptr<MMAEItem> > filterBank;
	double stateDim;
	double minimumProbability;

};

#endif // MMAE_HPP


