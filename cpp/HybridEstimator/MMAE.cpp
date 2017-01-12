#include "MMAE.hpp"



MMAE::MMAE(std::vector<BaseKalmanFilter*> modelList)
{
	this->modelList = modelList;
}


MMAE::~MMAE()
{
}

VectorXd MMAE::getStatePrediction()
{

	return VectorXd();
}

VectorXd MMAE::getStateCovariancePrediction()
{

	return VectorXd();
}

VectorXd MMAE::getStateEstimate()
{

	return VectorXd();
}

VectorXd MMAE::getStateCovariance()
{

	return VectorXd();
}


void MMAE::runEstimator()
{

	return;
}


void MMAE::updateDeltaT(double deltaT)
{

	return;
}


std::vector<double> MMAE::getAllModelProbabilities()
{

	std::vector<double> ret;
	return ret;
}