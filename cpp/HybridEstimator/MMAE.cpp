#include "MMAE.hpp"

/**
* @file MMAE.cpp
* Copyright (C) 2016, Joao Avelino
*
* This class implements the Multiple Model Adaptive Estimator (MMAE) with SKFB
* proposed by [1].
*
*  Refs:
*  [1] Hanlon, P. D., & Maybeck, P. S. (2000). Multiple-model adaptive estimation 
*	   using a residual correlation Kalman filter bank. IEEE Transactions on Aerospace 
*	   and Electronic Systems, 36(2), 393-406.
*
*  License stuff
*/

/**
*  @brief KalmanFilter::MMAE
*
*  The contructor of the Multiple Model Adaptive Estimator
*
*  @param[in] kalmanBank The bank of Kalman Filters.
*
*/

MMAE::MMAE(std::vector<std::shared_ptr<BaseKalmanFilter> > kalmanBank)
{
	this->kalmanBank = kalmanBank;
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