#include "MMAE.hpp"
#include <iostream>

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

MMAE::MMAE(std::vector<std::shared_ptr<MMAEItem> > filterBank)
{
	this->filterBank = filterBank;

	stateDim = 0;
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		if (stateDim < ptr->getStateDim())
			stateDim = ptr->getStateDim();
	}


}


MMAE::~MMAE()
{
}

VectorXd MMAE::getStatePrediction()
{

	//Set up a vector for the states
	VectorXd stateMixture = VectorXd::Zero(stateDim);


	//Get the mixture of states
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		stateMixture += ptr->getStatePred()*ptr->getProbability();
	}

	return stateMixture;
}

MatrixXd MMAE::getStateCovariancePrediction()
{

	MatrixXd covMat = MatrixXd::Zero(stateDim, stateDim);

	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		MatrixXd filterCovPred = ptr->getCovPred();
		VectorXd filterStatePred = ptr->getStatePred();

		MatrixXd augmentedMat = MatrixXd::Zero(stateDim, stateDim);
		VectorXd augmentedState = VectorXd::Zero(stateDim);


		int rows = filterCovPred.rows();
		int cols = filterCovPred.cols();

		int modelStateDim = filterStatePred.size();

		augmentedMat.block(0, 0, rows, cols) = filterCovPred;
		augmentedState.head(modelStateDim) = filterStatePred;

		VectorXd diffState = augmentedState - getStatePrediction();

		covMat += ptr->getProbability()*(augmentedMat-diffState*diffState.transpose());
	}


	return covMat;
}

VectorXd MMAE::getStatePosterior()
{

	//Set up a vector for the states
	VectorXd stateMixture = VectorXd::Zero(stateDim);


	//Get the mixture of states
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		stateMixture += ptr->getStatePost()*ptr->getProbability();
	}

	return stateMixture;

}

MatrixXd MMAE::getStateCovariancePosterior()
{
	MatrixXd covMat = MatrixXd::Zero(stateDim, stateDim);

	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		MatrixXd filterCovPost = ptr->getCovPost();
		VectorXd filterStatePost = ptr->getStatePost();

		MatrixXd augmentedMat = MatrixXd::Zero(stateDim, stateDim);
		VectorXd augmentedState = VectorXd::Zero(stateDim);


		int rows = filterCovPost.rows();
		int cols = filterCovPost.cols();

		int modelStateDim = filterStatePost.size();

		augmentedMat.block(0, 0, rows, cols) = filterCovPost;
		augmentedState.head(modelStateDim) = filterStatePost;

		VectorXd diffState = augmentedState - getStatePosterior();

		covMat += ptr->getProbability()*(augmentedMat - diffState*diffState.transpose());
	}


	return covMat;
}


void MMAE::predict(VectorXd & control)
{

	//Make prediction for each element on the model bank
	for (std::shared_ptr<MMAEItem> ptr: filterBank)
	{

		ptr->predict(control);

	}

}

void MMAE::update(VectorXd & measure)
{

	//Perform the MMAE measurement updates
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		ptr->update(measure);

	}

	computeProbabilities(measure);

}



void MMAE::updateDeltaT(double deltaT)
{

	//THIS IS IMPORTANT! VERY IMPORTANT

	return;
}


std::vector<double> MMAE::getAllModelProbabilities()
{

	std::vector<double> ret;

	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		ret.push_back(ptr->getProbability());
	}
	

	return ret;
}

void MMAE::computeProbabilities(VectorXd & measure)
{
	//If no measurement available, keep the probabilities
	if (measure.size() < 1)
		return;

	double sumOfDensities = 0;

	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		ptr->computeProbabilityDensity(measure);
		sumOfDensities += ptr->getProbDensity()*ptr->getProbability();
	}

 
	if (sumOfDensities == 0)
	{
		std::cerr << "WARN: The sum of probability densities equals zero. Previous probabilities will be used." << std::endl;
		return;
	}


	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		double prob = ptr->getProbDensity()* ptr->getProbability() / sumOfDensities;
		ptr->setProbabiliy(prob);
	}

}
