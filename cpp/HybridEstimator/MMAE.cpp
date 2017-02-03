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

MMAE::MMAE(std::vector<std::shared_ptr<MMAEItem> > filterBank, double minimumProbability, bool initializeProbs)
{
	this->filterBank = filterBank;

	this->minimumProbability = minimumProbability;

	stateDim = 0;
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		if (stateDim < ptr->getStateDim())
			stateDim = ptr->getStateDim();
	}

	obsDim = filterBank.at(0)->getObsDim();

	if (initializeProbs)
	{
		int nModels = filterBank.size();
		double prob = 1.0 / nModels;

		for (std::shared_ptr<MMAEItem> ptr : filterBank)
		{
			ptr->setProbabiliy(prob);
		}
	}


}


MMAE::~MMAE()
{
}

void MMAE::setStatePred(VectorXd statePred)
{
	for (auto& fil : filterBank)
	{
		fil->setStatePred(statePred);
	}

}

void MMAE::setCovPred(MatrixXd covPred)
{
	for (auto& fil : filterBank)
	{
		fil->setCovPred(covPred);
	}

}

void MMAE::setStatePost(VectorXd statePost)
{
	for (auto& fil : filterBank)
	{
		fil->setStatePost(statePost);
	}

}

void MMAE::setCovPost(MatrixXd covPost)
{
	for (auto& fil : filterBank)
	{
		fil->setCovPost(covPost);
	}

}

VectorXd MMAE::getStatePred()
{

	//Set up a vector for the states
	VectorXd stateMixture = VectorXd::Zero(stateDim);


	//Get the mixture of states
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		VectorXd augmentedState = VectorXd::Zero(stateDim);
		VectorXd partialVector = ptr->getStatePred()*ptr->getProbability();
		
		int modelStateDim = partialVector.size();
		augmentedState.head(modelStateDim) = partialVector;

		stateMixture += augmentedState;
	}

	return stateMixture;
}

MatrixXd MMAE::getCovPred()
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

		VectorXd diffState = augmentedState - getStatePred();

		covMat += ptr->getProbability()*(augmentedMat-diffState*diffState.transpose());
	}


	return covMat;
}

VectorXd MMAE::getStatePost()
{

	//Set up a vector for the states
	VectorXd stateMixture = VectorXd::Zero(stateDim);


	//Get the mixture of states
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		VectorXd augmentedState = VectorXd::Zero(stateDim);
		VectorXd partialVector = ptr->getStatePost()*ptr->getProbability();

		int modelStateDim = partialVector.size();
		augmentedState.head(modelStateDim) = partialVector;

		stateMixture += augmentedState;
	}

	return stateMixture;

}

MatrixXd MMAE::getCovPost()
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

		VectorXd diffState = augmentedState - getStatePost();

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


void MMAE::update(VectorXd & measure, MatrixXd &measurementCov)
{

	//Perform the MMAE measurement updates
	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{

		ptr->update(measure, measurementCov);

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

std::shared_ptr<BaseBayesianFilter> MMAE::clone()
{
	/*Clone each element in the vector*/

	std::vector<std::shared_ptr<MMAEItem> > filterBankClone;

	for (auto& fil : this->filterBank)
	{
		std::shared_ptr<MMBankItem> generalItemPTR = fil->clone();
		filterBankClone.push_back(std::static_pointer_cast<MMAEItem>(generalItemPTR));
	}

    return std::shared_ptr<HybridEstimator>(new MMAE(filterBankClone, this->minimumProbability, true));
}

std::shared_ptr<BaseBayesianFilter> MMAE::clone(VectorXd initial_state)
{
	/*Clone each element in the vector*/

	std::vector<std::shared_ptr<MMAEItem> > filterBankClone;

	for (auto& fil : this->filterBank)
	{
		std::cout << "Cloning MMAE element" << std::endl;
		std::shared_ptr<MMBankItem> generalItemPTR = fil->clone(initial_state);

		std::cout << "State pred: " << generalItemPTR->getStatePred() << std::endl;
		std::cout << "State post: " << generalItemPTR->getStatePost() << std::endl;

		filterBankClone.push_back(std::static_pointer_cast<MMAEItem>(generalItemPTR));
		std::cout << "Cloned MMAE element" << std::endl;
	}

    return std::shared_ptr<HybridEstimator>(new MMAE(filterBankClone, this->minimumProbability, true));
}

std::shared_ptr<BaseBayesianFilter> MMAE::clone(VectorXd initial_state, MatrixXd measurementCov)
{
	/*Clone each element in the vector*/

	std::vector<std::shared_ptr<MMAEItem> > filterBankClone;

	for (auto& fil : this->filterBank)
	{
		std::shared_ptr<MMBankItem> generalItemPTR = fil->clone(initial_state, measurementCov);
		filterBankClone.push_back(std::static_pointer_cast<MMAEItem>(generalItemPTR));
	}

    return std::shared_ptr<HybridEstimator>(new MMAE(filterBankClone, this->minimumProbability, true));
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


	double sumOfProbabilities = 0;
	bool needToRenormalize = false;

	for (std::shared_ptr<MMAEItem> ptr : filterBank)
	{
		double prob = ptr->getProbDensity()* ptr->getProbability() / sumOfDensities;


		if (prob < minimumProbability)
		{
			prob = minimumProbability+pow(minimumProbability, 2);
			needToRenormalize = true;
			
		}
		sumOfProbabilities += prob;

		ptr->setProbabiliy(prob);

		

	}

	//Renormalize probabilities if needed
	if (needToRenormalize)
	{
		for (std::shared_ptr<MMAEItem> ptr : filterBank)
		{
			ptr->setProbabiliy(ptr->getProbability() / sumOfProbabilities);
		}
	}



}


