#include "mex.h"
#include "MMAE_handle.hpp"
#include <MMAE.hpp>
#include <KalmanFilter.hpp>
#include <vector>
#include <deque>
#include <iostream>


// The class that we are interfacing t

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// Get the command string
	char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");

	// New
	if (!strcmp("new", cmd)) {
		// Check parameters
		if (nlhs != 1)
			mexErrMsgTxt("New: One output expected.");
		// Return a handle to a new C++ instance
		if (nrhs != 2)
			mexErrMsgTxt("Wrong inputs");

		//Number of fields of the structures
		int numFields = mxGetNumberOfFields(prhs[1]);

		if (numFields != 7)
			mexErrMsgTxt("Wrong structure. It should have 7 fields");

		//Get number of models
		int numModels = mxGetNumberOfElements(prhs[1]);


		//Let's create an  MMAEItem for each received model and add it
		//to the filterBank
		
		std::vector<std::shared_ptr<MMAEItem> > filterBank;

		for (int i = 0; i < numModels; i++)
		{
			//Get the name field
			mxArray *field_ptr = mxGetField(prhs[1], i, "modelName");

			char *name = mxArrayToString(field_ptr);
			std::string nameStr(name);

			//Get the state transition model
			mxArray *stateTransitionModel_mxArray = mxGetField(prhs[1], i, "stateTransitionModel");
			int stateTransM = mxGetM(stateTransitionModel_mxArray);
			int stateTransN = mxGetN(stateTransitionModel_mxArray);

			double *stateTransitionPTR = mxGetPr(stateTransitionModel_mxArray);
			MatrixXd stateTransitionModel(stateTransM, stateTransN);


			for (int m = 0; m < stateTransM; m++)
			{
				for (int n = 0; n < stateTransN; n++)
				{
					stateTransitionModel(m, n) = stateTransitionPTR[m + stateTransM*n];
				}
			}



			//Get the observation model matrix
			
			mxArray *observationModel_mxArray = mxGetField(prhs[1], i, "observationModel");
			int obsModelM = mxGetM(observationModel_mxArray);
			int obsModelN = mxGetN(observationModel_mxArray);
			double *obervationModelPTR = mxGetPr(observationModel_mxArray);
			MatrixXd observationModel(obsModelM, obsModelN);

			for (int m = 0; m < obsModelM; m++)
			{
				for (int n = 0; n < obsModelN; n++)
				{
					observationModel(m, n) = obervationModelPTR[m + obsModelM*n];
				}
			}


			//Get the processNoiseCovariance matrix
			mxArray *processNoiseCov_mxArray = mxGetField(prhs[1], i, "processNoiseCov");
			int processNoiseCovM = mxGetM(processNoiseCov_mxArray);
			int processNoiseCovN = mxGetN(processNoiseCov_mxArray);
			double *processNoiseCovPTR = mxGetPr(processNoiseCov_mxArray);
			MatrixXd processNoiseCov(processNoiseCovM, processNoiseCovN);

			for (int m = 0; m < processNoiseCovM; m++)
			{
				for (int n = 0; n < processNoiseCovN; n++)
				{
					processNoiseCov(m, n) = processNoiseCovPTR[m + processNoiseCovM*n];
				}
			}


			//Get the observation noise covariance
			mxArray *observationNoiseCov_mxArray = mxGetField(prhs[1], i, "observationNoiseCov");
			int observationNoiseCovM = mxGetM(observationNoiseCov_mxArray);
			int observationNoiseCovN = mxGetN(observationNoiseCov_mxArray);
			double *observationNoiseCovPTR = mxGetPr(observationNoiseCov_mxArray);
			MatrixXd observationNoiseCov(observationNoiseCovM, observationNoiseCovN);

			for (int m = 0; m < observationNoiseCovM; m++)
			{
				for (int n = 0; n < observationNoiseCovN; n++)
				{
					observationNoiseCov(m, n) = observationNoiseCovPTR[m + observationNoiseCovM*n];
				}
			}




			//Get the initial state
			mxArray *initialState_mxArray = mxGetField(prhs[1], i, "initialState");
			int initialStateM = mxGetM(initialState_mxArray);
			int initialStateN = mxGetN(initialState_mxArray);
			double *initialStatePTR = mxGetPr(initialState_mxArray);

			int vectorSize = 0;

			if (initialStateM > initialStateN)
			{
				vectorSize = initialStateM;
			}
			else
			{
				vectorSize = initialStateN;
			}

			VectorXd initialState(vectorSize);


			for (int m = 0; m < vectorSize; m++)
			{
				initialState(m) = initialStatePTR[m];
			}



			//Get the initial cov
			mxArray *initialCov_mxArray = mxGetField(prhs[1], i, "initialCov");
			int initialCovM = mxGetM(initialCov_mxArray);
			int initialCovN = mxGetN(initialCov_mxArray);
			double *initialCovPTR = mxGetPr(initialCov_mxArray);
			MatrixXd initialCov(initialCovM, initialCovN);

			for (int m = 0; m < initialCovM; m++)
			{
				for (int n = 0; n < initialCovN; n++)
				{
					initialCov(m, n) = initialCovPTR[m + initialCovM*n];
				}
			}




			std::shared_ptr<BaseBayesianFilter> model(new KalmanFilter(stateTransitionModel, observationModel,
				processNoiseCov, observationNoiseCov, initialState, initialCov));
			
			std::shared_ptr<MMAEItem> mmaeitem_ptr(new MMAEItem(model, nameStr));

			filterBank.push_back(mmaeitem_ptr);

		}


		MMAE *mmaestimator = new MMAE(filterBank);

		
		plhs[0] = convertPtr2Mat<MMAE>(mmaestimator);

		

		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object
		destroyObject<MMAE>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Get the class instance pointer from the second input
	MMAE *MMAE_instance = convertMat2Ptr<MMAE>(prhs[1]);


	if (!strcmp("getStatePrediction", cmd))
	{

		VectorXd statePred = MMAE_instance->getStatePrediction();

		plhs[0] = mxCreateDoubleMatrix(statePred.size(), 1, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < statePred.size(); m++)
			state[m] = statePred(m);

		return;
	}                                 

	if (!strcmp("getStateCovariancePrediction", cmd))
	{

		MatrixXd covMat = MMAE_instance->getStateCovariancePrediction();
		plhs[0] = mxCreateDoubleMatrix(covMat.rows(), covMat.cols(), mxREAL);

		double *covMatPredPTR = mxGetPr(plhs[0]);


		for (int m = 0; m < covMat.rows(); m++)
			for (int n = 0; n < covMat.cols(); n++)
				covMatPredPTR[m + covMat.rows()*n] = covMat(m, n);

		return;
	}

	if (!strcmp("getStatePosterior", cmd))
	{
		VectorXd statePost = MMAE_instance->getStatePosterior();

		plhs[0] = mxCreateDoubleMatrix(statePost.size(), 1, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < statePost.size(); m++)
			state[m] = statePost(m);

		return;
	}

	if (!strcmp("getStateCovariancePosterior", cmd))
	{

		MatrixXd covMat = MMAE_instance->getStateCovariancePosterior();
		plhs[0] = mxCreateDoubleMatrix(covMat.rows(), covMat.cols(), mxREAL);

		double *covMatPostPTR = mxGetPr(plhs[0]);


		for (int m = 0; m < covMat.rows(); m++)
			for (int n = 0; n < covMat.cols(); n++)
				covMatPostPTR[m + covMat.rows()*n] = covMat(m, n);

		return;
	}

	if (!strcmp("predict", cmd))
	{
		//Predict with no control signal
		if (nrhs == 2)
		{

			MMAE_instance->predict();

		}
		//Predict with control signal
		else if (nrhs == 3)
		{


		}
		else
		{
			mexErrMsgTxt("Invalid params.");
		}

		return;
	}

	if (!strcmp("update", cmd))
	{

		return;
	}

	if (!strcmp("getAllModelProbabilities", cmd))
	{

		std::vector<double> probs = MMAE_instance->getAllModelProbabilities();

		plhs[0] = mxCreateDoubleMatrix(probs.size(), 1, mxREAL);
		
		double *probsPTR = mxGetPr(plhs[0]);

		for (int i = 0; i< probs.size(); i++)
		{
			probsPTR[i] = probs.at(i);
		}

		return;
	}




	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}

