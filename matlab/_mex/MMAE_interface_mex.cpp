#include "mex.h"
#include "MMAE_handle.hpp"
#include <MMAE.hpp>
#include <KalmanFilter.hpp>
#include <vector>
#include <deque>
#include <iostream>
#include "util/eigen2matlab.hpp"

// The class that we are interfacing t

void teste10(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
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
			MatrixXd stateTransitionModel = matlabMatrixToEigen(stateTransitionModel_mxArray);
			
			//Get the observation model matrix
			mxArray *observationModel_mxArray = mxGetField(prhs[1], i, "observationModel");
			MatrixXd observationModel = matlabMatrixToEigen(observationModel_mxArray);

			//Get the processNoiseCovariance matrix
			mxArray *processNoiseCov_mxArray = mxGetField(prhs[1], i, "processNoiseCov");
			MatrixXd processNoiseCov = matlabMatrixToEigen(processNoiseCov_mxArray);

			//Get the observation noise covariance
			mxArray *observationNoiseCov_mxArray = mxGetField(prhs[1], i, "observationNoiseCov");
			MatrixXd observationNoiseCov = matlabMatrixToEigen(observationNoiseCov_mxArray);

			//Get the initial state
			mxArray *initialState_mxArray = mxGetField(prhs[1], i, "initialState");
			VectorXd initialState = matlabVectorToEigen(initialState_mxArray);

			//Get the initial cov
			mxArray *initialCov_mxArray = mxGetField(prhs[1], i, "initialCov");
			MatrixXd initialCov = matlabMatrixToEigen(initialCov_mxArray);

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

		VectorXd statePred = MMAE_instance->getStatePred();

		plhs[0] = eigenVectorToMatlab(statePred);

		return;
	}                                 

	if (!strcmp("getStateCovariancePrediction", cmd))
	{

		MatrixXd covMat = MMAE_instance->getCovPred();
		plhs[0] = eigenMatrixToMatlab(covMat);

		return;
	}

	if (!strcmp("getStatePosterior", cmd))
	{
		VectorXd statePost = MMAE_instance->getStatePost();

		plhs[0] = eigenVectorToMatlab(statePost);

		return;
	}

	if (!strcmp("getStateCovariancePosterior", cmd))
	{

		MatrixXd covMat = MMAE_instance->getCovPost();
		plhs[0] = eigenMatrixToMatlab(covMat);

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
			//Get the control signal
			VectorXd controlVec = matlabVectorToEigen(prhs[2]);
			MMAE_instance->predict(controlVec);
		}
		else
		{
			mexErrMsgTxt("Invalid params.");
		}

		return;
	}

	if (!strcmp("update", cmd))
	{
		//Get the measurement vector
		VectorXd meas = matlabVectorToEigen(prhs[2]);
		MMAE_instance->update(meas);
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