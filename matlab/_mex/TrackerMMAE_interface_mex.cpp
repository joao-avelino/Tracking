#include "mex.h"
#include "MMAE_handle.hpp"
#include <MMAE.hpp>
#include <KalmanFilter.hpp>
#include "TrackerWithBVT.hpp"
#include "Person3dBVT.hpp"
#include "Detection.hpp"
#include <vector>
#include <deque>
#include <iostream>
#include "util/eigen2matlab.hpp"


// The class that we are interfacing t

void mex55(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
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
		MatrixXd stateTransitionModel;
		MatrixXd observationModel;
		MatrixXd processNoiseCov;
		MatrixXd observationNoiseCov;
		VectorXd initialState;
		MatrixXd initialCov;

		for (int i = 0; i < numModels; i++)
		{
			//Get the name field
			mxArray *field_ptr = mxGetField(prhs[1], i, "modelName");

			char *name = mxArrayToString(field_ptr);
			std::string nameStr(name);

			//Get the state transition model
			mxArray *stateTransitionModel_mxArray = mxGetField(prhs[1], i, "stateTransitionModel");
			stateTransitionModel = matlabMatrixToEigen(stateTransitionModel_mxArray);
			
			//Get the observation model matrix
			mxArray *observationModel_mxArray = mxGetField(prhs[1], i, "observationModel");
			observationModel = matlabMatrixToEigen(observationModel_mxArray);

			//Get the processNoiseCovariance matrix
			mxArray *processNoiseCov_mxArray = mxGetField(prhs[1], i, "processNoiseCov");
			processNoiseCov = matlabMatrixToEigen(processNoiseCov_mxArray);

			//Get the observation noise covariance
			mxArray *observationNoiseCov_mxArray = mxGetField(prhs[1], i, "observationNoiseCov");
			observationNoiseCov = matlabMatrixToEigen(observationNoiseCov_mxArray);

			//Get the initial state
			mxArray *initialState_mxArray = mxGetField(prhs[1], i, "initialState");
			initialState = matlabVectorToEigen(initialState_mxArray);

			//Get the initial cov
			mxArray *initialCov_mxArray = mxGetField(prhs[1], i, "initialCov");
			initialCov = matlabMatrixToEigen(initialCov_mxArray);

			std::shared_ptr<BaseBayesianFilter> model(new KalmanFilter(stateTransitionModel, observationModel,
				processNoiseCov, observationNoiseCov, initialState, initialCov));
			
			std::shared_ptr<MMAEItem> mmaeitem_ptr(new MMAEItem(model, nameStr));

			filterBank.push_back(mmaeitem_ptr);

		}

		//From HERE on it's different ! ---- CHANGE-------
		int sizeObs = observationModel.rows();
		
		std::shared_ptr<Person3dBVT> person(new Person3dBVT(initialState.head(sizeObs), VectorXd(),
			initialCov.block(0, 0, sizeObs, sizeObs), 0.5));

		std::shared_ptr<MMAE> estim(new MMAE(filterBank));

		TrackerWithBVT<Person3dBVT, MMAE> *track = new TrackerWithBVT<Person3dBVT, MMAE>(person, estim, 0.5);

		
		plhs[0] = convertPtr2Mat<TrackerWithBVT<Person3dBVT, MMAE>>(track);

		

		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object
		destroyObject<TrackerWithBVT<Person3dBVT, MMAE>>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Get the class instance pointer from the second input
	TrackerWithBVT<Person3dBVT, MMAE> *Tracker_instance = convertMat2Ptr<TrackerWithBVT<Person3dBVT, MMAE>>(prhs[1]);


	if (!strcmp("getStatePrediction", cmd))
	{

		VectorXd statePred = Tracker_instance->positionEstimator->getStatePred();

		plhs[0] = eigenVectorToMatlab(statePred);

		return;
	}                                 

	if (!strcmp("getStateCovariancePrediction", cmd))
	{

		MatrixXd covMat = Tracker_instance->positionEstimator->getCovPred();
		plhs[0] = eigenMatrixToMatlab(covMat);

		return;
	}

	if (!strcmp("getStatePosterior", cmd))
	{
		VectorXd statePost = Tracker_instance->positionEstimator->getStatePost();

		plhs[0] = eigenVectorToMatlab(statePost);

		return;
	}

	if (!strcmp("getStateCovariancePosterior", cmd))
	{

		MatrixXd covMat = Tracker_instance->positionEstimator->getCovPost();
		plhs[0] = eigenMatrixToMatlab(covMat);

		return;
	}

	if (!strcmp("predict", cmd))
	{
		//Predict with no control signal
		if (nrhs == 2)
		{

			Tracker_instance->predict(VectorXd());

		}
		//Predict with control signal
		else if (nrhs == 3)
		{
			//Get the control signal
			VectorXd controlVec = matlabVectorToEigen(prhs[2]);
			Tracker_instance->predict(controlVec);
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
		MatrixXd covMat = Tracker_instance->positionEstimator->getCovPred();

		std::shared_ptr<Person3dBVT> detPerson(new Person3dBVT(meas, VectorXd(), covMat.block(0,0, meas.size(), meas.size()), 0.5));
		Detection<Person3dBVT> detection(detPerson, "Camera");

		Tracker_instance->update(detection);

		return;
	}

	if (!strcmp("getAllModelProbabilities", cmd))
	{

		std::vector<double> probs = Tracker_instance->positionEstimator->getAllModelProbabilities();

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