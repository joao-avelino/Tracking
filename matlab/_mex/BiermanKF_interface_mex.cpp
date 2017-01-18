#include "mex.h"
#include "BiermanKF_handle.hpp"
#include <vector>
#include <deque>
#include <KalmanFilter.hpp>

#include <iostream>


// The class that we are interfacing t

void teste(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
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
		if (nrhs != 7)
			mexErrMsgTxt("Wrong inputs");


		//Get the State Transition Matrix
		double *stateTransitionPTR = mxGetPr(prhs[1]);
		int stateTransM = mxGetM(prhs[1]);
		int stateTransN = mxGetN(prhs[1]);

		MatrixXd stateTransitionModel(stateTransM, stateTransN);


		for (int m = 0; m < stateTransM; m++)
		{
			for (int n = 0; n < stateTransN; n++)
			{
				stateTransitionModel(m, n) = stateTransitionPTR[m + stateTransM*n];
			}
		}

		//Get the observationModel
		double *obsModelPTR = mxGetPr(prhs[2]);
		int obsModelM = mxGetM(prhs[2]);
		int obsModelN = mxGetN(prhs[2]);

		MatrixXd observationModel(obsModelM, obsModelN);


		for (int m = 0; m < obsModelM; m++)
		{
			for (int n = 0; n < obsModelN; n++)
			{
				observationModel(m, n) = obsModelPTR[m + obsModelM*n];
			}
		}

		//Get the processNoiseCovariance
		double *processNoisePTR = mxGetPr(prhs[3]);
		int processNoiseM = mxGetM(prhs[3]);
		int processNoiseN = mxGetN(prhs[3]);

		MatrixXd processNoiseCovariance(processNoiseM, processNoiseN);


		for (int m = 0; m < processNoiseM; m++)
		{
			for (int n = 0; n < processNoiseN; n++)
			{
				processNoiseCovariance(m, n) = processNoisePTR[m + processNoiseM*n];
			}
		}

		//Get the observationNoiseCov
		double *observationNoisePTR = mxGetPr(prhs[4]);
		int observationNoiseM = mxGetM(prhs[4]);
		int observationNoiseN = mxGetN(prhs[4]);

		MatrixXd observationNoiseCov(observationNoiseM, observationNoiseN);


		for (int m = 0; m < observationNoiseM; m++)
		{
			for (int n = 0; n < observationNoiseN; n++)
			{
				observationNoiseCov(m, n) = observationNoisePTR[m + observationNoiseM*n];
			}
		}

		//Get the initial_state
		double *initialStatePTR = mxGetPr(prhs[5]);
		int initialStateM = mxGetM(prhs[5]);
		int initialStateN = mxGetN(prhs[5]);

		if (initialStateM > 1 && initialStateN > 1)
		{
			mexErrMsgTxt("Expecting the state vector but received a matrix");
			exit(-1);
		}

		int dimsState = std::max(initialStateM, initialStateN);

		VectorXd initial_state(dimsState);

		if (initialStateN >= initialStateM)
		{
			for (int n = 0; n < initialStateN; n++)
			{
				initial_state(n) = initialStatePTR[0 + initialStateM*n];
			}
		}
		else {

			for (int m = 0; m < initialStateM; m++)
			{
				initial_state(m) = initialStatePTR[m + initialStateM * 0];
			}

		}

		//Get the Initial State Covariance Matrix
		double *initialStateCovPTR = mxGetPr(prhs[6]);
		int stateCovM = mxGetM(prhs[6]);
		int stateCovN = mxGetN(prhs[6]);

		MatrixXd initial_cov(stateCovM, stateCovN);


		for (int m = 0; m < stateCovM; m++)
		{
			for (int n = 0; n < stateCovN; n++)
			{
				initial_cov(m, n) = initialStateCovPTR[m + stateCovM*n];
			}
		}
		/*
		std::stringstream ss;
		ss << "stateTransitionModel: " << stateTransitionModel << std::endl;
		mexPrintf(ss.str().c_str());

		std::stringstream ss2;
		ss2 << "observationModel: " << observationModel << std::endl;
		mexPrintf(ss2.str().c_str());

		std::stringstream ss3;
		ss3 << "processNoiseCovariance: " << processNoiseCovariance << std::endl;
		mexPrintf(ss3.str().c_str());

		std::stringstream ss4;
		ss4 << "observationNoiseCov: " << observationNoiseCov << std::endl;
		mexPrintf(ss4.str().c_str());

		std::stringstream ss5;
		ss5 << "initial_state: " << initial_state << std::endl;
		mexPrintf(ss5.str().c_str());

		std::stringstream ss6;
		ss6 << "initial_cov_: " << initial_cov << std::endl;
		mexPrintf(ss6.str().c_str());*/

		plhs[0] = convertPtr2Mat<KalmanFilter>(new KalmanFilter(stateTransitionModel, observationModel, processNoiseCovariance, observationNoiseCov,
			initial_state, initial_cov));
		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object
		destroyObject<KalmanFilter>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Get the class instance pointer from the second input
	KalmanFilter *KalmanFilter_instance = convertMat2Ptr<KalmanFilter>(prhs[1]);

	
	if(!strcmp("predict", cmd))
	{
	  if(nrhs != 2)
		mexErrMsgTxt("Test: Unexpected arguments.");

	  
	  KalmanFilter_instance->predict();

	  //Return the predicted state
	  VectorXd pred_state = KalmanFilter_instance->getStatePred();
	  plhs[0] = mxCreateDoubleMatrix(pred_state.size(), 1, mxREAL);

	  double *state = mxGetPr(plhs[0]);

	  for (int m = 0; m < pred_state.size(); m++)
		  state[m] = pred_state(m);


	  //Return the State Covariance Matrix
	  MatrixXd covMat = KalmanFilter_instance->getCovPred();
	  plhs[1] = mxCreateDoubleMatrix(covMat.rows(), covMat.cols(), mxREAL);

	  double *covMatPredPTR = mxGetPr(plhs[1]);


	  for (int m = 0; m < covMat.rows(); m++)
		  for (int n = 0; n < covMat.cols(); n++)
			  covMatPredPTR[m + covMat.rows()*n] = covMat(m, n);

	  return;
	}

	if (!strcmp("update", cmd))
	{
		if (nrhs != 3)
			mexErrMsgTxt("Test: Unexpected arguments.");

		//Get the measurement
		double *measurePTR = mxGetPr(prhs[2]);
		int measureM = mxGetM(prhs[2]);
		int measureN = mxGetN(prhs[2]);

		if (measureM > 1 && measureN > 1)
		{
			mexErrMsgTxt("Expecting the state vector but received a matrix");
			exit(-1);
		}

		int dimsState = std::max(measureM, measureN);

		VectorXd measure(dimsState);

		if (measureN >= measureM)
		{
			for (int n = 0; n < measureN; n++)
			{
				measure(n) = measurePTR[0 + measureM*n];
			}
		}
		else {

			for (int m = 0; m < measureM; m++)
			{
				measure(m) = measurePTR[m + measureM * 0];
			}

		}

		KalmanFilter_instance->update(measure);
		//Return the predicted state
		VectorXd post_state = KalmanFilter_instance->getStatePost();
		plhs[0] = mxCreateDoubleMatrix(post_state.size(), 1, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < post_state.size(); m++)
			state[m] = post_state(m);


		//Return the State Covariance Matrix
		MatrixXd covMat = KalmanFilter_instance->getCovPost();
		plhs[1] = mxCreateDoubleMatrix(covMat.rows(), covMat.cols(), mxREAL);

		double *covMatPostPTR = mxGetPr(plhs[1]);


		for (int m = 0; m < covMat.rows(); m++)
			for (int n = 0; n < covMat.cols(); n++)
				covMatPostPTR[m + covMat.rows()*n] = covMat(m, n);

		return;

	}

	if (!strcmp("update_empty", cmd))
	{
		KalmanFilter_instance->update();
		//Return the predicted state
		VectorXd post_state = KalmanFilter_instance->getStatePost();
		plhs[0] = mxCreateDoubleMatrix(post_state.size(), 1, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < post_state.size(); m++)
			state[m] = post_state(m);


		//Return the State Covariance Matrix
		MatrixXd covMat = KalmanFilter_instance->getCovPost();
		plhs[1] = mxCreateDoubleMatrix(covMat.rows(), covMat.cols(), mxREAL);

		double *covMatPostPTR = mxGetPr(plhs[1]);


		for (int m = 0; m < covMat.rows(); m++)
			for (int n = 0; n < covMat.cols(); n++)
				covMatPostPTR[m + covMat.rows()*n] = covMat(m, n);

		return;
	}




	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}
