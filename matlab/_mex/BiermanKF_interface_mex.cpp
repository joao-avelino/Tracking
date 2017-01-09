#include "mex.h"
#include "BiermanKF_handle.hpp"
#include <vector>
#include <deque>
#include <KalmanFilter.hpp>

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
	if(nrhs != 2)
	    mexErrMsgTxt("Wrong inputs");

	
	
	MatrixXd initial_cov;



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
		std::cerr << "Expecting the state vector but received a matrix" << std::endl;
		exit(-1);
	}

	int dimsState = std::max(initialStateM, initialStateN);

	VectorXd initial_state(dimsState);

	if(initialStateN >= initialStateM)
	{
		for (int n = 0; n < initialStateN; n++)
		{
			observationNoiseCov(n) = observationNoisePTR[0 + observationNoiseM*n];
		}
	}
	else {

		for (int m = 0; m < initialStateM; m++)
		{
			observationNoiseCov(m) = observationNoisePTR[m + observationNoiseM*0];
		}

	}


    
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

	/*
    if(!strcmp("updateDeltaT", cmd))
	{
	  if(nrhs != 3)
	    mexErrMsgTxt("Test: Unexpected arguments.");
	  double deltaT;
	  deltaT = *mxGetPr(prhs[2]);
	  KalmanFilter_instance->updateDeltaT(deltaT);
	  return;
	}*/

 
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
