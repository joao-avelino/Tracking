#include "mex.h"
#include "OpenCvKF_handle.hpp"
#include <opencv2/opencv.hpp>
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
		if (nrhs != 7)
			mexErrMsgTxt("Wrong inputs");


		//Get the State Transition Matrix
		double *stateTransitionPTR = mxGetPr(prhs[1]);
		int stateTransM = mxGetM(prhs[1]);
		int stateTransN = mxGetN(prhs[1]);

		cv::Mat stateTransitionModel(stateTransM, stateTransN, CV_64F);


		for (int m = 0; m < stateTransM; m++)
		{
			for (int n = 0; n < stateTransN; n++)
			{
				stateTransitionModel.at<double>(m, n) = stateTransitionPTR[m + stateTransM*n];
			}
		}

		//Get the observationModel
		double *obsModelPTR = mxGetPr(prhs[2]);
		int obsModelM = mxGetM(prhs[2]);
		int obsModelN = mxGetN(prhs[2]);

		cv::Mat observationModel(obsModelM, obsModelN, CV_64F);


		for (int m = 0; m < obsModelM; m++)
		{
			for (int n = 0; n < obsModelN; n++)
			{
				observationModel.at<double>(m, n) = obsModelPTR[m + obsModelM*n];
			}
		}

		//Get the processNoiseCovariance
		double *processNoisePTR = mxGetPr(prhs[3]);
		int processNoiseM = mxGetM(prhs[3]);
		int processNoiseN = mxGetN(prhs[3]);

		cv::Mat processNoiseCovariance(processNoiseM, processNoiseN, CV_64F);


		for (int m = 0; m < processNoiseM; m++)
		{
			for (int n = 0; n < processNoiseN; n++)
			{
				processNoiseCovariance.at<double>(m, n) = processNoisePTR[m + processNoiseM*n];
			}
		}

		//Get the observationNoiseCov
		double *observationNoisePTR = mxGetPr(prhs[4]);
		int observationNoiseM = mxGetM(prhs[4]);
		int observationNoiseN = mxGetN(prhs[4]);

		cv::Mat observationNoiseCov(observationNoiseM, observationNoiseN, CV_64F);


		for (int m = 0; m < observationNoiseM; m++)
		{
			for (int n = 0; n < observationNoiseN; n++)
			{
				observationNoiseCov.at<double>(m, n) = observationNoisePTR[m + observationNoiseM*n];
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

		cv::Mat initial_state(dimsState, 1, CV_64F);

		if (initialStateN >= initialStateM)
		{
			for (int n = 0; n < initialStateN; n++)
			{
				initial_state.at<double>(n, 0) = initialStatePTR[0 + initialStateM*n];
			}
		}
		else {

			for (int m = 0; m < initialStateM; m++)
			{
				initial_state.at<double>(m, 0) = initialStatePTR[m + initialStateM * 0];
			}

		}

		//Get the Initial State Covariance Matrix
		double *initialStateCovPTR = mxGetPr(prhs[6]);
		int stateCovM = mxGetM(prhs[6]);
		int stateCovN = mxGetN(prhs[6]);

		cv::Mat initial_cov(stateCovM, stateCovN, CV_64F);


		for (int m = 0; m < stateCovM; m++)
		{
			for (int n = 0; n < stateCovN; n++)
			{
				initial_cov.at<double>(m, n) = initialStateCovPTR[m + stateCovM*n];
			}
		}
		
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
		mexPrintf(ss6.str().c_str());

		
		cv::KalmanFilter *openCvKF = new cv::KalmanFilter(initial_state.rows, observationNoiseM, 0, CV_64F);

		openCvKF->transitionMatrix = stateTransitionModel;
		openCvKF->measurementMatrix = observationModel;
		openCvKF->processNoiseCov = processNoiseCovariance;
		openCvKF->measurementNoiseCov = observationNoiseCov;
		openCvKF->statePost = initial_state;
		openCvKF->errorCovPost = initial_cov;
		
		plhs[0] = convertPtr2Mat<cv::KalmanFilter>(openCvKF);

		

		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object
		destroyObject<cv::KalmanFilter>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Get the class instance pointer from the second input
	cv::KalmanFilter *KalmanFilter_instance = convertMat2Ptr<cv::KalmanFilter>(prhs[1]);


	if (!strcmp("predict", cmd))
	{
		if (nrhs != 2)
			mexErrMsgTxt("Test: Unexpected arguments.");


		KalmanFilter_instance->predict();

		//Return the predicted state
		cv::Mat pred_state = KalmanFilter_instance->statePre;
		plhs[0] = mxCreateDoubleMatrix(pred_state.rows, pred_state.cols, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < pred_state.rows; m++)
			state[m] = pred_state.at<double>(m, 0);


		//Return the State Covariance Matrix
		cv::Mat covMat = KalmanFilter_instance->errorCovPre;
		plhs[1] = mxCreateDoubleMatrix(covMat.rows, covMat.cols, mxREAL);

		double *covMatPredPTR = mxGetPr(plhs[1]);


		for (int m = 0; m < covMat.rows; m++)
			for (int n = 0; n < covMat.cols; n++)
				covMatPredPTR[m + covMat.rows*n] = covMat.at<double>(m, n);

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

		cv::Mat measure(dimsState, 1, CV_64F);

		if (measureN >= measureM)
		{
			for (int n = 0; n < measureN; n++)
			{
				measure.at<double>(n, 0) = measurePTR[0 + measureM*n];
			}
		}
		else {

			for (int m = 0; m < measureM; m++)
			{
				measure.at<double>(m, 0) = measurePTR[m + measureM * 0];
			}

		}

		KalmanFilter_instance->correct(measure);
		//Return the predicted state
		cv::Mat post_state = KalmanFilter_instance->statePost;
		plhs[0] = mxCreateDoubleMatrix(post_state.rows, post_state.cols, mxREAL);

		double *state = mxGetPr(plhs[0]);

		for (int m = 0; m < post_state.rows; m++)
			state[m] = post_state.at<double>(m, 0);


		//Return the State Covariance Matrix
		cv::Mat covMat = KalmanFilter_instance->errorCovPost;
		plhs[1] = mxCreateDoubleMatrix(covMat.rows, covMat.cols, mxREAL);

		double *covMatPostPTR = mxGetPr(plhs[1]);


		for (int m = 0; m < covMat.rows; m++)
			for (int n = 0; n < covMat.cols; n++)
				covMatPostPTR[m + covMat.rows*n] = covMat.at<double>(m, n);

		return;

	}




	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}

