#include "mex.h"
#include "MMAE_handle.hpp"
#include <MMAE.hpp>
#include <KalmanFilter.hpp>
#include <vector>
#include <deque>
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
		if (nrhs != 2)
			mexErrMsgTxt("Wrong inputs");

		//Number of fields of the structures
		int numFields = mxGetNumberOfFields(prhs[0]);

		if (numFields != 7)
			mexErrMsgTxt("Wrong structure. It should have 7 fields");

		//Get number of models
		int numModels = mxGetNumberOfElements(prhs[0]);


		//Let's create an  MMAEItem for each received model and add it
		//to the filterBank
		
		std::vector<std::shared_ptr<MMAEItem> > filterBank;

		for (int i = 0; i < numModels; i++)
		{
			//Get the name field
			mxArray *field_ptr = mxGetField(prhs[0], i, "modelName");

			char *name = mxArrayToString(field_ptr);
			std::string nameStr(name);

			//Get the state transition model
			mxArray *stateTransitionModel_mxArray = mxGetField(prhs[0], i, "stateTransitionModel");
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




			MatrixXd observationModel;






			MatrixXd processNoiseCov;






			MatrixXd observationNoiseCov;






			VectorXd initialState;





			MatrixXd initialCov;





			std::shared_ptr<BaseBayesianFilter> model(new KalmanFilter(stateTransitionModel, observationModel,
				processNoiseCov, observationNoiseCov, initialState, initialCov));
			
			std::shared_ptr<MMAEItem> mmaeitem_ptr(new MMAEItem(model, nameStr));


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


	if (!strcmp("predict", cmd))
	{


		return;
	}

	if (!strcmp("update", cmd))
	{

		return;
	}

	if (!strcmp("update_empty", cmd))
	{

		return;
	}




	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}

