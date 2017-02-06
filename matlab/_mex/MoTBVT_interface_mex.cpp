#include "mex.h"
#include "MoTBVT_handle.hpp"
#include <MMAE.hpp>
#include <KalmanFilter.hpp>
#include <TrackerWithBVT.hpp>
#include <SimpleTrackerManager.hpp>
#include <HungarianAssociator.hpp>
#include <MultiObjectTracker.hpp>
#include <Person3dBVT.hpp>
#include <vector>
#include <deque>
#include <iostream>
#include "util/eigen2matlab.hpp"

//Create a nickname for the difficult templates
typedef TrackerWithBVT<Person3dBVT, MMAE> MMAEpersonTracker;
typedef SimpleTrackerManager<Person3dBVT, MMAEpersonTracker> TrackerManager;
typedef HungarianAssociator<Person3dBVT, TrackerWithBVT<Person3dBVT, MMAE>> PeopleHungarianAssociator;
typedef MultiObjectTracker<MMAEpersonTracker, TrackerManager, PeopleHungarianAssociator, Person3dBVT> MoT;
typedef AssociationList<Person3dBVT, MMAEpersonTracker> PeopleAssociations;
typedef Association<Person3dBVT, MMAEpersonTracker> PersonTrackerAssociation;


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
			mexErrMsgTxt("Wrong inputs. 'new', [estimator structure], [histogram size], [color learning rate], [comparison mode], [metric], [numFramesBeforeDelete]");

		//Number of fields of the structures
		int numFields = mxGetNumberOfFields(prhs[1]);

		if (numFields != 7)
			mexErrMsgTxt("Wrong filter structure. It should have 7 fields");

		//Get number of models
		int numModels = mxGetNumberOfElements(prhs[1]);

		//Get the size of the BVT histogram
		int bvtSize = *mxGetPr(prhs[2]);

		//Get the color learning rate
		double learningRate = *mxGetPr(prhs[3]);

		//Get the comparison mode and metric
		int comparisonMode = *mxGetPr(prhs[4]);
		int metric = *mxGetPr(prhs[5]);

		int numFramesBeforeDelete = *mxGetPr(prhs[6]);


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

		//Let's build the tracker
		int sizeObs = observationModel.rows();

		//Create an object
		shared_ptr<Person3dBVT> dummy_person(new Person3dBVT(VectorXd::Zero(sizeObs), VectorXd::Zero(bvtSize), MatrixXd::Zero(sizeObs, sizeObs)));
		std::shared_ptr<MMAE> estim(new MMAE(filterBank));

		//Create a Tracker
		MMAEpersonTracker *trackerPTR = new MMAEpersonTracker(dummy_person, estim, learningRate);

		//Create an associator
		PeopleHungarianAssociator hung(comparisonMode, metric);

		//Create a tracker manager
		TrackerManager trkMgr(numFramesBeforeDelete);


		//Create a MoT
		MoT *multipersontracker = new MoT(trkMgr, hung, trackerPTR);

		plhs[0] = convertPtr2Mat<MoT>(multipersontracker);

		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object
		destroyObject<MoT>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 2)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Get the class instance pointer from the second input
	MoT *mot_instance = convertMat2Ptr<MoT>(prhs[1]);

	
	if (!strcmp("processData", cmd))
	{

		//Check the number of inputs
		if (nrhs != 3 )
			mexErrMsgTxt("Wrong inputs. 'processData', [object handle], [histogram size]");


		//Number of fields of the structures
		int numFields = mxGetNumberOfFields(prhs[2]);

		if (numFields != 4)
			mexErrMsgTxt("Wrong detection structure. It should have 4 fields");


		//Get a struct array of detections
		/*
		*------------------- Detection structure ------------------
		*
		*   pointsOnWorld -> 3d array [x, y, height]
		*	bvtHistogram -> colors histogram
		*	meanDetectionError -> the mean of detection error
		*	covDetectionError -> the covariance of the detection
		*
		*-----------------------------------------------------------
		*/

		//Get number of detections
		int numDetections = mxGetNumberOfElements(prhs[2]);

		vector<shared_ptr<Detection<Person3dBVT>>> detectionList;
		VectorXd pointsOnWorld;
		VectorXd bvtHistogram;
		VectorXd meanDetectionError;
		MatrixXd covDetectionError;

		for (int i = 0; i < numDetections; i++)
		{

			//Get the pointsOnWorld field
			mxArray *pointsOnWorld_mxArray = mxGetField(prhs[2], i, "pointsOnWorld");
			pointsOnWorld = matlabVectorToEigen(pointsOnWorld_mxArray);

			//Get the bvtHistogram field
			mxArray *bvtHistogram_mxArray = mxGetField(prhs[2], i, "bvtHistogram");
			bvtHistogram = matlabVectorToEigen(bvtHistogram_mxArray);

			//Get the meanDetectionError field
			mxArray *meanDetectionError_mxArray = mxGetField(prhs[2], i, "meanDetectionError");
			meanDetectionError = matlabVectorToEigen(meanDetectionError_mxArray);

			//Get the covDetectionError field
			mxArray *covDetectionError_mxArray = mxGetField(prhs[2], i, "covDetectionError");
			covDetectionError = matlabMatrixToEigen(covDetectionError_mxArray);

			//Build the detection
			shared_ptr<Person3dBVT> det(new Person3dBVT(pointsOnWorld- meanDetectionError, bvtHistogram, covDetectionError));
			shared_ptr<Detection<Person3dBVT>> detection(new Detection<Person3dBVT>(det, "Camera"));
			detectionList.push_back(detection);

		}


		mot_instance->processDetections(detectionList);

		//Get the list of current trackers
		auto trkVect = mot_instance->getTrackersVector();

		MatrixXd matrixOfTrakckers = MatrixXd::Zero(trkVect.size(), pointsOnWorld.size() + 1);

		// [x, y z, id] -> each tracker a row

		int i = 0;
		for (auto& trk : trkVect)
		{
			VectorXd trackrow = VectorXd::Zero(pointsOnWorld.size() + 1);

			double id = trk->getTrackerId();

			trackrow.head(pointsOnWorld.size()) = trk->getObjPTR()->getObservableStates();
			trackrow(pointsOnWorld.size()) = id;

			matrixOfTrakckers.row(i) = trackrow;
			i++;
		}

		plhs[0] = eigenMatrixToMatlab(matrixOfTrakckers);

		return;
	}                                 

	if (!strcmp("getStateCovariancePrediction", cmd))
	{


	}
	if (!strcmp("getStateCovariancePosterior", cmd))
	{


	}

	if (!strcmp("getValidationGates", cmd))
	{


	}


	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}