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
	if(nrhs != 7)
	    mexErrMsgTxt("Wrong inputs");

	MatrixXd stateTransitionModel;
	MatrixXd observationModel;
	MatrixXd processNoiseCovariance;
	MatrixXd observationNoiseCov;
	VectorXd initial_state;
	MatrixXd initial_cov;


	double *stateTransitionPTR = mxGetPr(prhs[1]);
	int stateTransM = mxGetM(prhs[1]);
	int stateTransN = mxGetN(prhs[1]);

	//Get the State Transition Matrix

	for (int n = 0; n < stateTransN; n++)
	{

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

    if(!strcmp("updateDeltaT", cmd))
	{
	  if(nrhs != 3)
	    mexErrMsgTxt("Test: Unexpected arguments.");
	  double deltaT;
	  deltaT = *mxGetPr(prhs[2]);
	  KalmanFilter_instance->updateDeltaT(deltaT);
	  return;
	}

    if(!strcmp("associateData", cmd))
	{
	  if(nrhs != 7)
	    mexErrMsgTxt("Test: Unexpected arguments.");
	  double *matx;
	  int M;
	  int N;
	  M = mxGetM(prhs[2]);
	  N = mxGetN(prhs[2]);
	  matx = mxGetPr(prhs[2]);
		
	  int Mrects = mxGetM(prhs[3]);
	  int Nrects = mxGetN(prhs[3]);
	  double *rects = mxGetPr(prhs[3]);

	  int Mcolors = mxGetM(prhs[4]);
	  int Ncolors = mxGetN(prhs[4]);
	  double *colorFeatures = mxGetPr(prhs[4]);
      
	  int Mmeans = mxGetM(prhs[5]);
	  int Nmeans = mxGetN(prhs[5]);
	  double *means = mxGetPr(prhs[5]);
      
      size_t Kcov = mxGetNumberOfDimensions(prhs[6]);
      const mwSize *Ncov = mxGetDimensions(prhs[6]);
      double *covs = mxGetPr(prhs[6]);
      
      //Code to extract means...
      std::vector<cv::Mat> meansArray;
      
      for(int nMeans = 0; nMeans < Nmeans; nMeans++)
      {
        double ux = means[0+Mmeans*nMeans];
        double uy = means[1+Mmeans*nMeans];
        
        cv::Mat meanDiscreteError = (cv::Mat_<double>(2, 1) << ux, uy);
        meansArray.push_back(meanDiscreteError);
      }
      //........................
      

      
      //Convert the 3D matrix to a vector of covariance Mats
      
      std::vector<cv::Mat> covMatrices;
      
      for(int n2 = 0; n2 < Ncov[2]; n2++)
      {
          double a00 = covs[0+Ncov[0]*(0+Ncov[1]*n2)];
          double a01 = covs[0+Ncov[0]*(1+Ncov[1]*n2)];
          double a10 = covs[1+Ncov[0]*(0+Ncov[1]*n2)];
          double a11 = covs[1+Ncov[0]*(1+Ncov[1]*n2)];
          
          cv::Mat covMatrix = (cv::Mat_<double>(2, 2) << a00, a01, a10, a11);
          covMatrices.push_back(covMatrix);
      }

	  //Convert the points matrix to a std::vector
	  std::vector<cv::Point3d> basePoints;
	  
	  for(int n = 0; n<N; n++)
	    {
	      cv::Point3d point;
	      point.x = matx[0+M*n];
	      point.y = matx[1+M*n];
	      point.z = matx[2+M*n];
	      basePoints.push_back(point);
	    }
	  
	  //Convert the rectangles to a list of cv::Rect_
	  std::vector<cv::Rect_<int> > bboxes;
	  for(int m = 0; m < Mrects; m++)
	    {
	      int topLeftX;
	      int topLeftY;
	      int width;
	      int height;

	      topLeftX = rects[m+Mrects*0]; //first column
	      topLeftY = rects[m+Mrects*1]; //second column
	      width = rects[m+Mrects*2]; //third column
	      height = rects[m+Mrects*3]; //fourth column

	      cv::Rect_<int> bbox(topLeftX, topLeftY, width, height);
	      bboxes.push_back(bbox);
	    }

	  //Color features
	  std::vector<cv::Mat> colorFeaturesList;
	  for(int m = 0; m < Mcolors; m++) //For each row -> each histogram
	    {
	      cv::Mat bvtHist = cv::Mat::zeros(1, Ncolors, CV_32F);;
	      for(int n = 0; n < Ncolors; n++) // Fill a mat
	        {
		  bvtHist.at<float>(0,n) = colorFeatures[m+Mcolors*n];
		}
	    
	      colorFeaturesList.push_back(bvtHist);
	    }

	  PersonList_instance->associateData(basePoints, bboxes, colorFeaturesList, meansArray, covMatrices);

	  //Kill tracklets that need to be killed
	  PersonList_instance->trackletKiller();

	  int numTracklets = PersonList_instance->personList.size();

	  //Return a matrix containing the 3D head points


	  // [x, y z, id] -> each tracklet a row
	  plhs[0] = mxCreateDoubleMatrix(numTracklets, 4, mxREAL);
	  
	  // each tracklet a row
          plhs[1] = mxCreateDoubleMatrix(numTracklets, 3, mxREAL);	  

	  double *trackingPoints;
	  double *probabilities;

	  trackingPoints = mxGetPr(plhs[0]);
	  probabilities = mxGetPr(plhs[1]);
	  
	  int mOut = 0;
	  
	  for(vector<PersonModel>::iterator it = PersonList_instance->personList.begin(); it != PersonList_instance->personList.end(); it++, mOut++)
            {
		Point3d head = it->getPositionEstimate();
		trackingPoints[mOut + numTracklets*0] = head.x;
		trackingPoints[mOut + numTracklets*1] = head.y;
		trackingPoints[mOut + numTracklets*2] = head.z;
		trackingPoints[mOut + numTracklets*3] = it->id;

		int nOut = 0;
		for(std::vector<double>::iterator pr = it->mmaeEstimator->probabilities.begin(); pr != it->mmaeEstimator->probabilities.end(); pr++, nOut++)
                  {
		    probabilities[mOut+numTracklets*nOut] = *pr;
		  }
	    }

	  return;
	}
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
