#include "mex.h"
#include "opencvmex.hpp"
#include <opencv2/opencv.hpp>
#include "colorFeatures.hpp"

#define inputImage prhs[0]
#define bgBins prhs[1]

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  int bins = *mxGetPr(bgBins);
  
  if(nrhs != 2)
     mexErrMsgTxt("Test: Unexpected arguments.");

  cv::Ptr<cv::Mat> inImage = ocvMxArrayToImage_uint8(inputImage, true);
  cv::Mat bvtHistogramOut;

  std::vector<cv::Rect> partMasks;  

      cv::FileStorage fs("partMasks.yaml", cv::FileStorage::READ);

    std::vector<int> headVect;
    std::vector<int> torsoVect;
    std::vector<int> legsVect;
    std::vector<int> feetVect;

    fs["headMask"] >> headVect;
    fs["torsoMask"] >> torsoVect;
    fs["legsMask"] >> legsVect;
    fs["feetMask"] >> feetVect;

    cv::Rect headMask(headVect[0], headVect[1], headVect[2], headVect[3]);
    cv::Rect torsoMask(torsoVect[0], torsoVect[1], torsoVect[2], torsoVect[3]);
    cv::Rect legsMask(legsVect[0], legsVect[1], legsVect[2], legsVect[3]);
    cv::Rect feetMask(feetVect[0], feetVect[1], feetVect[2], feetVect[3]);

    fs.release();

    partMasks.push_back(headMask);
    partMasks.push_back(torsoMask);
    partMasks.push_back(legsMask);
    partMasks.push_back(feetMask);

    cv::Mat bvtHistogram;
    cv::Mat resizedPerson;

    
    cv::resize(*inImage, resizedPerson, cv::Size(52, 128));
  
    extractBVT(resizedPerson, bvtHistogram, bins, partMasks);
    bvtHistogram.convertTo(bvtHistogram, CV_32FC1);

    plhs[0] = ocvMxArrayFromMat_single(bvtHistogram);

    return;
    
}
