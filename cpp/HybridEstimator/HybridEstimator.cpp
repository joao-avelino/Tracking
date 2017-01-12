#include "HybridEstimator.hpp"

HybridEstimator::HybridEstimator(double deltaT, std::vector<BaseKalmanFilter> modelList)
{
	this->deltaT = deltaT;
	this->modelList = modelList;

}
