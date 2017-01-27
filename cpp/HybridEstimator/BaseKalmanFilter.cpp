#include "BaseKalmanFilter.hpp"


VectorXd BaseKalmanFilter::getStatePred()
{
    return statePred;
}

MatrixXd BaseKalmanFilter::getCovPred()
{
    return covPred;
}


VectorXd BaseKalmanFilter::getStatePost()
{
    return statePost;
}

MatrixXd BaseKalmanFilter::getCovPost()
{
    return covPost;
}

MatrixXd BaseKalmanFilter::getKalmanGain()
{
	return kalmanGain;
}

void BaseKalmanFilter::setDeltaT(double deltaT)
{
    this->deltaT = deltaT;

	//TODO

}

void BaseKalmanFilter::setObservationNoiseCov(MatrixXd observationNoiseCov)
{
    this->observationNoiseCov = observationNoiseCov;
}
