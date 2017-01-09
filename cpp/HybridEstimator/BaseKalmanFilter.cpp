#include "BaseKalmanFilter.hpp"


void BaseKalmanFilter::update()
{
    statePost = statePred;
    covPost = covPred;

    return;
}

VectorXd BaseKalmanFilter::getStatePred()
{
    return statePred;
}

MatrixXd BaseKalmanFilter::getCovPred()
{
    return covPred;
}

VectorXd BaseKalmanFilter::getMeasurementResidual()
{
    return measurementResidual;
}

MatrixXd BaseKalmanFilter::getResidualCovariance()
{
    return residualCovariance;
}

MatrixXd BaseKalmanFilter::getKalmanGain()
{
    return kalmanGain;
}

VectorXd BaseKalmanFilter::getStatePost()
{
    return statePost;
}

MatrixXd BaseKalmanFilter::getCovPost()
{
    return covPost;
}

void BaseKalmanFilter::setDeltaT(double deltaT)
{
    this->deltaT = deltaT;
}

void BaseKalmanFilter::setObservationNoiseCov(MatrixXd observationNoiseCov)
{
    this->observationNoiseCov = observationNoiseCov;
}
