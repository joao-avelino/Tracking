#ifndef BASEKALMANFILTER_HPP
#define BASEKALMANFILTER_HPP

#include <eigen3/Eigen/Core> //Not sure about dense.
#include "BaseBayesianFilter.hpp"

using namespace Eigen;

class BaseKalmanFilter : public BaseBayesianFilter
{
public:

    virtual void predict(VectorXd &controlVector=VectorXd()) = 0;
    virtual void update(VectorXd &measureVector= VectorXd()) = 0;

    //If we have no measurement, then don't perform the update step (just update the statePost and covPost
    //with the previous one)
    void update();

    //Getters
    VectorXd getStatePred();
    MatrixXd getCovPred();
    virtual VectorXd getMeasurementResidual(VectorXd &measure) = 0;
    virtual MatrixXd getResidualCovariance() = 0;
    VectorXd getStatePost();
    MatrixXd getCovPost();

    MatrixXd getKalmanGain();

    //Setters
    void setDeltaT(double deltaT);
    void setObservationNoiseCov(MatrixXd observationNoiseCov);

protected:

    VectorXd statePred;
    MatrixXd covPred;
    MatrixXd kalmanGain;
    VectorXd statePost;
    MatrixXd covPost;

    MatrixXd processNoiseCov;
    MatrixXd observationNoiseCov;

    double deltaT;

};

#endif // BASEKALMANFILTER_HPP
