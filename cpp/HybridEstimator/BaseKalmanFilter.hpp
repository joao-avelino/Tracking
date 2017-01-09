#ifndef BASEKALMANFILTER_HPP
#define BASEKALMANFILTER_HPP

#include <eigen3/Eigen/Core> //Not sure about dense.

using namespace Eigen;

class BaseKalmanFilter
{
public:

    virtual void predict(VectorXd &controlVector) = 0;
    virtual void predict() = 0;
    virtual void update(VectorXd &measureVector) = 0;

    //If we have no measurement, then don't perform the update step (just update the statePost and covPost
    //with the previous one)
    void update();

    //Getters
    VectorXd getStatePred();
    MatrixXd getCovPred();
    VectorXd getMeasurementResidual();
    MatrixXd getResidualCovariance();
    MatrixXd getKalmanGain();
    VectorXd getStatePost();
    MatrixXd getCovPost();

    //Setters
    void setDeltaT(double deltaT);
    void setObservationNoiseCov(MatrixXd observationNoiseCov);

protected:

    VectorXd statePred;
    MatrixXd covPred;
    VectorXd measurementResidual;
    MatrixXd residualCovariance;
    MatrixXd kalmanGain;
    VectorXd statePost;
    MatrixXd covPost;

    MatrixXd processNoiseCov;
    MatrixXd observationNoiseCov;

    double deltaT;

};

#endif // BASEKALMANFILTER_HPP
