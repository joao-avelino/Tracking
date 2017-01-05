#include "KalmanFilter.hpp"
#include "Kfutils.hpp"
#include <iostream>

/**
 * @file KalmanFilter.cpp
 *
 * Copyright (C) 2016, Joao Avelino
 *
 * This class implements a linear Kalman Filter based on Bierman and Thornton's U-D
 * filter algorithm [1] for improved numerical stability. The implementation
 * is based on the guidelines provided by [2] and code provided by [3].
 *
 *  Refs:
 *  [1] Bierman, G.J. and C.L. Thornton (1977) Numerical comparison of Kalman filter
 *      algorithms: Orbit determination case study. Automatica, 13:23–35
 *  [2] Gibbs, B. P. (2011). Advanced Kalman filtering, least-squares and modeling:
 *      a practical handbook. John Wiley & Sons.
 *  [3] Grewal, M. S., Weill, L. R., & Andrews, A. P. (2007). Global positioning systems,
 *      inertial navigation, and integration. John Wiley & Sons.
 *
 *  License stuff
 */


KalmanFilter::KalmanFilter(MatrixXd stateTransitionModel, MatrixXd observationModel,
             MatrixXd processNoiseCov, MatrixXd observationNoiseCov, VectorXd initialState, MatrixXd initialCov)
{
    this->stateTransitionModel = stateTransitionModel;
    this->controlInputModel = MatrixXd::Zero(statePost.size(), 1);
    this->observationModel = observationModel;
    this->processNoiseCov = processNoiseCov;
    this->observationNoiseCov = observationNoiseCov;
    this->statePost = initialState;
    this->covPost = initialCov;
}

KalmanFilter::KalmanFilter(MatrixXd stateTransitionModel, MatrixXd controlInputModel, MatrixXd observationModel,
                           MatrixXd processNoiseCov, MatrixXd observationNoiseCov,
                           VectorXd initialState, MatrixXd initialCov)
{

    this->stateTransitionModel = stateTransitionModel;
    this->controlInputModel = controlInputModel;
    this->observationModel = observationModel;
    this->processNoiseCov = processNoiseCov;
    this->observationNoiseCov = observationNoiseCov;
    this->statePost = initialState;
    this->covPost = initialCov;

}


/**
 * @brief KalmanFilter::predict
 *
 *  Performs the Kalman Filter predict step using the Modified weighted Gram—Schrnidt method (MWGS)
 *  proposed by Thornton and Bierman (based on Matlab code provided by [3]).
 *
 *  Predict when a control signal is available
 */


void KalmanFilter::predict(VectorXd &controlVector)
{
    //Normal Kalman predict with control
    statePred = stateTransitionModel*statePost+controlInputModel*controlVector;


    //MWGS
    covPred = stateTransitionModel*covPost*stateTransitionModel.transpose()+processNoiseCov;

    int n = statePost.size();
    int r = processNoiseCov.cols();

    MatrixXd G_;
    MatrixXd U_ = MatrixXd::Identity(n,n);
    MatrixXd PhiU_ = stateTransitionModel*U_post;

}

/**
 * @brief KalmanFilter::predict
 *
 *  Performs the Kalman Filter predict step using the Modified weighted Gram—Schrnidt method (MWGS)
 *  proposed by Thornton and Bierman (based on Matlab code provided by [3]).
 *
 *  Predict when NO control signal is available
 */



void KalmanFilter::predict()
{
    //Normal Kalman predict without control
    statePred = stateTransitionModel*statePost;

    //MWGS
    covPred = stateTransitionModel*covPost*stateTransitionModel.transpose()+processNoiseCov;



}

/**
 * Performs the Kalman Filter update step using Bierman's method.
 * (see [2], pages 394-396)
 *
 * Note that the state update is incrementally updated, meaning that
 * for each SCALAR measurement (each uncorrelated measurement dimension)
 * the results of the previous SCALAR measurement are used (U, D, and x_pred).
 *
 * (This is not so obvious from the book...)
 *
 * for each measurement:
 *   x_pred = x_pred + k(observation - h*x_pred)
 * end for
 *
 * x_post = x_pred
 *
 *
 * @param[in] measureVector The measurement vector
 */



void KalmanFilter::update(VectorXd &measureVector)
{


    //JUST FOR TESTING


    KFUtils::uduFactorization(covPred, U_pred, D_pred);

    // -----------------------------

    //Decorrelate data
    MatrixXd uncorrH;
    VectorXd uncorrData;
    MatrixXd uncorrR;

    KFUtils::decorrelateData(measureVector, uncorrData, observationModel, uncorrH, observationNoiseCov, uncorrR);

    int obsDim = measureVector.size();
    int stateDims = statePost.size();

    kalmanGain = MatrixXd::Zero(stateDims, obsDim);

    U_post = U_pred;
    D_post = D_pred;

    //For each observation variable
    for(int i=0; i<obsDim; i++)
    {
        //Compute auxiliary variables
        VectorXd h = uncorrH.row(i);
        double Ri = uncorrR(i,i);

        VectorXd k;
        update_single_meas(h, Ri, k);
        kalmanGain.col(i) = k;
        statePred = statePred +k*(uncorrData(i)-h.transpose()*statePred);
    }

    covPost = U_post*D_post.asDiagonal()*U_post.transpose();
    statePost = statePred;

}

/**
 * U-D update step on single scalar value
 *
 * Returns a column of the overall Kalman Gain vector and computes the new U and D
 *
 * @param[in] h Uncorrelated observation row vector from the observation matrix H
 * @param[in] Ri Uncorrelated measurement variance
 * @param[out] k Kalman Gain vector
 */

void KalmanFilter::update_single_meas(const VectorXd &h, const double Ri, VectorXd &k)
{

    VectorXd gamma = VectorXd::Zero(h.size());
    MatrixXd Ubar = MatrixXd::Zero(U_post.rows(), U_post.cols());
    VectorXd Dbar = VectorXd::Zero(D_post.size());
    k = VectorXd::Zero(h.size());
    VectorXd f = U_post.transpose()*h;
    VectorXd g = D_post.asDiagonal()*f;

    double alpha = f.transpose()*g + Ri;

    gamma(0) = Ri+g(0)*f(0);


    Dbar(0) = D_post(0)*Ri/gamma(0);
    Ubar(0,0) = 1;
    k(0) = g(0);

    for(int j = 1; j< h.size(); j++)
    {
        gamma(j) = gamma(j-1)+g(j)*f(j);
        Dbar(j)= D_post(j)*gamma(j-1)/gamma(j);
        Ubar.col(j) = U_post.col(j)-(f(j)/gamma(j-1))*k;
        k = k+g(j)*U_post.col(j);
    }

    k = k/alpha;
    U_post = Ubar;
    D_post = Dbar;

    return;
}
