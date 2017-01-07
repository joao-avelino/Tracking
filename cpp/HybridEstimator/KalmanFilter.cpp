#include "KalmanFilter.hpp"
#include "Kfutils.hpp"
#include <eigen3/Eigen/Eigenvalues>
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
 *  [3] Grewal, M. S. (2011). Kalman filtering theory and practice using MATLAB.
 *      Springer Berlin Heidelberg.
 *
 *
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
 *  Performs the Kalman Filter predict step using the Modified weighted Gram—Schmidt method (MWGS)
 *  proposed by Thornton and Bierman (based on Matlab code provided by [3]).
 *
 *  Predict when a control signal is available
 */


void KalmanFilter::predict(VectorXd &controlVector)
{
    //Normal Kalman predict with control
    statePred = stateTransitionModel*statePost+controlInputModel*controlVector;


    //MWGS
    //covPred = stateTransitionModel*covPost*stateTransitionModel.transpose()+processNoiseCov;
    //this is just the usual
    //kf equation. not using it anymore

    int n = statePost.size();

    SelfAdjointEigenSolver<MatrixXd> selfSolver;
    selfSolver.compute(processNoiseCov);

    //Get a diagonal matrix
    VectorXd diagQ = selfSolver.eigenvalues();
    MatrixXd G_ = selfSolver.eigenvectors().transpose();

    MatrixXd U_ = MatrixXd::Identity(n,n);

    std::cout << "Here 0" << std::endl;

    MatrixXd PhiU_ = stateTransitionModel*U_post;

    std::cout << "Here 0.5" << std::endl;

    for (int i = n - 1; i >= 0; i--)
    {
        double sigma = 0;

        std::cout << "Here 1" << std::endl;

        VectorXd PhiU_row_square = PhiU_.row(i).array().square();

        std::cout << "Here 2" << std::endl;

        sigma = PhiU_row_square.transpose()*diagQ;

        std::cout << "Here 3" << std::endl;


        VectorXd G_row_squared = G_.row(i).array().square();

        std::cout << "Here 4" << std::endl;

        sigma = sigma + G_row_squared.transpose()*diagQ;

        std::cout << "Here 5" << std::endl;

        D_pred(i, i) = sigma;

        std::cout << "Here 6" << std::endl;


        for (int j = 0; j < i; j++)
        {
            sigma = 0;
            std::cout << "Here 7" << std::endl;

            VectorXd aux = PhiU_.row(j).array()*D_post.array();

            std::cout << "Here 8" << std::endl;

            sigma = aux.transpose()*PhiU_.row(j).transpose();

            std::cout << "Here 9" << std::endl;


            VectorXd aux2 = G_.row(i).array()*diagQ.array();

            std::cout << "Here 10" << std::endl;

            sigma = sigma + aux2.transpose()*G_.row(j).transpose();

            std::cout << "Here 11" << std::endl;


            U_(j, i) = sigma / D_post(i, i);

            std::cout << "Here 12" << std::endl;


            PhiU_.row(j) = PhiU_.row(j) - U_(j, i)*PhiU_.row(i);

            std::cout << "Here 13" << std::endl;


            G_.row(j) = G_.row(j) - U_(j, i)*G_.row(i);

            std::cout << "Here 14" << std::endl;

        }

    }

    U_pred = U_;
	
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
    //covPred = stateTransitionModel*covPost*stateTransitionModel.transpose()+processNoiseCov;
    //this is just the usual
    //kf equation. not using it anymore

    int n = statePost.size();

    SelfAdjointEigenSolver<MatrixXd> selfSolver;
    selfSolver.compute(processNoiseCov);

    std::cout << "processNoiseCov: " << processNoiseCov << std::endl;

    //Get a diagonal matrix
    VectorXd diagQ = selfSolver.eigenvalues();
    MatrixXd G_ = selfSolver.eigenvectors().transpose();

    std::cout << "G: " << G_ << std::endl;
    std::cout << "invG: " << G_.transpose() << std::endl;

    std::cout << "Test: " << processNoiseCov - G_*diagQ.asDiagonal()*G_.transpose();

    G_ << -0.9701, 0, 0.2425, 0,
            0, 0.9701, 0, -0.2425,
            0, 0.2425, 0, 0.9701,
            -0.2425, 0, -0.9701, 0;

    diagQ << 0, 0, 0.2656, 0.2656;

    MatrixXd U_ = MatrixXd::Identity(n,n);


    D_pred = VectorXd::Zero(n);

    MatrixXd PhiU_ = stateTransitionModel*U_post;

    VectorXd PhiU_row_square = VectorXd::Zero(PhiU_.cols());
    VectorXd G_row_squared = VectorXd::Zero(G_.cols());

    for i=n:-1:1,
       sigma = 0;

       sigma = PhiU(i,:).^2*diag(Din);
       sigma = sigma+G(i,:).^2*diag(Q);

       D(i,i) = sigma;
       for j=1:i-1,
          sigma = 0;
          sigma = PhiU(i,:).*diag(Din)'*PhiU(j,:)';
          sigma = sigma+G(i,:).*diag(Q)'*G(j,:)';

          U(j,i) = sigma/D(i,i);
          PhiU(j,:) = PhiU(j,:)-U(j,i)*PhiU(i,:);
          %
          G(j,:)=G(j,:)-U(j,i)*G(i,:);
       end;
    end;



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
