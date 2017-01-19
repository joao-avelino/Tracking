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

/**
 * @brief KalmanFilter Constructor
 * @details Constructs a Bierman-Thornton classical Kalman Filter object
 * 
 * @param[in] stateTransitionModel The state transition matrix, usually denoted by $\mathbf{\Phi}$
 * @param[in] observationModel The observation matrix usually denoted by $\mathbf{H}$
 * @param[in] processNoiseCov The process noise covariance matrix, usually denoted by $\mathbf{Q}$
 * @param[in] observationNoiseCov The observation noise covariance matrix, usually denoted by $\mathbf{R}$
 * @param[in] initialState The initial state of the Kalman Filter
 * @param[in] initialCov The initial covariance matrix $\mathbf{P}_0$. This matrix should have big numbers
 */

KalmanFilter::KalmanFilter(MatrixXd stateTransitionModel, MatrixXd observationModel,
             MatrixXd processNoiseCov, MatrixXd observationNoiseCov, VectorXd initialState, MatrixXd initialCov)
{
    this->stateTransitionModel = stateTransitionModel;
    this->controlInputModel = MatrixXd::Zero(initialState.size(), 1);
    this->observationModel = observationModel;
    this->processNoiseCov = processNoiseCov;
    this->observationNoiseCov = observationNoiseCov;
    this->statePost = initialState;
	this->statePred = initialState;
    this->covPost = initialCov;
	this->covPred = initialCov;

	KFUtils::uduFactorization(initialCov, U_post, D_post);

	U_pred = U_post;
	D_pred = D_post;

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

	KFUtils::uduFactorization(initialCov, U_post, D_post);
}



/**
 * @brief KalmanFilter::predict
 *  
 * @details Performs the Kalman Filter predict step using the Modified weighted Gram—Schmidt 
 *  method (MWGS) proposed by Thornton and Bierman (based on Matlab code provided by [3]).
 * 
 * @param[in] controlVector The control vector. If no control signal
 *  exits this method can be called with no arguments.
 */


void KalmanFilter::predict(VectorXd &controlVector)
{
    //Normal Kalman predict with control
	if (controlVector.size() > 0)
		statePred = stateTransitionModel*statePost + controlInputModel*controlVector;
	else
		statePred = stateTransitionModel*statePost;

    //covPred = stateTransitionModel*covPost*stateTransitionModel.transpose()+processNoiseCov;
    //this is just the usual
    //kf equation. not using it anymore

	//MWGS
	mwgs();

    //For safety. When no measurement is available the behaviour should be the same by either
    //not calling the update method or calling it with an empty measurement
	U_post = U_pred;
	D_post = D_pred;
	statePost = statePred;
	
}

/**
 * @brief KalmanFilter::mwgs
 *  
 * @details Performs the Modified weighted Gram—Schmidt method (MWGS) proposed by Thornton 
 *  and Bierman (based on Matlab code provided by [3]).
 */

void KalmanFilter::mwgs()
{

	int n = statePost.size();

	SelfAdjointEigenSolver<MatrixXd> selfSolver;
	selfSolver.compute(processNoiseCov);

	//Get a diagonal matrix
	VectorXd diagQ = selfSolver.eigenvalues();
	MatrixXd G_ = selfSolver.eigenvectors().transpose();

	U_pred = MatrixXd::Identity(n, n);
	D_pred = VectorXd::Zero(n);

	MatrixXd PhiU_ = stateTransitionModel*U_post;

	VectorXd PhiU_row_square = VectorXd::Zero(PhiU_.cols());
	VectorXd G_row_squared = VectorXd::Zero(G_.cols());
	VectorXd PhiU_dot_Din = VectorXd::Zero(PhiU_.cols());
	VectorXd D_dot_Q = VectorXd::Zero(D_post.cols());

	for (int i = n - 1; i >= 0; i--)
	{
		double sigma = 0;

		PhiU_row_square = PhiU_.row(i).array().square();

		sigma = PhiU_row_square.transpose()*D_post;

		G_row_squared = G_.row(i).array().square();
		sigma = sigma + G_row_squared.transpose()*diagQ;

		D_pred(i) = sigma;
		for (int j = 0; j <i; j++)
		{
			sigma = 0;

			PhiU_dot_Din = PhiU_.row(i).cwiseProduct(D_post.transpose());
			sigma = PhiU_dot_Din.transpose()*PhiU_.row(j).transpose();

			D_dot_Q = G_.row(i).cwiseProduct(diagQ.transpose());
			sigma = sigma + D_dot_Q.transpose()*G_.row(j).transpose();

			U_pred(j, i) = sigma / D_pred(i);
			PhiU_.row(j) = PhiU_.row(j) - U_pred(j, i)*PhiU_.row(i);

			G_.row(j) = G_.row(j) - U_pred(j, i)*G_.row(i);
		}
	}

	covPred = U_pred*D_pred.asDiagonal()*U_pred.transpose();

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
 * If no measurement is available it does nothing.
 *
 * @param[in] measureVector The measurement vector. Can be called with no arguments
 */

void KalmanFilter::update(VectorXd &measureVector)
{

    //Decorrelate data
    MatrixXd uncorrH;
    VectorXd uncorrData;
    MatrixXd uncorrR;

    KFUtils::decorrelateData(measureVector, uncorrData, observationModel, uncorrH, observationNoiseCov, uncorrR);

    int obsDim = measureVector.size();

	if (obsDim < 1)
	{
		U_post = U_pred;
		D_post = D_pred;
		statePost = statePred;
		return;
	}
		

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

    statePost = statePred;

}

/**
 * @brief KalmanFilter::getCovPost
 * @details Gets the state covariance matrix after
 * the update step
 * 
 * @return The state covariance matrix $\mathbf{P}_{k|k}$
 */

MatrixXd KalmanFilter::getCovPost()
{
	covPost = U_post*D_post.asDiagonal()*U_post.transpose();
	return covPost;
}

/**
 * @brief KalmanFilter::getCovPred
 * @details Gets the state covariance matrix after the
 *  prediction set
 * 
 * @return The state covariance matrix $\mathbf{P}_{k|k-1}$
 */

MatrixXd KalmanFilter::getCovPred()
{
	covPred = U_pred*D_pred.asDiagonal()*U_pred.transpose();
	return covPred;
}

/**
 * @brief KalmanFilter::getMeasurementResidual
 * @details Computes the measurement residual
 * 
 * @param measure A vector with a measurement
 * @return The residue $\mathbf{r}_k = \mathbf{z} - \mathbf{H}*\mathbf{x}_{k|k-1}
 */

VectorXd KalmanFilter::getMeasurementResidual(VectorXd & measure)
{
	return measure-observationModel*statePred;
}

MatrixXd KalmanFilter::getResidualCovariance()
{
	return observationModel*U_pred*D_pred*U_pred.transpose()*observationModel.transpose()+observationNoiseCov;
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
