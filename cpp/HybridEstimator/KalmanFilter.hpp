#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include "BaseKalmanFilter.hpp"

/**
 * @file KalmanFilter.hpp
 * Copyright (C) 2016, Joao Avelino
 *
 * This class implements a linear Kalman Filter based on Bierman and Thornton's U-D
 * filter algorithm [1] for improved numerical stability. The implementation
 * is based on the guidelines provided by [2].
 *
 *  Refs:
 *  [1] Bierman, G.J. and C.L. Thornton (1977) Numerical comparison of Kalman filter
 *      algorithms: Orbit determination case study. Automatica, 13:23–35
 *  [2] Gibbs, B. P. (2011). Advanced Kalman filtering, least-squares and modeling:
 *      a practical handbook. John Wiley & Sons.
 *
 *  License stuff
 */


class KalmanFilter : public BaseKalmanFilter
{
public:

    KalmanFilter(MatrixXd stateTransitionModel, MatrixXd observationModel,
                 MatrixXd processNoiseCov, MatrixXd observationNoiseCov, VectorXd initialState, MatrixXd initialCov);

    KalmanFilter(MatrixXd stateTransitionModel, MatrixXd controlInputModel, MatrixXd observationModel,
                 MatrixXd processNoiseCov, MatrixXd observationNoiseCov, VectorXd initialState, MatrixXd initialCov);



    void predict(VectorXd &controlVector);
	void predict()
	{
		VectorXd empty = VectorXd();
		predict(empty);
	}
	void mwgs();
    void update(VectorXd &measureVector);
	void update()
	{
		VectorXd empty = VectorXd();
		update(empty);
	}

	MatrixXd getCovPost();
	MatrixXd getCovPred();

	VectorXd getMeasurementResidual(VectorXd &measure);
	MatrixXd getResidualCovariance();

protected:

    MatrixXd stateTransitionModel;
    MatrixXd controlInputModel;
    MatrixXd observationModel;


private:

    MatrixXd U_pred;
    VectorXd D_pred;
    MatrixXd U_post;
    VectorXd D_post;

    void update_single_meas(const VectorXd &h, const double Ri, VectorXd &k);



};

#endif // KALMANFILTER_HPP
