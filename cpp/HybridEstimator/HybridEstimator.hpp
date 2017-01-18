#ifndef HYBRIDESTIMATOR_HPP
#define HYBRIDESTIMATOR_HPP

#include <eigen3/Eigen/Core>
#include "MMBankItem.hpp"
#include "MMBankItem.hpp"
#include <vector>
#include <memory>

using namespace Eigen;

/**
 * @file HybridEstimator.hpp
 *
 * Copyright (C) 2016, Joao Avelino
 *
 * This abstract class defines a Hybrid Estimator. It can be used to implement algorithms
 * that rely on multiple models and fuses their estimations on a probabilistic way, like
 * the Multiple Models Adaptive Estimator [1] or the Interactive Multiple Models [2].
 *
 *  Refs:
 *  [1] Hanlon, P. D., & Maybeck, P. S. (2000). Multiple-model adaptive estimation 
 *	   using a residual correlation Kalman filter bank. IEEE Transactions on Aerospace 
 *	   and Electronic Systems, 36(2), 393-406.
 *  [2] Mazor, Efim, et al. "Interacting multiple model methods in target tracking: a survey." 
 *		IEEE transactions on aerospace and electronic systems 34.1 (1998): 103-123.
 *
 *
 *
 *  License stuff
 */

class HybridEstimator
{
public:
    virtual VectorXd getStatePrediction() = 0;
    virtual MatrixXd getStateCovariancePrediction() = 0;
    virtual VectorXd getStatePosterior() = 0;
    virtual MatrixXd getStateCovariancePosterior() = 0;
    virtual void predict(VectorXd &control = VectorXd()) = 0;
	virtual void update(VectorXd &measure = VectorXd()) = 0;
    virtual void updateDeltaT(double deltaT) = 0;
    virtual std::vector<double> getAllModelProbabilities() = 0;
    HybridEstimator();

};

#endif // HYBRIDESTIMATOR_HPP
