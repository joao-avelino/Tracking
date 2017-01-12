#ifndef HYBRIDESTIMATOR_HPP
#define HYBRIDESTIMATOR_HPP

#include <eigen3/Eigen/Core>
#include "BaseKalmanFilter.hpp"
#include <vector>
using namespace Eigen;

/**
 * @file HybridEstimator.hpp
 *
 * Copyright (C) 2016, Joao Avelino
 *
 * This abstract class defines a Hybrid Estimator. It can be used to implement algorithms
 * that rely on multiple models and fuses their estimations on a probabilistic way, like
 * the Multiple Models Adaptive Estimator [1] or the Interactiv Multiple Models [2].
 *
 *  Refs:
 *  [1] 
 *  [2] 
 *
 *
 *
 *  License stuff
 */

class HybridEstimator
{
public:
    virtual VectorXd getStatePrediction() = 0;
    virtual VectorXd getStateCovariancePrediction() = 0;
    virtual VectorXd getStateEstimate() = 0;
    virtual VectorXd getStateCovariance() = 0;
    virtual void runEstimator() = 0;
    virtual void updateDeltaT(double deltaT) = 0;
    virtual std::vector<double> getAllModelProbabilities() = 0;
    HybridEstimator(double deltaT, std::vector<BaseKalmanFilter> modelList);

protected:

    std::vector<BaseKalmanFilter> modelList;
    double deltaT;
};

#endif // HYBRIDESTIMATOR_HPP
