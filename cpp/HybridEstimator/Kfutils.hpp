#ifndef KFUTILS_HPP
#define KFUTILS_HPP

#include <eigen3/Eigen/Core>

/*
 * Copyright (C) 2016, Joao Avelino
 *
 * Just an utility class to transform data and some matrices
 */

using namespace Eigen;

class KFUtils
{
public:

    static void decorrelateData(VectorXd correlatedData, VectorXd &uncorrelatedData,
                                MatrixXd H_normal, MatrixXd &H_uncorr, MatrixXd R, MatrixXd &uncorrR);

    static void uduFactorization(MatrixXd P, MatrixXd &U, VectorXd &D);

private:
	KFUtils() {};
};

#endif // KFUTILS_HPP
