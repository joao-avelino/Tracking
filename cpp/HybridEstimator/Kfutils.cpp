#include "Kfutils.hpp"
#include <eigen3/Eigen/QR>

#include <iostream>

/**
 * @file   Kfutils.cpp
 *
 * Copyright (C) 2016, Joao Avelino
 *
 * Just an utility class to transform data and matrices
 * for Kalman Filer implementations
 *
 * License stuff
 */


/**
 *
 * Performs observation decorrelation allowing the performance
 * of KF update steps with scalars
 *
 * @param[in] correlatedData The original (possibly) correlated data
 * @param[in] H_normal The observation matrix for correlated data
 * @param[in] R The original observation covariance matrix
 * @param[out] uncorrelatedData UNcorrelated observation data
 * @param[out] H_uncorr The observation matrix for the UNcorrelated data
 * @param[out] uncorrR The uncorrelated data covariance matrix
 */



void KFUtils::decorrelateData(VectorXd correlatedData, VectorXd &uncorrelatedData,
                              MatrixXd H_normal, MatrixXd &H_uncorr,
                              MatrixXd R, MatrixXd &uncorrR)
{

    //Make sure the covariance matrix is squared
    assert(R.rows() == R.cols() && "Matrix must be squared");

    //Make sure it has the same dimensions of the data
    assert(correlatedData.size() == R.cols() && "The correlation matrix must match the data...");


    if(R == (MatrixXd) R.diagonal().asDiagonal())
    {
        uncorrelatedData = correlatedData;
        H_uncorr = H_normal;
        uncorrR = R;
    }

    MatrixXd unCorrMat;
    VectorXd newRvect;

    uduFactorization(R, unCorrMat, newRvect);
    uncorrR = newRvect.asDiagonal();

    uncorrelatedData = unCorrMat.colPivHouseholderQr().solve(correlatedData);

    H_uncorr = unCorrMat.colPivHouseholderQr().solve(H_normal);

    return;

}


/**
 *Performs UDU' factorization of a symmetric matrix
 *
 *
 * @param[in] P The original matrix we want to factor.
 * @param[out] U The U matrix
 * @param[out] D A vector containing the diagonal values of the D matrix
 */

void KFUtils::uduFactorization(MatrixXd P, MatrixXd &U, VectorXd &D)
{
    //Make sure it's squared
    assert(P.rows() == P.cols() && "Matrix must be squared");

    //Make sure it's symmetric
    assert(P == P.transpose() && "Matrix must be symmetric");

    //Get P dimenstions
    int nDims = P.rows();

    int END = nDims-1;

    //Initialize U and D
    U = MatrixXd::Zero(nDims,nDims);
    D = VectorXd::Zero(nDims);

    D(END) = P(END, END);

    U.col(END) = P.col(END)/P(END,END);

    for(int j = END-1; j>=0; j--)
    {
        VectorXd aux = D.segment(j+1, END-j).array()*U.block(j, j+1, 1, END-j).array().square().transpose();
        D(j) = P(j, j) - aux.sum();
        U(j, j) = 1;
        for(int i= j-1; i>=0; i--)
        {
            VectorXd aux2 = D.segment(j+1, END-j).array()*U.block(i, j+1, 1, END-j).array().transpose()*U.block(j, j+1, 1, END-j).array().transpose();
            U(i, j) = (P(i, j) - aux2.sum())/D(j);
        }

    }
}
