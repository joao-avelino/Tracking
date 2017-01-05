#include <iostream>
#include "KalmanFilter.hpp"

#include "Kfutils.hpp"

using namespace std;

int main()
{


    MatrixXd P;
    MatrixXd U;
    VectorXd D;

    P = MatrixXd(3,3);

    P << 100, 30, 30,
            30, 115, 20,
            30, 20, 123;

    KFUtils::uduFactorization(P, U, D);

    cout << "U: " << U <<endl;
    MatrixXd D_mat = D.asDiagonal();
    cout << "Dif: " << P-U*D_mat*U.transpose() << endl;

    VectorXd correlatedData(2);
    correlatedData << 4.1864, 1.9174;
    VectorXd uncorrelatedData;
    MatrixXd H_normal(2,4);
    H_normal << 1, 0, 0, 0,
            0, 1, 0, 0;

    MatrixXd H_uncorr;
    MatrixXd R = MatrixXd(2,2);
    R << 1.5083, 0.8623,
            0.8623, 0.4965;

    MatrixXd uncorrR;

    KFUtils::decorrelateData(correlatedData,uncorrelatedData,
                                  H_normal,H_uncorr,
                                  R,uncorrR);


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    MatrixXd stateTransitionModel(4,4);
    stateTransitionModel << 1, 0, 0.5, 0,
            0, 1, 0, 0.5,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd observationModel(2,4);
    observationModel << 1, 0, 0, 0,
            0, 1, 0, 0;

    double T = 0.5;

    MatrixXd processNoiseCovariance(4,4);
    processNoiseCovariance << pow(T, 4)/4, 0, pow(T,3)/2, 0, 0, pow(T, 4)/4, 0, pow(T,3)/2, pow(T,3)/2, 0, pow(T,2), 0, 0, pow(T,3)/2, 0, pow(T,2);

    MatrixXd observationNoiseCov(2,2);
    observationNoiseCov << 1.5083, 0.4  ,
            0.4, 0.4965;

    VectorXd initial_state(4);
    initial_state << 10,10,10,10;

    MatrixXd initial_cov(4,4);
    initial_cov << 2000, 10, 10, 10,
            10, 2000, 10, 10,
            10, 10, 2000, 10,
            10, 10, 10, 2000;

    KalmanFilter kf(stateTransitionModel, observationModel, processNoiseCovariance, observationNoiseCov,
                    initial_state, initial_cov);

    kf.predict();

    std::cout << "Check" << std::endl;

    VectorXd meas(2);
    meas << 10, 5;

    kf.update(meas);

    return 0;
}

