#ifndef EIGEN2MATLAB_HPP
#define EIGEN2MATLAB_HPP

#include <eigen3\Eigen\Core>
#include "mex.h"

using namespace Eigen;

VectorXd matlabVectorToEigen(const mxArray *vectorArray);

MatrixXd matlabMatrixToEigen(const mxArray *matrixArray);

mxArray * eigenVectorToMatlab(const VectorXd &vector);

mxArray * eigenMatrixToMatlab(const MatrixXd &matrix);

#endif // !EIGEN2MATLAB_HPP
