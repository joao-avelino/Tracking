#include "eigen2matlab.hpp"

VectorXd matlabVectorToEigen(const mxArray *vectorArray)
{
	int vectorM = mxGetM(vectorArray);
	int vectorN = mxGetN(vectorArray);
	double *vectorPTR = mxGetPr(vectorArray);

	VectorXd vec;

	//Column vector
	if (vectorM >= vectorN)
	{
		vec = VectorXd(vectorM);
	}//Row vector
	else
	{
		vec = RowVectorXd(vectorN);
	}

	for (int m = 0; m < vec.size(); m++)
	{
		vec(m) = vectorPTR[m];
	}

	return vec;
}

MatrixXd matlabMatrixToEigen(const mxArray *matrixArray)
{
	int matrixM = mxGetM(matrixArray);
	int matrixN = mxGetN(matrixArray);

	double *matrixPTR = mxGetPr(matrixArray);
	MatrixXd matrix(matrixM, matrixN);


	for (int m = 0; m < matrixM; m++)
	{
		for (int n = 0; n < matrixN; n++)
		{
			matrix(m, n) = matrixPTR[m + matrixM*n];
		}
	}

	return matrix;
}

mxArray * eigenVectorToMatlab(const VectorXd &vector)
{

	mxArray *matlabVectorArray = mxCreateDoubleMatrix(vector.rows(), vector.cols(), mxREAL);

	double *matlabVectorPTR = mxGetPr(matlabVectorArray);

	for (int m = 0; m < vector.size(); m++)
		matlabVectorPTR[m] = vector(m);

	return matlabVectorArray;

}

mxArray * eigenMatrixToMatlab(const MatrixXd &matrix)
{

	mxArray *matlabMatrixArray = mxCreateDoubleMatrix(matrix.rows(), matrix.cols(), mxREAL);

	double *matrixPTR = mxGetPr(matlabMatrixArray);


	for (int m = 0; m < matrix.rows(); m++)
		for (int n = 0; n < matrix.cols(); n++)
			matrixPTR[m + matrix.rows()*n] = matrix(m, n);

	return matlabMatrixArray;
}