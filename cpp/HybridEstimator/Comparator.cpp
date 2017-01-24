#include "Comparator.hpp"
#include <iostream>

double Comparator::compareHist1D(const VectorXd & hist1, const VectorXd & hist2, int metric)
{
	switch (metric)
	{
	case(METRIC_EUCLIDEAN):
		
		return sqrt((hist1 - hist2).transpose()*(hist1 - hist2));
		break;

	case(METRIC_CORRELATION):
		//IMPLEMENTAR
		break;

	case(METRIC_INTERSECTION):
		//IMPLEMENTAR
		break;

	case(METRIC_CHISQUARED):
		//IMPLEMENTAR
		break;

	case(METRIC_HELLINGER):
		assert(hist1.size() == hist2.size() && "I'm sorry. I can't compare histograms of different sizes with the hellinger metric");

		return hellinger(hist1, hist2);
		break;

	case(METRIC_EARTHMOVERSDISTANCE):
		//IMPLEMENTAR
		break;

	default:
		break;
	}

	return -1;
}

double Comparator::euclidean(const VectorXd & x, const VectorXd & y)
{
	VectorXd residue;
	residue = x - y;

	return sqrt(residue.transpose()*residue);
}

double Comparator::mahalanobis(const VectorXd & x, const VectorXd & u, const MatrixXd covariance)
{
	VectorXd residue = x - u;
	VectorXd s_backslash_residue = covariance.colPivHouseholderQr().solve(residue);
	double distance_squared = residue.transpose()*s_backslash_residue;

	return sqrt(distance_squared);

}

double Comparator::hellinger(const VectorXd &h1, const VectorXd &h2)
{
	double h1_bar = h1.sum() / h1.size();
	double h2_bar = h2.sum() / h2.size();
	double N = h1.size();

	VectorXd sqrtH1 = h1.cwiseSqrt();
	VectorXd sqrtH2 = h2.cwiseSqrt();

	double sumOfSqrtMults = sqrtH1.transpose()*sqrtH2;
	double normalizationFactor = sqrt(h1_bar*h2_bar*N*N);

	return sqrt(1.0-sumOfSqrtMults/ normalizationFactor);

}
