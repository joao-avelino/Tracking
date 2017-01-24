#ifndef HISTOGRAMCOMPARATOR_HPP
#define HISTOGRAMCOMPARATOR_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

using namespace Eigen;

namespace Comparator
{
	//Metric consts
	static const int METRIC_MAHALANOBIS = 0;
	static const int METRIC_EUCLIDEAN = 1;
	static const int METRIC_CORRELATION = 2;
	static const int METRIC_INTERSECTION = 3;
	static const int METRIC_CHISQUARED = 4;
	static const int METRIC_HELLINGER = 5;
	static const int METRIC_EARTHMOVERSDISTANCE = 6;

	double compareHist1D(const VectorXd &hist1, const VectorXd &hist2, int metric);
	double euclidean(const VectorXd &x, const VectorXd &y);
	double mahalanobis(const VectorXd &x, const VectorXd &u, const MatrixXd covariance);
	double hellinger(const VectorXd & h1, const VectorXd & h2);
};

#endif //HISTOGRAMCOMPARATOR_HPP