#include "MMAEItem.hpp"
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>
#include <cmath>

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286


MMAEItem::MMAEItem(std::shared_ptr<BaseBayesianFilter> model, std::string modelName)
{
	this->filter = model;
	this->modelName = modelName;
	this->stateDim = model->getStatePost().size();
}

MMAEItem::~MMAEItem()
{
}

void MMAEItem::predict(VectorXd &control)
{
	filter->predict(control);
}

void MMAEItem::update(VectorXd & measurement)
{
	filter->update(measurement);
}

void MMAEItem::computeProbabilityDensity(VectorXd & measurement)
{

	//Deal with the case where we have no measurement
	if (measurement.size() < 1)
	{
		//Do nothing. The probabilities are the same as before since we don't have new information
		return;
	}
	else
	{
		//Note that the Residual Covariance matrix will converge. So after a while we don't need to waste compational power computing it.
		//Still need to get a way to do this

		MatrixXd Ak = filter->getResidualCovariance();
		double detAk = Ak.determinant();
		VectorXd rk = filter->getMeasurementResidual(measurement);
		int m = measurement.size();
		double beta = 1.0 / (pow(2*PI, m/2)*sqrt(detAk));

		//rk'*A^(-1)*rk
		double scalarLikelihood = rk.transpose()*(Ak.colPivHouseholderQr().solve(rk));
		
		//pdf(measurement)
		double probDensity = beta*exp((-1.0 / 2.0)*scalarLikelihood);

	}

}

double MMAEItem::getProbDensity()
{
	return probDensity;
}

double MMAEItem::getProbability()
{
	return probability;
}

void MMAEItem::setProbabiliy(double prob)
{
	probability = prob;
}

double MMAEItem::getStateDim()
{
	return stateDim;
}
