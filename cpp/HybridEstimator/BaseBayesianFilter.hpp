#ifndef BASEBAYESIANFILTER_HPP
#define BASEBAYESIANFILTER_HPP

#include <memory>
#include <eigen3/Eigen/Core>

using namespace Eigen;

class BaseBayesianFilter
{
public:
	BaseBayesianFilter();
	~BaseBayesianFilter();

	virtual void predict(VectorXd &controlVector = VectorXd()) = 0;
	virtual void update(VectorXd &measureVector = VectorXd()) = 0;

	std::string getModelName();
	void setModelName(std::string newName);

	virtual VectorXd getStatePred() = 0;
	virtual MatrixXd getCovPred() = 0;
	virtual VectorXd getMeasurementResidual(VectorXd &measure) = 0;
	virtual MatrixXd getResidualCovariance() = 0;
	virtual VectorXd getStatePost() = 0;
	virtual MatrixXd getCovPost() = 0;

protected:
	std::string modelName;


};

#endif //BASEBAYESIANFILTER_HPP