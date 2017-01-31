#ifndef BASEBAYESIANFILTER_HPP
#define BASEBAYESIANFILTER_HPP

#include <memory>
#include <eigen3/Eigen/Core>

using namespace Eigen;

static VectorXd EMPTYVEC = VectorXd();

class BaseBayesianFilter
{
public:
	BaseBayesianFilter() {};
	~BaseBayesianFilter() {};

    virtual void predict(VectorXd &controlVector) = 0;
    virtual void update(VectorXd &measureVector) = 0;

    void predict()
    {
        predict(EMPTYVEC);
    };


    void update()
    {
        update(EMPTYVEC);
    };

	std::string getModelName()
	{
		return modelName;
	};

	void setModelName(std::string newName)
	{
		modelName = newName;
	};

	virtual VectorXd getStatePred() = 0;
	virtual MatrixXd getCovPred() = 0;
	virtual VectorXd getMeasurementResidual(VectorXd &measure) = 0;
	virtual MatrixXd getResidualCovariance() = 0;
	virtual VectorXd getStatePost() = 0;
	virtual MatrixXd getCovPost() = 0;

	virtual std::shared_ptr<BaseBayesianFilter> clone() = 0;

protected:
	std::string modelName;

};

#endif //BASEBAYESIANFILTER_HPP
