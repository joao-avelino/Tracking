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

	virtual void update(VectorXd &measureVector, MatrixXd &detCov) = 0;

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

	int getObsDim()
	{
		return obsDim;
	}

	virtual VectorXd getStatePred() = 0;
	virtual MatrixXd getCovPred() = 0;
	virtual VectorXd getMeasurementResidual(VectorXd &measure) = 0;
	virtual MatrixXd getResidualCovariance() = 0;
	virtual VectorXd getStatePost() = 0;
	virtual MatrixXd getCovPost() = 0;

	virtual void setStatePred(VectorXd statePred) = 0;
	virtual void setCovPred(MatrixXd covPred) = 0;
	virtual void setStatePost(VectorXd statePost) = 0;
	virtual void setCovPost(MatrixXd covPost) = 0;

	virtual std::shared_ptr<BaseBayesianFilter> clone() = 0;
	virtual std::shared_ptr<BaseBayesianFilter> clone(VectorXd initial_state) = 0;
	virtual std::shared_ptr<BaseBayesianFilter> clone(VectorXd initial_state, MatrixXd measurementCov) = 0;

protected:
	std::string modelName;
	int obsDim;

};

#endif //BASEBAYESIANFILTER_HPP
